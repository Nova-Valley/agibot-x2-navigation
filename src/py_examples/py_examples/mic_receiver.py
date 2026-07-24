#!/usr/bin/env python3
"""
Microphone data receiving example

This example subscribes to the `/agent/process_audio_output` topic to receive the robot's
noise-suppressed audio data. It supports both the built-in microphone and the external
microphone audio streams, and automatically saves complete speech segments as PCM files
based on the VAD (Voice Activity Detection) state.

Features:
- Supports receiving multiple audio streams at the same time (built-in mic stream_id=1, external mic stream_id=2)
- Automatically detects speech start / in-progress / end based on VAD state
- Automatically saves complete speech segments as PCM files
- Stores files categorized by timestamp and audio stream
- Supports audio duration calculation and logging

VAD state description:
- 0: No speech
- 1: Speech start
- 2: Speech in progress
- 3: Speech end
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from aimdk_msgs.msg import ProcessedAudioOutput, AudioVadStateType
import os
import time
from datetime import datetime
from collections import defaultdict
from typing import Dict, List


class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')

        # Audio buffers, stored separately by stream_id
        # stream_id -> buffer
        self.audio_buffers: Dict[int, List[bytes]] = defaultdict(list)
        self.recording_state: Dict[int, bool] = defaultdict(bool)

        # Create audio output directory
        self.audio_output_dir = "audio_recordings"
        os.makedirs(self.audio_output_dir, exist_ok=True)

        # VAD state name mapping
        self.vad_state_names = {
            0: "No speech",
            1: "Speech start",
            2: "Speech in progress",
            3: "Speech end"
        }

        # Audio stream name mapping
        self.stream_names = {
            1: "Built-in microphone",
            2: "External microphone"
        }

        # QoS settings
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Create subscriber
        self.subscription = self.create_subscription(
            ProcessedAudioOutput,
            '/agent/process_audio_output',
            self.audio_callback,
            qos
        )

        self.get_logger().info("Start subscribing to noise-suppressed audio data...")

    def audio_callback(self, msg: ProcessedAudioOutput):
        """Audio data callback"""
        try:
            stream_id = msg.stream_id
            vad_state = msg.audio_vad_state.value
            audio_data = bytes(msg.audio_data)

            self.get_logger().info(
                f"Received audio data: stream_id={stream_id}, "
                f"vad_state={vad_state}({self.vad_state_names.get(vad_state, 'Unknown state')}), "
                f"audio_size={len(audio_data)} bytes"
            )

            self.handle_vad_state(stream_id, vad_state, audio_data)

        except Exception as e:
            self.get_logger().error(
                f"Error while processing audio message: {str(e)}")

    def handle_vad_state(self, stream_id: int, vad_state: int, audio_data: bytes):
        """Handle VAD state changes"""
        stream_name = self.stream_names.get(
            stream_id, f"Unknown stream {stream_id}")
        vad_name = self.vad_state_names.get(
            vad_state, f"Unknown state {vad_state}")

        self.get_logger().info(
            f"[{stream_name}] VAD state: {vad_name} audio: {len(audio_data)} bytes"
        )

        # Speech start
        if vad_state == 1:
            self.get_logger().info("🎤 Speech start detected")
            self.audio_buffers[stream_id].clear()
            self.recording_state[stream_id] = True
            if audio_data:
                self.audio_buffers[stream_id].append(audio_data)

        # Speech in progress
        elif vad_state == 2:
            self.get_logger().info("🔄 Speech in progress...")
            if self.recording_state[stream_id] and audio_data:
                self.audio_buffers[stream_id].append(audio_data)

        # Speech end
        elif vad_state == 3:
            self.get_logger().info("✅ Speech end")
            if self.recording_state[stream_id] and audio_data:
                self.audio_buffers[stream_id].append(audio_data)

            if self.recording_state[stream_id] and self.audio_buffers[stream_id]:
                self.save_audio_segment(stream_id)
            self.recording_state[stream_id] = False

        # No speech
        elif vad_state == 0:
            if self.recording_state[stream_id]:
                self.get_logger().info("⏹️ Reset recording state")
                self.recording_state[stream_id] = False

        # Print current buffer status
        buffer_size = sum(len(chunk)
                          for chunk in self.audio_buffers[stream_id])
        recording = self.recording_state[stream_id]
        self.get_logger().debug(
            f"[Stream {stream_id}] Buffer size: {buffer_size} bytes, recording: {recording}"
        )

    def save_audio_segment(self, stream_id: int):
        """Save audio segment"""
        if not self.audio_buffers[stream_id]:
            return

        # Merge all audio data
        audio_data = b''.join(self.audio_buffers[stream_id])

        # Get current timestamp
        now = datetime.now()
        timestamp = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]  # to milliseconds

        # Create subdirectory by stream_id
        stream_dir = os.path.join(self.audio_output_dir, f"stream_{stream_id}")
        os.makedirs(stream_dir, exist_ok=True)

        # Generate filename
        stream_name = "internal_mic" if stream_id == 1 else "external_mic" if stream_id == 2 else f"stream_{stream_id}"
        filename = f"{stream_name}_{timestamp}.pcm"
        filepath = os.path.join(stream_dir, filename)

        try:
            # Save PCM file
            with open(filepath, 'wb') as f:
                f.write(audio_data)

            self.get_logger().info(
                f"Audio segment saved: {filepath} (size: {len(audio_data)} bytes)")

            # Calculate audio duration (assuming 16 kHz, 16-bit, mono)
            sample_rate = 16000
            bits_per_sample = 16
            channels = 1
            bytes_per_sample = bits_per_sample // 8
            total_samples = len(audio_data) // (bytes_per_sample * channels)
            duration_seconds = total_samples / sample_rate

            self.get_logger().info(
                f"Audio duration: {duration_seconds:.2f} s ({total_samples} samples)")

        except Exception as e:
            self.get_logger().error(f"Failed to save audio file: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = AudioSubscriber()

    try:
        node.get_logger().info("Listening to noise-suppressed audio data, press Ctrl+C to exit...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt signal received, exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
