#include <aimdk_msgs/msg/audio_vad_state_type.hpp>
#include <aimdk_msgs/msg/processed_audio_output.hpp>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

class AudioSubscriber : public rclcpp::Node {
public:
  AudioSubscriber() : rclcpp::Node("audio_subscriber") {
    // Audio buffers, stored separately by stream_id
    // stream_id -> buffer
    audio_buffers_ = {};
    recording_state_ = {};

    audio_output_dir_ = "audio_recordings";
    fs::create_directories(audio_output_dir_);

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.keep_last(10).best_effort();

    subscription_ =
        this->create_subscription<aimdk_msgs::msg::ProcessedAudioOutput>(
            "/agent/process_audio_output", qos,
            std::bind(&AudioSubscriber::audio_callback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Starting to subscribe to denoised audio data...");
  }

private:
  void
  audio_callback(const aimdk_msgs::msg::ProcessedAudioOutput::SharedPtr msg) {
    try {
      uint32_t stream_id = msg->stream_id;
      uint8_t vad_state = msg->audio_vad_state.value;
      const std::vector<uint8_t> &audio_data = msg->audio_data;

      static const std::unordered_map<uint8_t, std::string> vad_state_names = {
          {0, "No Speech"},
          {1, "Speech Start"},
          {2, "Speech Processing"},
          {3, "Speech End"}};
      static const std::unordered_map<uint32_t, std::string> stream_names = {
          {1, "Internal Microphone"}, {2, "External Microphone"}};

      RCLCPP_INFO(this->get_logger(),
                  "Audio data received: stream_id=%u, vad_state=%u(%s), "
                  "audio_size=%zu bytes",
                  stream_id, vad_state,
                  vad_state_names.count(vad_state)
                      ? vad_state_names.at(vad_state).c_str()
                      : "Unknown State",
                  audio_data.size());

      handle_vad_state(stream_id, vad_state, audio_data);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing audio message: %s",
                   e.what());
    }
  }

  void handle_vad_state(uint32_t stream_id, uint8_t vad_state,
                        const std::vector<uint8_t> &audio_data) {
    // Initialize the buffer for this stream_id (if it does not exist)
    if (audio_buffers_.count(stream_id) == 0) {
      audio_buffers_[stream_id] = std::vector<uint8_t>();
      recording_state_[stream_id] = false;
    }

    static const std::unordered_map<uint8_t, std::string> vad_state_names = {
        {0, "No Speech"},
        {1, "Speech Start"},
        {2, "Speech Processing"},
        {3, "Speech End"}};
    static const std::unordered_map<uint32_t, std::string> stream_names = {
        {1, "Internal Microphone"}, {2, "External Microphone"}};

    RCLCPP_INFO(this->get_logger(), "[%s] VAD Atate: %s Audio Data: %zu bytes",
                stream_names.count(stream_id)
                    ? stream_names.at(stream_id).c_str()
                    : ("Unknown Stream " + std::to_string(stream_id)).c_str(),
                vad_state_names.count(vad_state)
                    ? vad_state_names.at(vad_state).c_str()
                    : ("Unknown State" + std::to_string(vad_state)).c_str(),
                audio_data.size());

    // AUDIO_VAD_STATE_BEGIN
    if (vad_state == 1) {
      RCLCPP_INFO(this->get_logger(), "🎤 Speech detected - Start");
      audio_buffers_[stream_id].clear();
      recording_state_[stream_id] = true;
      if (!audio_data.empty()) {
        audio_buffers_[stream_id].insert(audio_buffers_[stream_id].end(),
                                         audio_data.begin(), audio_data.end());
      }

      // AUDIO_VAD_STATE_PROCESSING
    } else if (vad_state == 2) {
      RCLCPP_INFO(this->get_logger(), "🔄 Speech Processing...");
      if (recording_state_[stream_id] && !audio_data.empty()) {
        audio_buffers_[stream_id].insert(audio_buffers_[stream_id].end(),
                                         audio_data.begin(), audio_data.end());
      }

      // AUDIO_VAD_STATE_END
    } else if (vad_state == 3) {
      RCLCPP_INFO(this->get_logger(), "✅ Speech End");
      if (recording_state_[stream_id] && !audio_data.empty()) {
        audio_buffers_[stream_id].insert(audio_buffers_[stream_id].end(),
                                         audio_data.begin(), audio_data.end());
      }
      if (recording_state_[stream_id] && !audio_buffers_[stream_id].empty()) {
        save_audio_segment(audio_buffers_[stream_id], stream_id);
      }
      recording_state_[stream_id] = false;

      // AUDIO_VAD_STATE_NONE
    } else if (vad_state == 0) {
      if (recording_state_[stream_id]) {
        RCLCPP_INFO(this->get_logger(), "⏹️ Recording state reset");
        recording_state_[stream_id] = false;
      }
    }

    // Output the current buffer status.
    size_t buffer_size = audio_buffers_[stream_id].size();
    bool recording = recording_state_[stream_id];
    RCLCPP_DEBUG(this->get_logger(),
                 "[Stream %u] Buffer size: %zu bytes, Recording state: %s",
                 stream_id, buffer_size, recording ? "true" : "false");
  }

  void save_audio_segment(const std::vector<uint8_t> &audio_data,
                          uint32_t stream_id) {
    if (audio_data.empty())
      return;

    // Get the current timestamp.
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;

    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << "_"
        << std::setw(3) << std::setfill('0') << ms.count();

    // Create a subdirectory by stream_id.
    fs::path stream_dir =
        fs::path(audio_output_dir_) / ("stream_" + std::to_string(stream_id));
    fs::create_directories(stream_dir);

    static const std::unordered_map<uint32_t, std::string> stream_names = {
        {1, "internal_mic"}, {2, "external_mic"}};
    std::string stream_name = stream_names.count(stream_id)
                                  ? stream_names.at(stream_id)
                                  : ("stream_" + std::to_string(stream_id));
    std::string filename = stream_name + "_" + oss.str() + ".pcm";
    fs::path filepath = stream_dir / filename;

    try {
      std::ofstream ofs(filepath, std::ios::binary);
      ofs.write(reinterpret_cast<const char *>(audio_data.data()),
                audio_data.size());
      ofs.close();
      RCLCPP_INFO(this->get_logger(),
                  "Audio segment saved: %s (size: %zu bytes)", filepath.c_str(),
                  audio_data.size());

      // Record audio file duration (assuming 16kHz, 16-bit, mono)
      int sample_rate = 16000;
      int bits_per_sample = 16;
      int channels = 1;
      int bytes_per_sample = bits_per_sample / 8;
      size_t total_samples = audio_data.size() / (bytes_per_sample * channels);
      double duration_seconds =
          static_cast<double>(total_samples) / sample_rate;

      RCLCPP_INFO(this->get_logger(),
                  "Audio duration: %.2f seconds (%zu samples)",
                  duration_seconds, total_samples);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save audio file: %s",
                   e.what());
    }
  }

  // Member variables
  std::unordered_map<uint32_t, std::vector<uint8_t>> audio_buffers_;
  std::unordered_map<uint32_t, bool> recording_state_;
  std::string audio_output_dir_;
  rclcpp::Subscription<aimdk_msgs::msg::ProcessedAudioOutput>::SharedPtr
      subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AudioSubscriber>();
  RCLCPP_INFO(node->get_logger(),
              "Listening for denoised audio data, press Ctrl+C to exit...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}