from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pcd_pub = Node(
        package='py_examples',
        executable='pcd_to_cloud',
        name='pcd_to_cloud',
        output='screen',
        parameters=[{
            'pcd_path': '/home/ubuntu/agibot_ws/src/spark-fast-lio/spark_fast_lio/PCD/map_merged.pcd',
            'topic_name': '/map',
            'frame_id': 'map',
            'publish_rate': 1.0,
        }]
    )


    return LaunchDescription([
        pcd_pub,
    ])
