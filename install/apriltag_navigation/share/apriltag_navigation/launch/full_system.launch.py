from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'apriltag_navigation'
    
    # Запуск Gazebo с миром
    world_path = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'apriltag_world.world'
    )
    
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_path],
        output='screen'
    )
    
    # Запуск детектора AprilTag (ваш существующий launch файл)
    detector_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'apriltag_ros', 'tag_detector.launch.py'],
        output='screen'
    )
    
    # Запуск навигатора
    navigator = Node(
        package=package_name,
        executable='apriltag_navigator',
        name='apriltag_navigator',
        output='screen',
        parameters=[{
            'target_distance': 1.0,
            'max_linear_speed': 0.3,
            'max_angular_speed': 0.8,
            'kp_linear': 0.5,
            'kp_angular': 1.5,
            'tag_id': 0
        }]
    )
    
    return LaunchDescription([
        gazebo,
        detector_launch,
        navigator,
    ])
