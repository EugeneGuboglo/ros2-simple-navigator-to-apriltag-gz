import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Получаем путь к пакету
    pkg_share = get_package_share_directory('apriltag_navigation')
    
    # Пути к файлам
    world_path = os.path.join(pkg_share, 'worlds', 'apriltag_world.sdf')
    model_path = os.path.join(pkg_share, 'models', 'simple_robot', 'model.sdf')
    
    return LaunchDescription([
        
        # Запуск Gazebo с миром AprilTag
        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                '-r',
                world_path
            ],
            output='screen'
        ),
        
        # Spawn робота (с задержкой, чтобы Gazebo успел запуститься)
        ExecuteProcess(
            cmd=[
                'bash', '-c', 
                f'gz service -s /world/apriltag_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req "sdf_filename: \\"{model_path}\\""'
            ],
            output='screen'
        ),
        
        # Bridge для камеры (первая команда)
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 run ros_gz_bridge parameter_bridge /world/apriltag_world/model/my_robot/link/camera_link/sensor/camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            output='screen'
        ),
        
        # Bridge для информации камеры (вторая команда)
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 run ros_gz_bridge parameter_bridge /world/apriltag_world/model/my_robot/link/camera_link/sensor/camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            output='screen'
        ),
        
         # Bridge для cmd_vel
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        ),
        # Запуск AprilTag детектора (с задержкой)
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                ' ros2 launch apriltag_ros tag_detector.launch.py'
            ],
            output='screen'
        ),
        
         # Запуск apriltag_navigator
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 run apriltag_navigation apriltag_navigator'
            ],
            output='screen'
        ),
        
    ])
