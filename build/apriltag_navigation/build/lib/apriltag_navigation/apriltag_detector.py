#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseArray, Pose
from std_msgs.msg import Header
import math

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        
        # Публикация позиции тега
        self.pose_pub = self.create_publisher(PoseArray, '/tag_pose', 10)
        self.distance_pub = self.create_publisher(Point, '/tag_distance', 10)
        
        # Таймер для эмуляции обнаружения тега
        self.timer = self.create_timer(0.1, self.detect_tag)
        
        self.get_logger().info('AprilTag Detector node started')
    
    def detect_tag(self):
        # Эмуляция обнаружения тега - постепенно уменьшаем расстояние
        # Начальное расстояние 3.0 метра, уменьшаем на 0.05 каждый вызов
        if not hasattr(self, 'current_distance'):
            self.current_distance = 3.0
        
        # Уменьшаем расстояние для эмуляции движения к тегу
        self.current_distance = max(0.5, self.current_distance - 0.02)
        
        # Добавляем небольшое случайное смещение по Y
        import random
        y_offset = random.uniform(-0.1, 0.1)
        
        position = Point()
        position.x = self.current_distance  # расстояние по X
        position.y = y_offset  # смещение по Y
        position.z = 0.0  # высота
        
        # Создаем PoseArray для визуализации
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_link"
        
        tag_pose = Pose()
        tag_pose.position = position
        pose_array.poses.append(tag_pose)
        
        # Публикуем позицию
        self.pose_pub.publish(pose_array)
        
        # Публикуем расстояние
        distance_msg = Point()
        distance = math.sqrt(position.x**2 + position.y**2 + position.z**2)
        distance_msg.x = distance  # общее расстояние
        distance_msg.y = position.x  # расстояние по X
        distance_msg.z = position.y  # смещение по Y
        
        self.distance_pub.publish(distance_msg)
        
        self.get_logger().info(f'Tag detected - Distance: {distance:.2f}m, X: {position.x:.2f}, Y: {position.y:.2f}')

def main():
    rclpy.init()
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
