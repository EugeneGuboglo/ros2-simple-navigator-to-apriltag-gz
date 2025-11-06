#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import math

class PIDController:
    def __init__(self, kp, ki, kd, max_output, min_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.previous_error = 0.0
        self.integral = 0.0
    
    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # Ограничение выхода
        if output > self.max_output:
            output = self.max_output
        elif output < self.min_output:
            output = self.min_output
        
        self.previous_error = error
        return output

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Параметры ПИД-регуляторов
        self.linear_kp = 0.5
        self.linear_ki = 0.01
        self.linear_kd = 0.1
        self.linear_max = 0.5
        
        self.angular_kp = 1.0
        self.angular_ki = 0.05
        self.angular_kd = 0.2
        self.angular_max = 1.0
        
        # Целевые значения
        self.target_distance = 1.0
        self.distance_tolerance = 0.1
        self.y_tolerance = 0.2
        
        # Инициализация ПИД-регуляторов
        self.linear_pid = PIDController(
            self.linear_kp, self.linear_ki, self.linear_kd,
            self.linear_max, -self.linear_max
        )
        
        self.angular_pid = PIDController(
            self.angular_kp, self.angular_ki, self.angular_kd,
            self.angular_max, -self.angular_max
        )
        
        # Переменные состояния
        self.current_distance = 0.0
        self.current_y_offset = 0.0
        self.tag_detected = False
        self.last_time = self.get_clock().now()
        
        # Подписки и публикации
        self.distance_sub = self.create_subscription(
            Point, '/tag_distance', self.distance_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        
        # Таймер для управления
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Navigation Controller node started')
    
    def distance_callback(self, msg):
        self.current_distance = msg.x
        self.current_y_offset = msg.z
        self.tag_detected = True
        self.last_time = self.get_clock().now()
    
    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        cmd_msg = Twist()
        goal_reached_msg = Bool()
        
        if self.tag_detected:
            # Проверяем достижение цели
            distance_error = abs(self.current_distance - self.target_distance)
            y_error = abs(self.current_y_offset)
            
            if (distance_error <= self.distance_tolerance and 
                y_error <= self.y_tolerance):
                goal_reached_msg.data = True
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.get_logger().info('🎯 Goal reached! Stopping robot.')
            else:
                goal_reached_msg.data = False
                
                # Управление линейной скоростью (расстояние)
                linear_error = self.current_distance - self.target_distance
                linear_vel = self.linear_pid.compute(linear_error, dt)
                
                # Управление угловой скоростью (смещение по Y)
                angular_vel = self.angular_pid.compute(self.current_y_offset, dt)
                
                cmd_msg.linear.x = linear_vel
                cmd_msg.angular.z = angular_vel
                
                self.get_logger().info(f'🚗 Controlling - Linear: {linear_vel:.2f}, Angular: {angular_vel:.2f}, Distance: {self.current_distance:.2f}m')
        else:
            # Если тег не обнаружен, останавливаемся
            goal_reached_msg.data = False
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.get_logger().info('❌ No tag detected - waiting...')
        
        self.cmd_pub.publish(cmd_msg)
        self.goal_reached_pub.publish(goal_reached_msg)
        self.last_time = current_time

def main():
    rclpy.init()
    node = NavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
