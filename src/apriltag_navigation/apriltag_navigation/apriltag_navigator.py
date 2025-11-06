#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray
import math
import numpy as np
from tf_transformations import euler_from_quaternion

class AprilTagNavigator(Node):
    def __init__(self):
        super().__init__('apriltag_navigator')
        
        # Параметры
        self.declare_parameters(namespace='', parameters=[
            ('target_distance', 1.0),
            ('max_linear_speed', 0.05),
            ('max_angular_speed', 0.5),
            ('kp_linear', 0.3),
            ('ki_linear', 0.005),
            ('kd_linear', 0.1),
            ('kp_angular', 0.8),
            ('ki_angular', 0.005),
            ('kd_angular', 0.1),
            ('tag_id', 0),
            ('command_frequency', 2.0),
        ])
        
        self.target_distance = self.get_parameter('target_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.ki_linear = self.get_parameter('ki_linear').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.ki_angular = self.get_parameter('ki_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        self.tag_id = self.get_parameter('tag_id').value
        self.command_frequency = self.get_parameter('command_frequency').value
        
        # Публикаторы и подписчики
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag_detections',
            self.detection_callback,
            10
        )
        
        # Переменные состояния
        self.current_detection = None
        self.has_target = False
        self.detection_count = 0
        self.last_command_time = self.get_clock().now()
        self.goal_reached = False
        
        # PID переменные
        self.prev_distance_error = 0.0
        self.integral_distance = 0.0
        self.prev_angle_error = 0.0
        self.integral_angle = 0.0
        
        # Таймер для управления с очень низкой частотой
        command_interval = 1.0 / self.command_frequency
        self.control_timer = self.create_timer(command_interval, self.control_loop)
        
        self.get_logger().info('AprilTag Navigator запущен!')
        self.get_logger().info(f'Целевое расстояние: {self.target_distance} м')
        self.get_logger().info(f'Целевая метка ID: {self.tag_id}')
        self.get_logger().info(f'Макс. скорость: linear={self.max_linear_speed}, angular={self.max_angular_speed}')
        self.get_logger().info(f'Частота команд: {self.command_frequency} Гц')
        
    def detection_callback(self, msg):
        """Обработка обнаруженных AprilTag меток"""
        self.detection_count += 1
        
        if not msg.detections:
            self.has_target = False
            if self.detection_count % 20 == 0:
                self.get_logger().debug('Метки не обнаружены')
            return
            
        # Ищем метку с нужным ID
        for detection in msg.detections:
            if detection.id == self.tag_id:
                self.current_detection = detection
                self.has_target = True
                
                pose = detection.pose
                position = pose.pose.pose.position
                
                # Детальная отладка координат
                self.get_logger().info(
                    f'Метка {detection.id}: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}',
                    throttle_duration_sec=1.0
                )
                return
                
        self.has_target = False
        if self.detection_count % 20 == 0:
            self.get_logger().info(f'Метка {self.tag_id} не найдена')
        
    def is_goal_reached(self, distance, y_offset, angle):
        """Проверка достижения целевой позиции"""
        distance_ok = abs(distance - self.target_distance) <= 0.05
        y_offset_ok = abs(y_offset) <= 0.1
        angle_ok = abs(angle) <= math.radians(10)
        
        return distance_ok and y_offset_ok and angle_ok
        
    def pid_control(self, error, kp, ki, kd, prev_error, integral, dt=0.5):
        """ПИД-регулятор с ограничением интеграла"""
        # Ограничение интегральной составляющей
        integral += error * dt
        integral_max = 0.5 / ki if ki > 0 else 1.0
        integral = np.clip(integral, -integral_max, integral_max)
        
        derivative = (error - prev_error) / dt if dt > 0 else 0.0
        
        output = kp * error + ki * integral + kd * derivative
        
        return output, integral, error
    
    def control_loop(self):
        """Основной цикл управления роботом"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_command_time).nanoseconds / 1e9
        self.last_command_time = current_time
        
        cmd_msg = Twist()
        
        # Если цель уже достигнута - полная остановка
        if self.goal_reached:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_pub.publish(cmd_msg)
            return
        
        if not self.has_target or self.current_detection is None:
            # Поиск метки - быстрое вращение
            cmd_msg.angular.z = 0.3
            cmd_msg.linear.x = 0.0
            # Сброс интегральных составляющих при потере метки
            self.integral_distance = 0.0
            self.integral_angle = 0.0
            
            if self.detection_count % 10 == 0:
                self.get_logger().info('🔍 Поиск AprilTag метки...')
        else:
            # Управление движением к метке
            pose = self.current_detection.pose
            position = pose.pose.pose.position
            
            # В системе координат камеры:
            distance = abs(position.z)  # Расстояние до метки
            y_offset = position.x       # Боковое смещение
            
            # Вычисляем угол до метки (направление на метку)
            if distance > 0.05:
                angle_to_target = math.atan2(y_offset, distance)
            else:
                angle_to_target = 0.0
            
            # Проверяем достижение цели
            if self.is_goal_reached(distance, y_offset, angle_to_target):
                self.goal_reached = True
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.get_logger().info('🎯 Цель достигнута! Полная остановка.')
                self.cmd_pub.publish(cmd_msg)
                return
            
            # ПИД-регулятор для линейной скорости
            distance_error = distance - self.target_distance
            linear_speed, self.integral_distance, self.prev_distance_error = self.pid_control(
                distance_error, self.kp_linear, self.ki_linear, self.kd_linear,
                self.prev_distance_error, self.integral_distance, dt
            )
            
            # ПИД-регулятор для угловой скорости
            angular_speed, self.integral_angle, self.prev_angle_error = self.pid_control(
                angle_to_target, self.kp_angular, self.ki_angular, self.kd_angular,
                self.prev_angle_error, self.integral_angle, dt
            )
            
            # ПРОСТОЕ ОГРАНИЧЕНИЕ СКОРОСТИ (без плавного торможения)
            linear_speed = np.clip(linear_speed, -self.max_linear_speed, self.max_linear_speed)
            angular_speed = np.clip(angular_speed, -self.max_angular_speed, self.max_angular_speed)
            
            cmd_msg.linear.x = linear_speed
            cmd_msg.angular.z = angular_speed
            
            # Логирование для отладки
            self.get_logger().info(
                f'К метке: dist={distance:.2f}m (цель: {self.target_distance}m), '
                f'ошибка={distance_error:.3f}m, '
                f'y_offset={y_offset:.3f}m, '
                f'angle={math.degrees(angle_to_target):.1f}°, '
                f'cmd=({linear_speed:.3f}, {angular_speed:.3f})'
            )
        
        # Публикуем команду
        self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = AprilTagNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        # Остановка робота при завершении
        stop_msg = Twist()
        navigator.cmd_pub.publish(stop_msg)
        navigator.get_logger().info('Навигация остановлена')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
