#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time
import os

class CupSpawner(Node):
    def __init__(self):
        super().__init__('cup_spawner')
        self.timer_period = 2.0  # 초 간격으로 컵 드롭 (2초로 영상 찍긴 했어)
        self.count = 0
        self.max_cups = 10

        # cup.urdf 절대경로
        self.cup_urdf_path = '/home/yobellee/dev_ws/src/IFRA_ConveyorBelt/conveyorbelt_gazebo/urdf/cup.urdf'
        if not os.path.exists(self.cup_urdf_path):
            self.get_logger().error(f'컵 URDF 파일이 존재하지 않음: {self.cup_urdf_path}')
            return

        self.timer = self.create_timer(self.timer_period, self.spawn_next_cup)

    def spawn_next_cup(self):
        if self.count >= self.max_cups:
            self.get_logger().info('✅ 모든 컵을 드롭했습니다.')
            self.timer.cancel()
            return

        model_name = f"cup_{self.count}"
        x, y, z = 0.0, -2.3, 0.9 + self.count * 0.0  # loader 위치 y축(2번째 term) 조정
        

        cmd = [
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', model_name,
            '-file', self.cup_urdf_path,
            '-robot_namespace', '/',
            '-x', str(x), '-y', str(y), '-z', str(z)
        ]

        self.get_logger().info(f'📦 컵 드롭 중: {model_name}')
        subprocess.Popen(cmd)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CupSpawner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('🛑 사용자 중단')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()