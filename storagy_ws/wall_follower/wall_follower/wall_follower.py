#!/usr/bin/env python3
# wall_follower_node.py

import math
import random
import statistics
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from wall_follower_msgs.msg import Stop


class WallFollower(Node):
    """
    상태:
        0  방황  (장애물 탐색)
        1  벽 쪽으로 회전
        2  벽 추종
    별도: STOP 신호 수신 시 3초 정지 (타이머 기반)
    """

    def __init__(self):
        super().__init__('wall_follower')

        # ───────── 파라미터 ─────────────────────────────────────────
        self.declare_parameter('wall_distance', 2.0)
        self.declare_parameter('wall_lead', 2.5)
        self.declare_parameter('side', 1)          # 오른쪽 벽: +1, 왼쪽 벽: -1
        self.declare_parameter('speed', 0.1)

        self.wall_distance = self.get_parameter('wall_distance').value
        self.wall_lead     = self.get_parameter('wall_lead').value
        self.side          = self.get_parameter('side').value
        self.default_speed = self.get_parameter('speed').value

        # ───────── 내부 변수 ───────────────────────────────────────
        self.state           = 0
        self.turn_start_time = self.get_clock().now().nanoseconds / 1e9
        self.wall_direction  = None
        self.alpha           = 0.0

        self.stop_until = None
        self.stop_sign_received_time = None
        self.stopped = False

        # ───────── ROS 인터페이스 ─────────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Stop, '/sign', self.sign_callback, 10)
        self.create_timer(0.05, self.timer_callback)  # 20 Hz

    # =============================================================
    # STOP 토픽 콜백
    # =============================================================
    def sign_callback(self, msg: Stop):
        if msg.sign:
            now = self.get_clock().now()
            self.stop_sign_received_time = now
            self.stop_until = now + Duration(seconds=3)
            self.stopped = False

    # =============================================================
    # LiDAR 콜백 (주행 로직)
    # =============================================================
    def scan_callback(self, msg: LaserScan):
        scan_max = msg.range_max

        regions = {
            'N'  : self._region_median(msg, 0),
            'NNE': self._region_median(msg, -22.5),
            'NE' : self._region_median(msg, -45),
            'E'  : self._region_median(msg, -80),
            'NNW': self._region_median(msg, 22.5),
            'NW' : self._region_median(msg, 45),
            'W'  : self._region_median(msg, 80),
        }

        # ───── STATE 0: 방황 ─────
        if self.state == 0:
            if any(regions[r] < scan_max for r in ['N', 'NE', 'NW', 'NNE', 'NNW']):
                self.state = 1
                self.wall_direction = min(
                    ['N', 'NE', 'NW', 'NNE', 'NNW'],
                    key=lambda k: regions[k]
                )
                self.turn_start_time = self._now_sec()
            else:
                delta = self._now_sec() - self.turn_start_time
                if delta > 6:
                    rand = random.randint(0, 4)
                    self.alpha = math.pi/2 - rand * math.pi/4
                    self.turn_start_time = self._now_sec()
                elif delta > 1:
                    self.alpha = 0.0

        # ───── STATE 1: 벽 회전 ─────
        elif self.state == 1:
            if min(regions['N'], regions['NW'], regions['NE'],
                   regions['NNE'], regions['NNW']) < self.wall_distance + 0.3:
                self.state = 2
                if self.side == 0:
                    left  = (regions['W'] + regions['NW'] + regions['NNW']) / 3
                    right = (regions['E'] + regions['NE'] + regions['NNE']) / 3
                    self.side = -1 if right < left else 1
            else:
                dir_map = {
                    'W' :  math.pi/2,
                    'NW':  math.pi/4,
                    'N' :  0.0,
                    'NE': -math.pi/4,
                    'E' : -math.pi/2
                }
                delta = self._now_sec() - self.turn_start_time
                self.alpha = dir_map.get(self.wall_direction, 0.0) if delta <= 1 else 0.0

        # ───── STATE 2: 벽 추종 ─────
        elif self.state == 2:
            y0  = regions['E']/5 if self.side == -1 else regions['W']/5
            ref = regions['NE']  if self.side == -1 else regions['NW']
            x1  = ref * math.sin(math.radians(23))
            y1  = ref * math.cos(math.radians(23))

            if y0 >= self.wall_distance*2 and regions['N'] < scan_max:
                self.alpha = -math.pi/4 * self.side
            else:
                front     = regions['N']
                turn_fix  = 0 if front >= 0.5 else 1 - front
                abs_alpha = math.atan2(y1 - self.wall_distance,
                                       x1 + self.wall_lead - y0) - turn_fix*1.5
                self.alpha = self.side * abs_alpha
        self.get_logger().info(f"STATE: {self.state}")
    # =============================================================
    # 제어 루프
    # =============================================================
    def timer_callback(self):
        now = self.get_clock().now()
        stop = self.stop_until and now < self.stop_until

        if self.stop_until and now >= self.stop_until:
            self.stop_until = None
            self.stop_sign_received_time = None
            self.stopped = False

        v = 0.0 if stop else self.default_speed
        w = 0.0 if stop else self.alpha

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

        # STOP 감지 시 정지까지 걸린 시간 1회 출력
        if stop and not self.stopped and self.stop_sign_received_time:
            dt = (now - self.stop_sign_received_time).nanoseconds / 1e9
            self.get_logger().info(f'[STOP 측정] STOP 명령 수신 후 정지까지 {dt:.3f}초 소요')
            self.stopped = True

    # =============================================================
    # 유틸
    # =============================================================
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _region_median(self, msg: LaserScan, center_deg: float,
                       width_deg: float = 10) -> float:
        angle_min  = math.degrees(msg.angle_min)
        angle_inc  = math.degrees(msg.angle_increment)
        num_ranges = len(msg.ranges)

        center_idx = int((center_deg - angle_min) / angle_inc)
        half_span  = int(width_deg / angle_inc)
        start      = max(0, center_idx - half_span)
        end        = min(num_ranges, center_idx + half_span)

        vals = [msg.ranges[i] for i in range(start, end) if math.isfinite(msg.ranges[i])]
        return statistics.median(vals) if vals else msg.range_max


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
