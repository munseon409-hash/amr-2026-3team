#!/usr/bin/env python3
"""
산불 감지&구조 로봇 - ROS2 기반 행동 트리
"""

import py_trees
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Bool


class FireDetectionRobotBT(Node):
    def __init__(self):
        super().__init__('fire_detection_bt')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.search_waypoints = [(1.0, 1.0), (5.0, 1.0), (5.0, 5.0), (1.0, 5.0)]
        self.waypoint_index = 0
        
        self.red_subscription = self.create_subscription(
            Bool, '/red_detected', self.red_callback, 10)
        self.point_subscription = self.create_subscription(
            Point, '/red_target_point', self.point_callback, 10)
        
        self.fire_detected = False
        self.target_point = None
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_moving = False
        self.is_searching = True
        self.tts_pub = self.create_publisher(String, '/tts_output', 10)
        self.fire_location_pub = self.create_publisher(String, '/fire_location_report', 10)
        
        self.get_logger().info("🚀 BT 초기화 완료!")
    
    def red_callback(self, msg):
        self.fire_detected = msg.data
        if self.fire_detected:
            self.get_logger().warn("🔥 빨간색 감지!")
    
    def point_callback(self, msg):
        self.target_point = (msg.x, msg.y)
        self.get_logger().info(f"📍 위치: ({msg.x:.1f}, {msg.y:.1f})")
    
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("⏹️ 로봇 정지!")
    
    def output_voice(self, message):
        msg = String()
        msg.data = message
        self.tts_pub.publish(msg)
        self.get_logger().info(f"📢 음성: {message}")
    
    def report_location(self):
        msg = String()
        if self.target_point:
            msg.data = f"빨간색 감지! 위치: ({self.target_point[0]:.1f}, {self.target_point[1]:.1f})"
        else:
            msg.data = "빨간색 감지!"
        self.fire_location_pub.publish(msg)


class CheckRedDetectionNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, bt):
        super().__init__(name)
        self.bt = bt
    
    def update(self):
        return py_trees.common.Status.SUCCESS if self.bt.fire_detected else py_trees.common.Status.FAILURE


class StopRobotNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, bt):
        super().__init__(name)
        self.bt = bt
    
    def update(self):
        self.bt.stop_robot()
        return py_trees.common.Status.SUCCESS


class FireAlertNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, bt):
        super().__init__(name)
        self.bt = bt
    
    def update(self):
        self.bt.output_voice("빨간색 감지! 긴급 신호 송출!")
        self.bt.report_location()
        return py_trees.common.Status.SUCCESS


class ContinueSearchNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, bt):
        super().__init__(name)
        self.bt = bt
    
    def update(self):
        self.bt.get_logger().info("🔄 계속 탐색 중...")
        return py_trees.common.Status.RUNNING


def create_behavior_tree(bt):
    root = py_trees.composites.Sequence("산불감지미션", memory=False)
    mission_selector = py_trees.composites.Selector("감지확인", memory=False)
    
    fire_response = py_trees.composites.Sequence("감지시처리", memory=False)
    fire_response.add_children([
        CheckRedDetectionNode("신호확인", bt),
        StopRobotNode("정지", bt),
        FireAlertNode("알람", bt),
    ])
    
    search = ContinueSearchNode("탐색", bt)
    mission_selector.add_children([fire_response, search])
    root.add_child(mission_selector)
    
    return root


def main(args=None):
    rclpy.init(args=args)
    bt = FireDetectionRobotBT()
    root = create_behavior_tree(bt)
    
    def bt_tick():
        root.tick_once()
    
    timer = bt.create_timer(0.1, bt_tick)
    
    bt.get_logger().info("=" * 50)
    bt.get_logger().info("[ BT 구조 ]")
    bt.get_logger().info("Root (산불감지미션)")
    bt.get_logger().info("=" * 50)
    
    try:
        rclpy.spin(bt)
    except KeyboardInterrupt:
        bt.stop_robot()
        bt.get_logger().info("🛑 종료")
    finally:
        bt.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
