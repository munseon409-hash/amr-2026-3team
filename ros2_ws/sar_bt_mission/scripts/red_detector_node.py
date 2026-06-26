#!/usr/bin/env python3
"""
red_detector_node.py

카메라 토픽(/color/image_raw)을 구독해서 빨간색 영역을 검출하고,
- /red_detected        (std_msgs/Bool)        : 빨간색 발견 여부
- /red_target_point     (geometry_msgs/Point)  : 화면 좌표 기준 빨간 영역 중심 (x: -1.0~1.0, y: -1.0~1.0, z: area비율)
- /red_detection/image  (sensor_msgs/Image)    : 디버그용, 박스가 그려진 영상

를 발행하는 ROS2 노드.

나중에 Nav2 우선순위 로직(예: behavior tree, 별도 commander 노드)에서
/red_detected 와 /red_target_point 를 구독해서
"빨간색이 보이면 그 방향으로 우선 이동" 로직을 짜면 됩니다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

from cv_bridge import CvBridge
import cv2
import numpy as np


class RedDetectorNode(Node):
    def __init__(self):
        super().__init__('red_detector_node')

        # ---- 파라미터 (런타임에 조정 가능) ----
        self.declare_parameter('image_topic', '/color/image_raw')
        self.declare_parameter('min_area_ratio', 0.002)  # 전체 화면 대비 최소 비율 (노이즈 제거용)
        self.declare_parameter('publish_debug_image', True)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.min_area_ratio = self.get_parameter('min_area_ratio').get_parameter_value().double_value
        self.publish_debug = self.get_parameter('publish_debug_image').get_parameter_value().bool_value

        # ---- HSV 빨간색 범위 ----
        # 빨강은 Hue가 0 근처와 180 근처 양쪽에 걸쳐 있어서 두 범위를 OR로 합쳐야 함
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        self.bridge = CvBridge()

        # 카메라 토픽은 best-effort sensor QoS로 오는 경우가 많아 맞춰줌
        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        self.detected_pub = self.create_publisher(Bool, '/red_detected', 10)
        self.point_pub = self.create_publisher(Point, '/red_target_point', 10)

        if self.publish_debug:
            self.debug_pub = self.create_publisher(Image, '/red_detection/image', 10)

        self.get_logger().info(f'Red detector started. Subscribing to: {image_topic}')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        height, width = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 노이즈 제거 (작은 점들 없애기)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_msg = Bool()
        point_msg = Point()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            area_ratio = area / float(width * height)

            if area_ratio >= self.min_area_ratio:
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w / 2.0
                cy = y + h / 2.0

                # 화면 좌표를 -1.0 ~ 1.0 범위로 정규화
                # x: 음수면 화면 왼쪽, 양수면 화면 오른쪽 (로봇 회전 방향 결정에 바로 쓸 수 있음)
                # y: 음수면 화면 위쪽, 양수면 화면 아래쪽
                norm_x = (cx - width / 2.0) / (width / 2.0)
                norm_y = (cy - height / 2.0) / (height / 2.0)

                detected_msg.data = True
                point_msg.x = float(norm_x)
                point_msg.y = float(norm_y)
                point_msg.z = float(area_ratio)  # 화면에서 차지하는 비율 -> 대략적인 거리 힌트로 활용 가능

                if self.publish_debug:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (int(cx), int(cy)), 5, (0, 255, 0), -1)
                    cv2.putText(
                        frame, f'RED area={area_ratio:.3f}',
                        (x, max(0, y - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2
                    )
            else:
                detected_msg.data = False
        else:
            detected_msg.data = False

        self.detected_pub.publish(detected_msg)
        self.point_pub.publish(point_msg)

        if self.publish_debug:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RedDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
