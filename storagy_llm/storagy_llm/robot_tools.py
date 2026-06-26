import math
import time
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from langchain_core.tools import StructuredTool
from typing import List

def create_tools(tool_set) -> List[StructuredTool]:
    tool_specs = [
        {
            "name": "get_current_location",
            "description": "사용자가 현재 로봇의 위치나 근처 장소를 물어봤을 때 사용합니다. 각 장소와 거리를 계산해 가장 가까운 위치를 반환합니다.",
            "func": lambda: tool_set.get_current_location()
        },
        {
            "name": "list_locations",
            "description": "사용자가 어디로 이동할 수 있는지 물어봤을 때 사용합니다. 이동 가능한 장소 목록을 보여줍니다.",
            "func": lambda: ", ".join(tool_set.list_locations())
        },
        {
            "name": "move_to_location",
            "description": "사용자가 특정 장소로 이동하라고 말했을 때 사용합니다.",
            "func": lambda place: tool_set.move_to_location(place)
        },
        {
            "name": "explain_front_camera",
            "description": "사용자가 앞에 무엇이 보이는지, 혹은 카메라 이미지를 분석하여 현재 로봇 전방의 상황을 설명해 달라고 했을 때 사용합니다.",
            "func": lambda: tool_set.explain_front_camera()
        },
        {
            "name": "cancel_navigation",
            "description": "사용자가 이동을 취소하거나, 로봇을 멈추라고(정지) 명령했을 때 사용합니다. 랜덤 이동과 네비게이션을 모두 중단하고 로봇을 정지시킵니다.",
            "func": lambda: tool_set.cancel_navigation()
        },
        {
            "name": "start_random_wander",
            "description": "사용자가 랜덤 이동(자유 주행, 배회, 돌아다니기)을 시작하라고 했을 때 사용합니다.",
            "func": lambda: tool_set.set_wander(True)
        },
        {
            "name": "stop_random_wander",
            "description": "사용자가 랜덤 이동(배회)을 멈추라고 했을 때 사용합니다. 진행 중인 랜덤 이동 목표도 즉시 취소됩니다.",
            "func": lambda: tool_set.set_wander(False)
        },
        {
            "name": "move_robot",
            "description": "지정한 거리만큼 로봇을 직진 이동시킵니다 (텔레옵). direction은 'forward'(앞) 또는 'backward'(뒤), distance_m은 미터 단위 거리. 예: '앞으로 1미터 가줘' → direction='forward', distance_m=1.0. 실행 전 랜덤 이동/네비게이션은 자동으로 정지됩니다.",
            "func": lambda direction, distance_m: tool_set.teleop_move(direction, distance_m)
        },
        {
            "name": "rotate_robot",
            "description": "로봇을 제자리에서 회전시킵니다 (텔레옵). direction은 'left' 또는 'right', angle_deg는 회전 각도(도). 예: '오른쪽으로 돌아' → right, 90 / '뒤로 돌아(반바퀴)' → left, 180. 실행 전 랜덤 이동/네비게이션은 자동으로 정지됩니다.",
            "func": lambda direction, angle_deg: tool_set.teleop_rotate(direction, angle_deg)
        },
        {
            "name": "set_speed",
            "description": "텔레옵 속도를 설정하거나 조회합니다. linear_mps=직진 속도(m/s, 0.05~0.7), angular_rps=회전 속도(rad/s, 0.2~1.8). 바꾸지 않을 값은 0을 전달하세요 (둘 다 0이면 현재 속도 조회). '더 빠르게/느리게' 요청 시 먼저 조회한 뒤 적절히 가감해 다시 호출하세요.",
            "func": lambda linear_mps, angular_rps: tool_set.set_speed(linear_mps, angular_rps)
        },
    ]

    tools = [
        StructuredTool.from_function(
            func=spec["func"],
            name=spec["name"],
            description=spec["description"]
        )
        for spec in tool_specs
    ]

    return tools

class ToolSet(Node):
    def __init__(self, places: dict, explain_fn=None):
        super().__init__('tool_set')
        self.places = places
        self.explain_fn = explain_fn
        self.frame_id = "map"
        self.base_frame = "base_link"
        self.tolerance_m = 0.3

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.llm_active_pub = self.create_publisher(Bool, '/llm_active', 10)

        # Teleop / wander control state
        self.linear_speed = 0.2    # m/s
        self.angular_speed = 0.8   # rad/s
        self.wander_enabled = False
        wander_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )
        self.wander_pub = self.create_publisher(Bool, '/wander_enabled', wander_qos)
        self.event_pub = self.create_publisher(String, '/robot_events', 10)

        self.goal_handle = None

    def publish_llm_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.llm_active_pub.publish(msg)

    def publish_event(self, text: str):
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)

    def set_wander(self, enabled: bool) -> str:
        self.wander_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.wander_pub.publish(msg)
        if enabled:
            self.publish_event("랜덤 이동 시작")
            return "[WANDER] 랜덤 이동을 시작했습니다."
        self.publish_event("랜덤 이동 정지")
        return "[WANDER] 랜덤 이동을 정지했습니다."

    def _stop_all_motion(self):
        """Stop wandering and any active navigation so teleop has exclusive cmd_vel."""
        if self.wander_enabled:
            self.set_wander(False)
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"[TELEOP] Failed to cancel LLM goal: {e}")
            self.goal_handle = None
            self.publish_llm_active(False)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)  # give Nav2 a moment to release cmd_vel

    @staticmethod
    def _normalize_angle(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _teleop_pose(self):
        """(x, y, yaw) for teleop feedback. Prefers odom (no AMCL jumps), falls back to map."""
        for frame in ('odom', self.frame_id):
            try:
                tf = self.tf_buffer.lookup_transform(frame, self.base_frame, Time(),
                                                     timeout=Duration(seconds=0.2))
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return (tf.transform.translation.x, tf.transform.translation.y, yaw)
            except Exception:
                continue
        return None

    def teleop_move(self, direction, distance_m) -> str:
        try:
            distance = abs(float(distance_m))
        except (TypeError, ValueError):
            return "[ERR] 이동 거리 값을 이해하지 못했습니다."
        if distance <= 0.0 or distance > 5.0:
            return "[ERR] 이동 거리는 0보다 크고 5m 이하만 가능합니다."

        backward = str(direction).lower().startswith('b')
        self._stop_all_motion()

        start = self._teleop_pose()
        if start is None:
            return "[ERR] 로봇 위치(TF)를 확인할 수 없어 이동할 수 없습니다."
        sx, sy = start[0], start[1]
        sign = -1.0 if backward else 1.0
        speed = self.linear_speed

        deadline = time.time() + distance / speed * 3.0 + 5.0
        traveled = 0.0
        tw = Twist()
        while time.time() < deadline:
            cur = self._teleop_pose()
            if cur is not None:
                traveled = math.hypot(cur[0] - sx, cur[1] - sy)
                if traveled >= distance:
                    break
            # Slow down near the target for a clean stop
            remaining = distance - traveled
            v = speed if remaining > 0.15 else max(0.05, speed * 0.4)
            tw.linear.x = sign * v
            self.cmd_pub.publish(tw)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)  # let the velocity smoother finish decelerating
        cur = self._teleop_pose()
        if cur is not None:
            traveled = math.hypot(cur[0] - sx, cur[1] - sy)

        label = "뒤로" if backward else "앞으로"
        self.publish_event(f"텔레옵: {label} {traveled:.2f}m 이동 (목표 {distance:.2f}m)")
        if traveled < distance * 0.9:
            return (f"[TELEOP] {label} 이동이 {traveled:.2f}m에서 멈췄습니다 "
                    f"(목표 {distance:.2f}m). 장애물이나 시간 초과일 수 있습니다.")
        return f"[TELEOP] {label} {traveled:.2f}m 이동을 완료했습니다."

    def teleop_rotate(self, direction, angle_deg) -> str:
        try:
            angle = abs(float(angle_deg))
        except (TypeError, ValueError):
            angle = 90.0
        if angle <= 0.0 or angle > 360.0:
            return "[ERR] 회전 각도는 0보다 크고 360도 이하만 가능합니다."

        right = str(direction).lower().startswith('r')
        self._stop_all_motion()

        start = self._teleop_pose()
        if start is None:
            return "[ERR] 로봇 위치(TF)를 확인할 수 없어 회전할 수 없습니다."
        target = math.radians(angle)
        sign = -1.0 if right else 1.0
        speed = self.angular_speed

        deadline = time.time() + target / speed * 3.0 + 5.0
        prev_yaw = start[2]
        turned = 0.0
        tw = Twist()
        while time.time() < deadline:
            cur = self._teleop_pose()
            if cur is not None:
                turned += abs(self._normalize_angle(cur[2] - prev_yaw))
                prev_yaw = cur[2]
                if turned >= target:
                    break
            # Slow down near the target for a clean stop
            remaining = target - turned
            w = speed if remaining > math.radians(12.0) else max(0.2, speed * 0.4)
            tw.angular.z = sign * w
            self.cmd_pub.publish(tw)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)  # let the velocity smoother finish decelerating
        cur = self._teleop_pose()
        if cur is not None:
            turned += abs(self._normalize_angle(cur[2] - prev_yaw))

        turned_deg = math.degrees(turned)
        label = "오른쪽" if right else "왼쪽"
        self.publish_event(f"텔레옵: {label}으로 {turned_deg:.0f}° 회전 (목표 {angle:.0f}°)")
        if turned_deg < angle * 0.9:
            return (f"[TELEOP] {label} 회전이 {turned_deg:.0f}도에서 멈췄습니다 "
                    f"(목표 {angle:.0f}도). 시간 초과일 수 있습니다.")
        return f"[TELEOP] {label}으로 {turned_deg:.0f}도 회전을 완료했습니다."

    def set_speed(self, linear_mps=0, angular_rps=0) -> str:
        try:
            linear = float(linear_mps)
            angular = float(angular_rps)
        except (TypeError, ValueError):
            return "[ERR] 속도 값을 이해하지 못했습니다."

        changed = False
        if linear > 0:
            self.linear_speed = min(max(linear, 0.05), 0.7)
            changed = True
        if angular > 0:
            self.angular_speed = min(max(angular, 0.2), 1.8)
            changed = True

        status = (f"직진 {self.linear_speed:.2f}m/s, "
                  f"회전 {self.angular_speed:.2f}rad/s")
        if changed:
            self.publish_event(f"속도 변경: {status}")
            return f"[SPEED] 속도를 설정했습니다. 현재 {status}"
        return f"[SPEED] 현재 속도: {status}"

    def lookup_current_pose(self):
        try:
            # We look up transform with a timeout to prevent locking if TF is not fully available yet
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, Time(), timeout=Duration(seconds=1.0))
            return (tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.rotation)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def get_current_location(self) -> str:
        cur = self.lookup_current_pose()
        if not cur:
            return "[ERR] 위치 확인 실패 (TF lookup failed)"
        cx, cy, _ = cur

        best, dist = None, 1e9
        for name, (x, y, qz, qw) in self.places.items():
            d = math.hypot(cx - x, cy - y)
            if d < dist:
                best, dist = name, d

        if dist < self.tolerance_m:
            return f"[HERE] {best}"
        else:
            return f"[HERE] Unknown (nearest={best}, dist={dist:.2f}m)"

    def list_locations(self):
        return list(self.places.keys())

    def explain_front_camera(self) -> str:
        if self.explain_fn:
            return self.explain_fn()
        return "[ERR] 이미지 분석 및 설명 기능을 사용할 수 없습니다."

    def navigate_to_pose(self, x, y, qz=0.0, qw=1.0, done_callback=None):
        self.publish_llm_active(True)
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
        goal.pose = ps

        self.get_logger().info("[NAV] Waiting for navigation action server...")
        self.ac.wait_for_server()
        self.get_logger().info(f"[NAV] Sending navigation goal: x={x:.2f}, y={y:.2f}")

        self._goal_future = self.ac.send_goal_async(goal)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("[NAV] Goal rejected")
                self.goal_handle = None
                self.publish_llm_active(False)
                if done_callback:
                    done_callback(False)
            else:
                self.get_logger().info("[NAV] Goal accepted")
                self.goal_handle = goal_handle
                self._result_future = goal_handle.get_result_async()
                self._result_future.add_done_callback(result_callback)

        def result_callback(future):
            self.goal_handle = None
            self.publish_llm_active(False)
            result = future.result()
            status = result.status
            if status == 4: # STATUS_SUCCEEDED
                self.get_logger().info("[NAV] Goal reached successfully!")
                if done_callback:
                    done_callback(True)
            else:
                self.get_logger().warn(f"[NAV] Navigation failed with status: {status}")
                if done_callback:
                    done_callback(False)

        self._goal_future.add_done_callback(goal_response_callback)

    def move_to_location(self, place: str):
        if place not in self.places:
            return f"[ERR] Unknown place: {place}"

        x, y, qz, qw = self.places[place]
        self.navigate_to_pose(x, y, qz, qw)
        self.publish_event(f"'{place}'(으)로 이동 명령")
        return f"[NAV] Heading to {place}"

    def cancel_navigation(self) -> str:
        stopped = []
        if self.wander_enabled:
            self.set_wander(False)
            stopped.append("랜덤 이동")
        if self.goal_handle is not None:
            self.get_logger().info("[NAV] Canceling active navigation goal...")
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.publish_llm_active(False)
            stopped.append("네비게이션")
        # Publish zero velocity to stop the robot
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        if not stopped:
            return "[WARN] 진행 중인 이동 명령이 없습니다. 로봇은 정지 상태입니다."
        self.publish_event("정지 명령: " + ", ".join(stopped) + " 중지")
        return f"[NAV] {', '.join(stopped)}을(를) 중지하고 로봇을 정지시켰습니다."
