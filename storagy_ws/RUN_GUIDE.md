# SAR 로봇 — 빨강 추적 네비게이션 실행 가이드

> 빨간 물체를 카메라로 탐지 → depth로 거리/3D 좌표 계산 → Nav2 goal 전송 → 로봇 이동.

## 0. 환경 (자동 설정됨)

`~/.bashrc`에서 새 터미널마다 아래가 자동 적용되므로 별도로 export/source 안 해도 됩니다.

```bash
export ROS_LOCALHOST_ONLY=1      # ★ 모든 노드가 동일하게 1이어야 토픽 데이터가 흐름
export ROS_DOMAIN_ID=1
source /opt/ros/humble/setup.bash
source /home/storagy/Desktop/storagy_ws/install/local_setup.bash
```

---

## 실행 순서 (터미널 3개)

### 터미널 1 — Orbbec 카메라 (color + depth)

> ★★ 반드시 depth 프로파일(640×400 / Y11)을 지정해야 합니다.
> 옵션 없이 실행하면 기본값 640×480·Y16으로 시도하다가 이 카메라(SV1301S_U3)가
> 지원하지 않아 **즉시 크래시**하고 depth가 안 나옵니다.

```bash
ros2 run orbbec_camera orbbec_camera_node --ros-args \
  -p enable_color:=true -p enable_depth:=true -p enable_ir:=false \
  -p depth_registration:=true \
  -p depth_width:=640 -p depth_height:=400 -p depth_fps:=30 -p depth_format:=Y11
```

### 터미널 2 — Nav2 + 미션 brinup (red_detector 포함)

```bash
ros2 launch storagy bringup.launch.py map:=/home/storagy/maps/2026_amr.yaml
```

- 이 launch가 **nav2 + amcl + map_server + red_detector**를 함께 띄웁니다.
  → red_detector는 따로 실행하지 마세요(중복 노드 발생).
- 실행 후 **RViz에서 `2D Pose Estimate`로 초기 위치를 지정**해야 AMCL이 위치추정을 시작하고
  `map → odom` TF가 생깁니다. (이게 없으면 좌표 변환 실패로 이동 안 함)

### 터미널 3 — 빨강 추적 네비게이터

```bash
cd ~/Desktop/storagy_ws
source install/setup.bash
python3 red_navigator.py --ros-args -p send_goal:=true
```

- `send_goal:=true` 여야 실제 Nav2 goal을 보냅니다. (false면 로그만 찍고 이동 안 함)

> 빨간 물체를 카메라에 비추면 로봇이 그 지점 앞(stop_distance 0.5m)까지 이동합니다.

---

## ⚠️ 주의사항

- **카메라/red_detector를 중복 실행하지 말 것.** Orbbec은 물리 장치가 1개라
  두 번째 카메라 노드는 장치를 못 열어 실패합니다. 이전 세션 잔여 노드가 살아있으면
  먼저 정리하세요:
  ```bash
  pkill -f orbbec_camera_node
  pkill -f red_detector_node      # bringup 재실행 전 잔여 detector 정리용
  ```
- 모든 터미널의 `ROS_LOCALHOST_ONLY` 값이 **동일(=1)** 해야 합니다. 섞이면 노드는
  보이는데 토픽 데이터(echo/hz)가 안 흐릅니다.

---

## 빠른 점검 (문제 시)

```bash
# depth가 실제로 발행되는지 (Publisher count ≥ 1, rate 표시되어야 정상)
ros2 topic info /depth/image_raw
ros2 topic hz   /depth/image_raw

# 카메라 내부 파라미터
ros2 topic hz /depth/camera_info

# 위치추정 됐는지 (값이 나와야 함 = 2D Pose Estimate 완료)
ros2 topic echo /amcl_pose --once

# 카메라 ↔ 맵 좌표 변환 가능한지
ros2 run tf2_ros tf2_echo map camera_color_optical_frame

# Nav2 액션 서버
ros2 action list | grep navigate_to_pose

# 빨강 탐지 신호
ros2 topic echo /red_detected
ros2 topic echo /red_target_point   # z = 화면 내 빨강 면적 비율
```

### 증상별 원인
| 증상 | 확인 | 원인 |
|---|---|---|
| 빨강 감지되는데 안 움직임 | `/depth/image_raw` Publisher 0 | 카메라 depth 프로파일 오류/중복 노드 → 터미널 1 재실행 |
| 〃 | `/amcl_pose` 비어있음 | 2D Pose Estimate 안 함 → 위치추정 필요 |
| 〃 | tf2_echo "map ... does not exist" | AMCL 미동작(위치추정) |
| 토픽 echo가 전부 빈값 | `echo $ROS_LOCALHOST_ONLY` | 터미널 간 값 불일치 |
