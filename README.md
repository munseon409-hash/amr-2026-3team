# AMR 2026 — 3team Storagy Robot

산불/위험 상황을 자율 순찰하며 감지·접근·보고하는 ROS2 기반 자율주행 로봇(Storagy) 프로젝트입니다.

---

## 저장소 구조

이 저장소는 **3개의 독립적인 작업 영역**으로 구성되어 있습니다. 같은 이름의 패키지(`motor_driver2`, `storagy`)가 두 워크스페이스에 각각 존재하지만, 서로 다른 시점/용도로 수정된 **독립 버전**이므로 합치지 않고 그대로 보존했습니다.

```
amr-2026-3team/
├── ros2_ws/                 # SAR 미션(BT) 통합 워크스페이스
│   ├── storagy/              # 로봇 메인 패키지 (URDF, launch, map, param, rviz)
│   ├── motor_driver2/         # 모터 드라이버
│   └── sar_bt_mission/        # SAR 임무 Behavior Tree + C++ 액션 노드
│
├── storagy_ws/               # 빨간색 표적 추적 워크스페이스
│   ├── storagy/               # 로봇 메인 패키지 (위와 별도 버전)
│   ├── motor_driver2/          # 모터 드라이버 (위와 별도 버전)
│   ├── red_detector/            # 카메라 영상에서 빨간색 영역 검출
│   ├── wall_follower/            # 벽 추종 주행 + YOLO 표지판 인식
│   ├── wall_follower_msgs/        # wall_follower용 커스텀 메시지(Stop)
│   └── RUN_GUIDE.md             # 빨간색 추적 네비게이션 실행 가이드
│
└── storagy_llm/              # 자연어(LLM) 기반 로봇 제어 에이전트
```

> **제외된 항목**: `build/`, `install/`, `log/` (빌드 산출물), `.env` (API 키 등 민감정보 — `.env.example`로 대체), `*.onnx`/`*.whl` (대용량 모델/패키지 파일), `navigation2`/`OrbbecSDK_ROS2`/`BehaviorTree.ROS2` (외부 공식 오픈소스, 별도 clone 필요)

---

## 패키지별 설명

### 1. `storagy` — 로봇 메인 패키지

로봇의 물리적 정의(URDF)와 시뮬레이션·내비게이션 실행에 필요한 모든 설정을 담은 핵심 패키지입니다.

| 폴더 | 내용 |
|---|---|
| `urdf/` | 로봇 본체·센서·휠 구조 정의 (storagy.urdf) |
| `launch/` | 부팅(`bringup`), 하드웨어 구동(`hardware_bringup`), SLAM(`cartographer`), Nav2(`navigation2/`) 등 실행 스크립트 |
| `map/` | SLAM으로 생성한 점유격자지도(`map.pgm`/`.yaml`) |
| `param/` | Nav2·라이다 등 파라미터 설정 (`navigation2/storagy.yaml`, `setting.yaml`) |
| `config/sick_scan_xd/` | SICK TiM 라이다 드라이버 설정 |
| `config/cartographer/` | Cartographer SLAM 설정 |
| `rviz/` | RViz 시각화 레이아웃 |
| `behavior_trees/` | Nav2 기본 제공 BT(재시도·복구 포함 경로 계획) XML |
| `scripts/fire_detection_bt.py` | `py_trees` 기반 화재 감지·접근 행동 트리 — `/red_detected`, `/red_target_point`를 구독해 화재 의심 지점으로 이동하고 `/tts_output`(음성 안내), `/fire_location_report`(위치 보고)를 발행 |

**ros2_ws/storagy**와 **storagy_ws/storagy**는 launch·param 일부 내용이 서로 다른 독립 버전입니다 (어느 쪽이 최신인지는 작업 시점에 따라 다르므로 사용 전 확인 필요).

### 2. `motor_driver2` — 모터 드라이버

시리얼 통신으로 모터 제어 보드와 직접 통신하는 노드입니다. `/cmd_vel`(속도 명령)을 받아 모터로 전달하고, 엔코더 값을 읽어 `/odom`(오도메트리)과 TF(`odom→base_link`)를 발행합니다. 비상정지(Emergency), 릴레이, LED 제어 레지스터도 함께 다룹니다.

### 3. `sar_bt_mission` — SAR(Search and Rescue) 임무 통합 워크스페이스

**BehaviorTree.CPP v4 + BehaviorTree.ROS2**로 작성된 임무 제어 트리(`behavior_trees/sar_mission.xml`)가 핵심입니다.

- **순찰(Patrol) 분기**: 정의된 웨이포인트(WP1→WP2)를 Nav2로 무한 순회하며, 각 지점 도착 시 좌우로 스캔(40°→80°→복귀) 후 5초 대기.
- **긴급 구조(Emergency) 분기**: `raw_detection`(위험 감지) 신호가 true가 되는 순간, `ReactiveFallback` 구조에 의해 순찰을 즉시 인터럽트하고 다음을 순서대로 수행합니다.
  1. `AlignToTarget` — 화면 x좌표 기준 정렬, 전방 1.5m 지점을 맵 좌표로 투영
  2. `NavigateToTarget` — Nav2로 해당 좌표까지 이동 (Backup/Spin/ClearCostmap 등 내장 복구 포함)
  3. `SafeDistanceReached` — 라이다로 1.0m 이내 안전 접근 확인
  4. `VoiceInteraction` — 음성 안내 및 가상 대화 (5초)
  5. `PublishRescueReport` — 관제 UI 전송용 구조 보고서(JSON) 발행
- 각 액션/조건 노드(`AlignToTarget`, `NavigateToTarget`, `NavigateToWaypoint`, `ScanAtWaypoint`, `SafeDistanceReached`, `VoiceInteraction`, `PublishRescueReport`, `LockTarget`, `MarkTargetServed`, `TargetConfirmed`)는 `src/`, `include/`에 C++로 구현되어 있습니다.
- `scripts/red_detector_node.py`도 이 패키지에 포함되어 함께 실행됩니다.

### 4. `red_detector` — 빨간색 표적 검출

카메라 영상(`/camera/color/image_raw`)을 구독해 OpenCV로 빨간색 영역을 검출하고 다음을 발행합니다.

- `/red_detected` (`std_msgs/Bool`) — 빨간색 발견 여부
- `/red_target_point` (`geometry_msgs/Point`) — 화면 좌표 기준 중심점(x,y: -1.0~1.0)과 면적 비율(z)
- `/red_detection/image` (`sensor_msgs/Image`) — 박스가 그려진 디버그 영상

> `image_topic` 파라미터로 입력 토픽을 바꿀 수 있습니다(기본값 `/color/image_raw`). 카메라 노드가 `/camera/color/image_raw`로 발행하는 경우 `--ros-args -p image_topic:=/camera/color/image_raw`를 지정해야 합니다.

### 5. `wall_follower` + `wall_follower_msgs` — 벽 추종 + 표지판 인식

- `wall_follower.py`: 라이다 7방향(N/NE/E/NW/W 등) 거리 측정으로 상태머신(방황→벽 회전→벽 추종)을 구성해 벽을 따라 주행합니다. `/sign`(`Stop` 메시지) 수신 시 3초간 정지합니다.
- `camera_subscriber.py`: 카메라 영상을 YOLOv5 기반 학습 모델(`best.onnx`, ONNX Runtime으로 추론)에 입력해 표지판(정지 신호 등)을 인식하고, 신뢰도 0.85 이상이면 `/sign` 토픽으로 정지 신호를 발행합니다.
- `wall_follower_msgs/Stop.msg`: `bool sign` 필드 하나로 구성된 커스텀 메시지.
- `yolov5/`: 추론에 사용하는 YOLOv5 소스 (모델 가중치 `best.onnx`, 의존 패키지 `onnx-*.whl`은 용량 문제로 저장소에서 제외 — 별도 보관 필요).

### 6. `storagy_llm` — 자연어 로봇 제어 에이전트

LangChain + LangGraph + OpenAI(`gpt-4o-mini`) 기반의 음성/텍스트 명령 에이전트입니다.

- `agent_service.py`: ROS2 서비스(`llm_agent`)로 동작. 카메라 영상을 구독해 현재 시야를 함께 참고하며, `MemorySaver`로 세션별 대화 기록을 유지합니다.
- `robot_tools.py`: LLM이 호출 가능한 도구(Tool) 목록 정의 — 현재 위치 조회, 이동 가능 장소 목록, 특정 장소로 이동, 전방 카메라 설명, 이동 취소, 랜덤 배회 시작/중지 등을 자연어 명령과 매핑합니다.
- `agent_client.py`: 에이전트 서비스 호출 클라이언트.
- `web_dashboard.py`: 로봇 상태를 보여주는 웹 대시보드.
- `params/points.yaml`: 이동 가능한 장소명과 좌표(x, y, qz, qw) 목록.
- `params/prompt.yaml`: 에이전트 시스템 프롬프트.
- `web/`: 대시보드 프론트엔드(HTML/JS/CSS).

> `.env`(OpenAI API 키 등)는 저장소에 포함하지 않습니다. `storagy_llm/storagy_llm/.env.example`을 참고해 `.env`를 직접 만들어야 합니다.

---

## 실행 가이드

빨간색 표적 추적 시나리오의 상세 실행 순서는 `storagy_ws/RUN_GUIDE.md`를 참고하세요. 핵심 요약:

1. **카메라 노드** 실행 (Orbbec, color/depth 프로파일 지정 필수)
2. **Nav2 + bringup** 실행 (`red_detector` 등 자동 포함 — 중복 실행 금지)
3. RViz에서 **`2D Pose Estimate`**로 초기 위치 지정 (AMCL 위치추정 시작)
4. **추적/임무 노드** 실행 (`red_navigator.py` 또는 `sar_bt_mission` BT)

공통 주의사항:
- 같은 노드(카메라, red_detector 등)를 중복 실행하지 않도록 항상 `ps aux | grep <노드명>`으로 먼저 확인합니다.
- 모든 터미널의 `ROS_LOCALHOST_ONLY`, `ROS_DOMAIN_ID` 값이 일치해야 토픽 데이터가 흐릅니다.
- `red_detector_node`의 `image_topic` 파라미터가 실제 카메라 발행 토픽과 일치하는지 확인합니다.

---

## 환경 / 의존성

- ROS2 Humble
- Nav2, Cartographer, SICK scan driver(`sick_scan_xd`)
- BehaviorTree.CPP v4, BehaviorTree.ROS2
- OpenCV, cv_bridge, ONNX Runtime
- LangChain, LangGraph, OpenAI API (`storagy_llm`)
- Orbbec 카메라 SDK (`OrbbecSDK_ROS2`, 별도 설치 필요)

외부 오픈소스 패키지(`navigation2`, `OrbbecSDK_ROS2`, `BehaviorTree.ROS2`)는 이 저장소에 포함되어 있지 않으며, 각 공식 저장소에서 별도로 clone하여 워크스페이스에 추가해야 합니다.
