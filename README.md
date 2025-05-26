# Robit Humanoid Arm Control System

## 📦 패키지 구성

### 1. `arm_ctrl`
- **기능**: IK 계산, trajectory 생성, 조인트 각도 퍼블리시 및 Gazebo 시각화.
- **메시지**: 
  - `ArmCtrlCmd.msg`: EE 목표 좌표 명령
  - `ArmCtrlFlag.msg`: 마스터 시스템 준비 플래그
  - `ArmJointAngle.msg`: 조인트 각도 명령
- **노드**:
  - `main_node`: 메인 제어 흐름 및 퍼블리셔 포함
- **설정파일**:
  - `arm_params.yaml`: IK 파라미터
  - `controller.yaml`: Gazebo용 controller 설정
### 2. `robit_humanoid_offset_22DOF_description`
- **기능**: 22자유도 로봇 URDF 및 Gazebo 시뮬레이션 구성
- **구성**:
  - `urdf/robit_humanoid_offset_22DOF.xacro`: 로봇 모델 정의
  - `meshes/`: STL 파일 포함
  - `launch/gazebo.launch.py`: Gazebo 환경 설정

## 🧠 주요 기능 설명

- **EE 목표 지정 → IK 계산 → 조인트 명령 생성 → Gazebo 시각화**까지 일련의 흐름 자동화
- 좌/우팔 구분하여 조인트 명령 발행
- Gazebo 상의 실제 URDF와 controller를 통해 시뮬레이션

## 🚀 실행 방법

### 1. 빌드
```bash
colcon build --packages-select arm_ctrl robit_humanoid_offset_22DOF_description
source install/setup.bash

