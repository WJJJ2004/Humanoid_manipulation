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
 - **트리**
'''
	├── CMakeLists.txt
	├── config
	│   ├── arm_params.yaml
	│   └── controller.yaml
	├── include
	│   ├── ik_module.hpp
	│   ├── main_node.hpp
	│   └── trajectory_planner.hpp
	├── launch
	│   ├── arm_ctrl.launch.py
	│   └── sim.launch.py
	├── msg
	│   ├── ArmCtrlCmd.msg
	│   ├── ArmCtrlFlag.msg
	│   └── ArmJointAngle.msg
	├── package.xml
	└── src
	    ├── ik_module.cpp
	    ├── main_node.cpp
	    └── trajectory_planner.cpp

	5 directories, 15 files

'''
### 2. `robit_humanoid_offset_22DOF_description`
- **기능**: 22자유도 로봇 URDF 및 Gazebo 시뮬레이션 구성
- **구성**:
  - `urdf/robit_humanoid_offset_22DOF.xacro`: 로봇 모델 정의
  - `meshes/`: STL 파일 포함
  - `launch/gazebo.launch.py`: Gazebo 환경 설정
- **tree**
  '''
	├── config
	│   └── display.rviz
	├── launch
	│   ├── display.launch.py
	│   └── gazebo.launch.py
	├── meshes
	│   ├── ABD_Part_Up_L_1.stl
	│   ├── ABD_Part_Up_R_1.stl
	│   ├── ABD_Roll_L_1.stl
	│   ├── ABD_Roll_lower_L_1.stl
	│   ├── ABD_Roll_lower_R_1.stl
	│   ├── ABD_Roll_R_1.stl
	│   ├── Arm_End_Set_L_1.stl
	│   ├── Arm_End_set_R_1.stl
	│   ├── Arm_First_Set_L_1.stl
	│   ├── Arm_First_Set_R_1.stl
	│   ├── Arm_Mid_Set_L_1.stl
	│   ├── Arm_Mid_Set_R_1.stl
	│   ├── base_link.stl
	│   ├── Head_1.stl
	│   ├── Left_Foot_ABD_1.stl
	│   ├── Lower_Leg_L_1.stl
	│   ├── Lower_Leg_R_1.stl
	│   ├── MX_0_1.stl
	│   ├── MX_10_1.stl
	│   ├── MX_11_1.stl
	│   ├── MX_1_1.stl
	│   ├── MX_12_1.stl
	│   ├── MX_13_1.stl
	│   ├── MX_14_1.stl
	│   ├── MX_15_1.stl
	│   ├── MX_16_1.stl
	│   ├── MX_17_1.stl
	│   ├── MX_18_1.stl
	│   ├── MX_19_1.stl
	│   ├── MX_20_1.stl
	│   ├── MX_21_1.stl
	│   ├── MX_2_1.stl
	│   ├── MX_22_1.stl
	│   ├── MX_3_1.stl
	│   ├── MX_4_1.stl
	│   ├── MX_5_1.stl
	│   ├── Right_Foot_ABD_1.stl
	│   ├── Right_Upper_Leg_L_1.stl
	│   └── Right_Upper_Leg_R_1.stl
	├── package.xml
	├── resource
	│   └── robit_humanoid_offset_22DOF_description
	├── robit_humanoid_offset_22DOF_description
	│   └── __init__.py
	├── setup.cfg
	├── setup.py
	├── test
	│   ├── test_copyright.py
	│   ├── test_flake8.py
	│   └── test_pep257.py
	└── urdf
	    ├── materials.xacro
	    ├── robit_humanoid_offset_22DOF.gazebo
	    ├── robit_humanoid_offset_22DOF.trans
	    └── robit_humanoid_offset_22DOF.xacro

	7 directories, 54 files

'''
## 🧠 주요 기능 설명

- **EE 목표 지정 → IK 계산 → 조인트 명령 생성 → Gazebo 시각화**까지 일련의 흐름 자동화
- 좌/우팔 구분하여 조인트 명령 발행
- Gazebo 상의 실제 URDF와 controller를 통해 시뮬레이션

## 🚀 실행 방법

### 1. 빌드
```bash
colcon build --packages-select arm_ctrl robit_humanoid_offset_22DOF_description
source install/setup.bash

