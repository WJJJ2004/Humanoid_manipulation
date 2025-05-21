## 🚀 실행 방법

```bash
# 패키지 빌드
colcon build --packages-select arm_ctrl
source install/setup.bash

# 노드 실행
ros2 launch arm_ctrl arm_ctrl.launch.py
