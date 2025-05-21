#include "main_node.hpp"            // 클래스 정의
#include <memory>                   // std::make_shared
#include <functional>               // std::placeholders

using std::placeholders::_1;            // 콜백 바인딩용 placeholder
using namespace std;

MainNode::MainNode()
: Node("main_node"),
  is_master_ready_(false), imu_yaw_set_(false), ee_received_(false),
  trajectory_index_(0), trajectory_generated_(false)
{
  ik_pub_ = this->create_publisher<humanoid_interfaces::msg::Master2IkMsg>("/master_to_ik", 10);
  joint_angle_pub_ = this->create_publisher<arm_ctrl::msg::ArmJointAngle>("/joint_angle_cmd", 10);

  imu_sub_ = this->create_subscription<humanoid_interfaces::msg::ImuMsg>(
    "/imu_data", 10, std::bind(&MainNode::imuCallback, this, _1));
  cmd_sub_ = this->create_subscription<arm_ctrl::msg::ArmCtrlCmd>(
    "/arm_cmd", 10, std::bind(&MainNode::cmdCallback, this, _1));
  flag_sub_ = this->create_subscription<arm_ctrl::msg::ArmCtrlFlag>(
    "/arm_flag", 10, std::bind(&MainNode::flagCallback, this, _1));

  timer_ = this->create_wall_timer(30ms, std::bind(&MainNode::mainLoop, this));

  ik_module_ = std::make_unique<IKModule>(this);
}

void MainNode::imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  imu_yaw_ = msg->yaw;
  if (!imu_yaw_set_) {
    initial_yaw_ = imu_yaw_;
    imu_yaw_set_ = true;
  }

  RCLCPP_INFO(this->get_logger(), "IMU Yaw: %f", imu_yaw_);
}

void MainNode::cmdCallback(const arm_ctrl::msg::ArmCtrlCmd::SharedPtr msg)
{
  ee_target_.x = msg->x;
  ee_target_.y = msg->y;
  ee_target_.z = msg->z;
  ee_received_ = true;
    RCLCPP_INFO(this->get_logger(), "EE Target: (%f, %f, %f)", ee_target_.x, ee_target_.y, ee_target_.z);
}

void MainNode::flagCallback(const arm_ctrl::msg::ArmCtrlFlag::SharedPtr msg)
{
  is_master_ready_ = msg->is_master_ready;
  RCLCPP_INFO(this->get_logger(), "Master Ready: %d", is_master_ready_);
}

void MainNode::mainLoop()
{
  if (!is_master_ready_ || !ee_received_ || !imu_yaw_set_)
  {
    if(!is_master_ready_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for master ready...");
      return;
    }
    else if(!ee_received_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for EE target...");
      return;
    }
    else if(!imu_yaw_set_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for IMU data...");
      return;
    }
  }

  Eigen::Vector3d current_target(ee_target_.x, ee_target_.y, ee_target_.z);
  bool is_left_arm = (ee_target_.y < 0);

  switch(loop_squence)
  {
    case 0: // 워크 스페이스까지 접근
      cout << "Step 1: IK computation" << endl;
      if(ik_module_->isReachable(current_target, is_left_arm))
      {
        pub_walk_cmd(0.0, 0.0, 0.0,false); // 정지 명령
        cout << "Target reachable, sending stop command." << endl;
        rclcpp::sleep_for(1s);
        loop_squence = 1;
      }
      else
      {
        // ----- 전진/후진/사이드/회전 판단하여 보행 명령 생성 -----
        if(ee_target_.x > 0.1)
        {
          pub_walk_cmd(5.0, 0.0, 0.0,true); // 전진 명령
          cout << "Target unreachable, sending forward command." << endl;
          rclcpp::sleep_for(1s);
        }
        else if(ee_target_.y < 0.0)
        {
          pub_walk_cmd(0.0, 5.0, 0.0,true); // 전진 명령
          cout << "Target unreachable, sending forward command." << endl;
        }
        else if(ee_target_.y > 0.0)
        {
          pub_walk_cmd(0.0, -5.0, 0.0,true); // 전진 명령
          cout << "Target unreachable, sending forward command." << endl;
        }
      }
      break;

    case 1: // 팔 궤적 생성
      if (!trajectory_generated_) {
        double push_yaw = initial_yaw_ + M_PI / 2.0;  // 벽을 밀기 위해
        trajectory_ = planner_.generateStraightPushTrajectory(current_target,ik_module_->offset_coord, push_yaw, 20);
        trajectory_index_ = 0;
        trajectory_generated_ = true;
        RCLCPP_INFO(this->get_logger(), "Trajectory generated.");
        loop_squence = 2;
      }
      break;
    case 2:
      cout << "Step 2: IK computation" << endl;
      if (trajectory_index_ < trajectory_.size())
      {
        auto point = trajectory_[trajectory_index_++];
        auto q_opt = ik_module_->computeIK(point, is_left_arm);

        if (q_opt.has_value())
        {
          auto q = q_opt.value();
          arm_ctrl::msg::ArmJointAngle msg;
          msg.theta1 = q(0);
          msg.theta2 = q(1);
          msg.theta3 = q(2);
          msg.is_left_arm = is_left_arm;
          joint_angle_pub_->publish(msg);
          cout << "Joint angles: (" << q(0) << ", " << q(1) << ", " << q(2) << ")" << endl;
        }
      }

      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Invalid step");
      return;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainNode>());
  rclcpp::shutdown();
  return 0;
}
