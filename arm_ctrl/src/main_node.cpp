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
  this->declare_parameter<double>("L1", 0.0);
  this->declare_parameter<double>("L2", 0.0);
  this->declare_parameter<double>("L3", 0.0);
  this->declare_parameter<double>("L4", 0.0);
  this->declare_parameter<double>("d1", 0.0);
  this->declare_parameter<double>("d2", 0.0);
  this->declare_parameter<double>("offset_theta1", 0.0);
  this->declare_parameter<double>("offset_theta2", 0.0);
  this->declare_parameter<double>("offset_theta3", 0.0);

  ik_pub_ = this->create_publisher<humanoid_interfaces::msg::Master2IkMsg>("/master_to_ik", 10);
  joint_angle_pub_ = this->create_publisher<arm_ctrl::msg::ArmJointAngle>("/joint_angle_cmd", 10);
  traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/arm_controller/joint_trajectory", 10);


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

void MainNode::publishJointCommands(const Eigen::Vector3d &q, bool is_left_arm)
{
  // 사용자 정의 메시지
  arm_ctrl::msg::ArmJointAngle angle_msg;
  // 시뮬레이터 메시지
  trajectory_msgs::msg::JointTrajectory sim_msg;

  sim_msg.joint_names = {
    "rotate_0", "rotate_1", "rotate_2", "rotate_3", "rotate_4", "rotate_5",
    "rotate_6", "rotate_7", "rotate_8", "rotate_9", "rotate_10", "rotate_11",
    "rotate_12", "rotate_13", "rotate_14", "rotate_15", "rotate_16", "rotate_17",
    "rotate_18", "rotate_19", "rotate_20", "rotate_21"
  };

  trajectory_msgs::msg::JointTrajectoryPoint traj_point;
  traj_point.positions.resize(22, 0.0);

  if (is_left_arm) {
    angle_msg.rotate_0 = -1.0 * q(0);
    angle_msg.rotate_2 = -1.0 * q(1);
    angle_msg.rotate_4 = -1.0 * q(2);

    angle_msg.rotate_1 = angle_msg.rotate_3 = angle_msg.rotate_5 = 0.0;

    traj_point.positions[0] = angle_msg.rotate_0;
    traj_point.positions[2] = angle_msg.rotate_2;
    traj_point.positions[4] = angle_msg.rotate_4;
  } else {
    angle_msg.rotate_0 = angle_msg.rotate_2 = angle_msg.rotate_4 = 0.0;

    angle_msg.rotate_1 = q(0);
    angle_msg.rotate_3 = q(1);
    angle_msg.rotate_5 = q(2);

    traj_point.positions[1] = angle_msg.rotate_1;
    traj_point.positions[3] = angle_msg.rotate_3;
    traj_point.positions[5] = angle_msg.rotate_5;
  }

  traj_point.time_from_start = rclcpp::Duration::from_seconds(0.1);
  sim_msg.points.push_back(traj_point);

  joint_angle_pub_->publish(angle_msg);
  traj_pub_->publish(sim_msg);
}


void MainNode::mainLoop()
{
  // if (!is_master_ready_ || !ee_received_ || !imu_yaw_set_)
  // {
  //   if(!is_master_ready_) {
  //     RCLCPP_INFO(this->get_logger(), "Waiting for master ready...");
  //     return;
  //   }
  //   else if(!ee_received_) {
  //     RCLCPP_INFO(this->get_logger(), "Waiting for EE target...");
  //     return;
  //   }
  //   else if(!imu_yaw_set_) {
  //     RCLCPP_INFO(this->get_logger(), "Waiting for IMU data...");
  //     return;
  //   }
  // }

  if(!ee_received_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for EE target...");
      return;
    }

  Eigen::Vector3d current_target(ee_target_.x, ee_target_.y, ee_target_.z);
  bool is_left_arm = (ee_target_.y < 0);

  switch(loop_squence)
  {
    case 0: // task space 까지 접근
      cout << "Reaching to Task Space" << endl;
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

    case 1: // EE 경로 생성
      if (!trajectory_generated_) {
        double push_yaw = initial_yaw_ + M_PI / 2.0;  // 벽과 수직하는 경로로 밀기
        trajectory_ = planner_.generateStraightPushTrajectory(current_target,ik_module_->offset_coord, push_yaw, 5.0, 0.1);
        trajectory_index_ = 0;
        trajectory_generated_ = true;
        RCLCPP_INFO(this->get_logger(), "Trajectory generated.");
        loop_squence = 2;
      }
      break;
    case 2:
      cout << "Ready to Manipulate" << endl;
      if (trajectory_index_ < trajectory_.size())
      {
        auto point = trajectory_[trajectory_index_++]; // Eigen::Vector3d
        auto q_opt = ik_module_->computeIK(point, is_left_arm); // optional

        if (q_opt.has_value())
        {
          publishJointCommands(q_opt.value(), is_left_arm);
          RCLCPP_INFO(this->get_logger(), "Publishing joint commands: (%f, %f, %f)"
                                        , q_opt->x(), q_opt->y(), q_opt->z());
        }
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
        trajectory_generated_ = false;
        loop_squence = 0;
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