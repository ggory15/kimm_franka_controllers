
#include <kimm_franka_controllers/basic_franka_controller_real.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace kimm_franka_controllers
{

bool BasicFrankaController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{

  ros::Subscriber ctrl_type_sub = node_handle.subscribe("/real_robot/ctrl_type", 1, &BasicFrankaController::ctrltypeCallback, this);
  ee_state_pub_ = node_handle.advertise<geometry_msgs::Transform>("ns0/real_robot/ee_state", 5);
  ee_state_msg_ = geometry_msgs::Transform();

  isgrasp_ = false;
  
  gripper_ac_.waitForServer();

  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  //keyboard event
  mode_change_thread_ = std::thread(&BasicFrankaController::modeChangeReaderProc, this);

  ctrl_ = new RobotController::FrankaWrapper("ns0", false, node_handle);
  ctrl_->initialize();

  
  return true;
}

void BasicFrankaController::starting(const ros::Time& time) {
  dq_filtered_.setZero();
  time_ = 0.;
}


void BasicFrankaController::update(const ros::Time& time, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();

  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  std::array<double, 49> massmatrix_array = model_handle_->getMass();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  
  Eigen::Map<Vector7d> gravity(gravity_array.data());
  robot_g_ = gravity;
  Eigen::Map<Matrix7d> mass_matrix(massmatrix_array.data());
  robot_mass_ = mass_matrix;
  Eigen::Map<Vector7d> non_linear(coriolis_array.data());
  robot_nle_ = non_linear;
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  robot_J_ = jacobian;
  Eigen::Map<Vector7d> franka_q(robot_state.q.data());
  Eigen::Map<Vector7d> franka_dq(robot_state.dq.data());
  
  // Filtering
  double cutoff = 20.0; // Hz //20
  double RC = 1.0 / (cutoff * 2.0 * M_PI);
  double dt = 0.001;
  double alpha = dt / (RC + dt);
  
  dq_filtered_ = alpha * franka_dq + (1 - alpha) * dq_filtered_;
 
  // Franka update
  ctrl_->franka_update(franka_q, franka_dq);

  // HQP thread
  if (calculation_mutex_.try_lock())
  {
    calculation_mutex_.unlock();
    if (async_calculation_thread_.joinable())
      async_calculation_thread_.join();

    async_calculation_thread_ = std::thread(&BasicFrankaController::asyncCalculationProc, this);
  }

  ros::Rate r(30000);
  for (int i = 0; i < 9; i++)
  {
    r.sleep();
    if (calculation_mutex_.try_lock())
    {
      calculation_mutex_.unlock();
      if (async_calculation_thread_.joinable())
        async_calculation_thread_.join();
      break;
    }
  }
  
  // if (print_rate_trigger_()) {
  //   ROS_INFO_STREAM("tau :" << franka_torque_.transpose());
  // }

  
   for (int i = 0; i < 7; i++)
      joint_handles_[i].setCommand(0.0);
  
    time_ += 0.001;

}
void BasicFrankaController::stopping(const ros::Time& time){
    ROS_INFO("Robot Controller::stopping");
}
void BasicFrankaController::ctrltypeCallback(const std_msgs::Int16ConstPtr &msg){
    calculation_mutex_.lock();
    ROS_WARN("%d", msg->data);
    
    if (msg->data != 899){
        int data = msg->data;
        ctrl_->ctrl_update(data);
    }
    else{
        if (isgrasp_){
            isgrasp_=false;
            franka_gripper::MoveGoal goal;
            goal.speed = 0.1;
            goal.width = 0.08;
            gripper_ac_.sendGoal(goal);
        }
        else{
            isgrasp_=true;
            franka_gripper::MoveGoal goal;
            goal.speed = 0.1;
            goal.width = 0.0;
            gripper_ac_.sendGoal(goal);
        }
    }
    calculation_mutex_.unlock();
}
void BasicFrankaController::asyncCalculationProc(){
  calculation_mutex_.lock();

  ctrl_->compute(time_);
  
  ctrl_->franka_output(franka_qacc_); // this is only for simulation mode

  ctrl_->state(state_);

  franka_torque_ = robot_mass_ * franka_qacc_ + robot_nle_;

  if (ctrl_->ctrltype() != 0)
      UpdateMob();
  else
      InitMob();

  this->setFrankaCommand();

  calculation_mutex_.unlock();
}

void BasicFrankaController::setFrankaCommand(){
  
  MatrixXd Kd(7, 7);
  Kd.setIdentity();
  Kd = 2.0 * sqrt(5.0) * Kd;
  Kd(5, 5) = 0.2;
  Kd(4, 4) = 0.2;
  Kd(6, 6) = 0.2; // this is practical term
  franka_torque_ -= Kd * dq_filtered_;  
}

void BasicFrankaController::getEEState(){
    Vector3d pos;
    Quaterniond q;
    ctrl_->ee_state(pos, q);

    ee_state_msg_.translation.x = pos(0);
    ee_state_msg_.translation.y = pos(1);
    ee_state_msg_.translation.z = pos(2);

    ee_state_msg_.rotation.x = q.x();
    ee_state_msg_.rotation.y = q.y();
    ee_state_msg_.rotation.z = q.z();
    ee_state_msg_.rotation.w = q.w();
    ee_state_pub_.publish(ee_state_msg_);
}

void BasicFrankaController::modeChangeReaderProc(){
  while (!quit_all_proc_)
  {
    char key = getchar();
    key = tolower(key);
    calculation_mutex_.lock();

    int msg = 0;
    switch (key){
      case 'g': //gravity
          msg = 0;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Gravity mode" << endl;
          cout << " " << endl;
          break;
      case 'h': //home
          msg = 1;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Initialization" << endl;
          cout << " " << endl;
          break;
      case 'a': //home
          msg = 2;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Move EE with wholebody Motion" << endl;
          cout << " " << endl;
          break;    
      case 's': //home
          msg = 3;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Move EE with wholebody Motion" << endl;
          cout << " " << endl;
          break;    
      case 'd': //home
          msg = 4;
          ctrl_->ctrl_update(msg);
          cout << " " << endl;
          cout << "Move EE with wholebody Motion" << endl;
          cout << " " << endl;
          break;   
      case 'z': //grasp
          if (isgrasp_){
              cout << "Release hand" << endl;
              isgrasp_ = false;
          }
          else{
              cout << "Grasp object" << endl;
              isgrasp_ = true; 
          }
          break;
    }
    
    calculation_mutex_.unlock();
  }
}

} // namespace kimm_franka_controllers


PLUGINLIB_EXPORT_CLASS(kimm_franka_controllers::BasicFrankaController, controller_interface::ControllerBase)
