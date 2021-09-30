
#include <kimm_franka_controllers/basic_franka_controller.h>
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
  

  return true;
}

void BasicFrankaController::starting(const ros::Time& time) {
  

}


void BasicFrankaController::update(const ros::Time& time, const ros::Duration& period) {
  


}

} // namespace kimm_franka_controllers


PLUGINLIB_EXPORT_CLASS(kimm_franka_controllers::BasicFrankaController, controller_interface::ControllerBase)
