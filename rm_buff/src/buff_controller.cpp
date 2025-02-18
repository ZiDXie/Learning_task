#include "buff_controller.h"
#include "pluginlib/class_list_macros.h"

namespace rm_buff
{
void BuffController::reset()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis_a(a_min, a_max);
  std::uniform_real_distribution<double> dis_w(w_min, w_max);
  a_ = dis_a(gen);
  w_ = dis_w(gen);

  b_ = 2.090 - a_;

  start_t_ = ros::Time::now();
}

double BuffController::calculate_spd(const ros::Time& time)
{
  double t = (time - start_t_).toSec();
  return a_ * sin(w_ * t) + b_;
}

bool BuffController::init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
                          ros::NodeHandle& controller_nh)
{
  joint_ = effort_joint_interface->getHandle("rotating_joint");
  if (controller_nh.hasParam("pid"))
  {
    if (!pid_.init(ros::NodeHandle(controller_nh, "pid")))
    {
      ROS_ERROR("Failed to init pid");
      return false;
    }
  }

  reset();
  pid_.reset();

  return true;
}

void BuffController::update(const ros::Time& time, const ros::Duration& period)
{
  double target_vel = calculate_spd(time);
  double current_vel = joint_.getVelocity();

  double error = target_vel - current_vel;
  double effort = pid_.computeCommand(error, period);

  joint_.setCommand(effort);
}
}  // namespace rm_buff

PLUGINLIB_EXPORT_CLASS(rm_buff::BuffController, controller_interface::ControllerBase)
