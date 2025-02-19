#include "buff_controller.h"

#include <std_msgs/Float64.h>

#include "pluginlib/class_list_macros.h"

namespace rm_buff
{
void BuffController::pid_cb(buffConfig& config, uint32_t level)
{
  pid_.setGains(config.p, config.i, config.d, config.max, config.min);
  mode = config.mode;
  kf = config.kf;
  use_feedforward = config.use_feedforward;
  ROS_INFO("mode: %d", mode);
  ROS_INFO("p: %f, i: %f, d: %f, max: %f, min: %f", config.p, config.i, config.d, config.max, config.min);
  ROS_INFO("kf: %f", kf);
}

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

void BuffController::move(const ros::Time& time, const ros::Duration& period)
{
  if (mode == BIG && last_mode != BIG)
  {
    reset();
  }
  last_mode = mode;

  if (mode == STOP)
  {
    target_vel = 0;
  }
  else if (mode == BIG)
  {
    target_vel = calculate_spd(time);
  }
  else if (mode == SMALL)
  {
    target_vel = M_PI / 3;
  }
  else
  {
    mode = STOP;
  }
  double current_vel = joint_.getVelocity();
  double error = target_vel - current_vel;
  double effort = pid_.computeCommand(error, period);
  if (use_feedforward)
  {
    effort += kf * target_vel;
  }
  joint_.setCommand(effort);
}

void BuffController::target_vel_pub_()
{
  std_msgs::Float64 target_vel_msg;
  target_vel_msg.data = target_vel;
  target_vel_pub.publish(target_vel_msg);
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
  if (controller_nh.param<bool>("usedynamic", use_dynamcic, true))
  {
    server_ = std::make_shared<dynamic_reconfigure::Server<rm_buff::buffConfig> >(controller_nh);
    server_->setCallback(
        [this](auto&& PH1, auto&& PH2) { pid_cb(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); });
    ROS_INFO("Dynamic reconfigure server initialized");
  }

  controller_nh.param<bool>("usefeedforward", use_feedforward, false);

  target_vel_pub = controller_nh.advertise<std_msgs::Float64>("target_vel", 1);

  reset();
  pid_.reset();

  return true;
}

void BuffController::update(const ros::Time& time, const ros::Duration& period)
{
  move(time, period);
  target_vel_pub_();
}
}  // namespace rm_buff

PLUGINLIB_EXPORT_CLASS(rm_buff::BuffController, controller_interface::ControllerBase)
