#ifndef BUFF_CONTROLLER_H
#define BUFF_CONTROLLER_H

#pragma once

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "random"

namespace rm_buff
{
class BuffController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  BuffController() = default;

  ~BuffController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void reset();

  double calculate_spd(const ros::Time& time);

  hardware_interface::JointHandle joint_;
  control_toolbox::Pid pid_;

  // 参数
  ros::Time start_t_;
  double a_;
  double b_;
  double w_;

  const double a_min = 0.780;
  const double a_max = 1.045;
  const double w_min = 1.884;
  const double w_max = 2.000;
};
}  // namespace rm_buff

#endif  // BUFF_CONTROLLER_H
