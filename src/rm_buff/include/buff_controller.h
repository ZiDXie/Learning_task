#ifndef BUFF_CONTROLLER_H
#define BUFF_CONTROLLER_H

#pragma once

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "random"
#include "rm_buff/buffConfig.h"
#include "dynamic_reconfigure/server.h"

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

  // 动态参数服务器
  std::shared_ptr<dynamic_reconfigure::Server<rm_buff::buffConfig> > server_;
  bool use_dynamcic;

  void pid_cb(rm_buff::buffConfig& config, uint32_t level);

  // 参数
  enum
  {
    STOP,
    START
  };

  void stop(const ros::Time& time, const ros::Duration& period);

  void start(const ros::Time& time, const ros::Duration& period);

  int mode = STOP;

  bool use_feedforward;
  double kf;

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
