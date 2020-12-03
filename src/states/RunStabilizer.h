/*
 * Copyright (c) 2020, CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <cmath>

#include <lipm_walking/Controller.h>
#include <lipm_walking/State.h>

#include <mc_rtc/ros.h>
#include <ros/ros.h>

namespace lipm_walking
{

/** States of the controller's finite state machine.
 *
 */
namespace states
{

/**
 * Adds/removes the global stabilizer task to the QP
 */
struct RunStabilizer : State
{
  RunStabilizer();

  void start() override;
  void teardown() override;
  void runState() override;
  /// Always true
  bool checkTransitions() override;

 private:
  void setupLogger(mc_control::fsm::Controller & ctl);

  void setupGui(mc_control::fsm::Controller & ctl);

  inline sva::PTransformd projGround(const sva::PTransformd& pose) const
  {
    Eigen::Vector3d pos = pose.translation();
    pos[2] = 0;
    return sva::PTransformd(
        sva::RotZ(std::acos(Eigen::Vector3d::UnitX().dot(
            pose.rotation() * Eigen::Vector3d::UnitX()))),
        pos);
  }

  inline sva::PTransformd footMidpose(mc_control::fsm::Controller & ctl) const
  {
    return sva::interpolate(
        projGround(ctl.robot().surfacePose("LeftFoot")),
        projGround(ctl.robot().surfacePose("RightFoot")),
        0.5);
  }

  std::shared_ptr<mc_tasks::force::AdmittanceTask> left_admit_task_;
  std::shared_ptr<mc_tasks::force::AdmittanceTask> right_admit_task_;

  int reach_phase_ = 1;

  Eigen::Vector3d target_left_hand_force_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_right_hand_force_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_left_hand_force_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_right_hand_force_ = Eigen::Vector3d::Zero();

  std::vector<std::pair<Eigen::Vector3d, sva::ForceVecd> > ext_wrenches_;

  Eigen::Vector3d target_cnoid_ext_force_offset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_cnoid_ext_force_offset_ = Eigen::Vector3d::Zero();

  double first_ = true;

  double hand_force_rate_limit_ = 50;

  mc_control::fsm::Contact left_hand_contact_;
  mc_control::fsm::Contact right_hand_contact_;

  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher ext_force_pub_;

  std::vector<sva::PTransformd> rel_target_poses_;
};

} // namespace states

} // namespace lipm_walking
