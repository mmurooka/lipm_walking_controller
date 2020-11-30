/*
 * Copyright (c) 2020, CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <lipm_walking/Controller.h>
#include <lipm_walking/State.h>

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
  void start() override;
  void teardown() override;
  void runState() override;
  /// Always true
  bool checkTransitions() override;

 private:
  void setupLogger(mc_control::fsm::Controller & ctl);

  void setupGui(mc_control::fsm::Controller & ctl);

  std::shared_ptr<mc_tasks::force::AdmittanceTask> left_admit_task_;
  std::shared_ptr<mc_tasks::force::AdmittanceTask> right_admit_task_;

  int reach_phase_ = 1;

  Eigen::Vector3d target_left_hand_force_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_right_hand_force_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_left_hand_force_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_right_hand_force_ = Eigen::Vector3d::Zero();

  std::vector<std::pair<Eigen::Vector3d, sva::ForceVecd> > ext_wrenches_;

  double first_ = true;

  double hand_force_rate_limit_ = 50;

  mc_control::fsm::Contact left_hand_contact_;
  mc_control::fsm::Contact right_hand_contact_;
};

} // namespace states

} // namespace lipm_walking
