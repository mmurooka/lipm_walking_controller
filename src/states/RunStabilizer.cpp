/*
 * Copyright (c) 2020, CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "RunStabilizer.h"

namespace lipm_walking
{

void states::RunStabilizer::start()
{
  auto & ctl = controller();
  ctl.solver().addTask(stabilizer());
  stabilizer()->setApplyComOffset(true);

  // add end-effector task
  left_hand_task_ = std::make_shared<mc_tasks::EndEffectorTask>(
      "Lhand_Link0_Plan2", Eigen::Vector3d(0, 0, -0.14),
      ctl.robots(), 0, 5.0, 500.0);
  ctl.solver().addTask(left_hand_task_);
  left_hand_task_->reset();
  right_hand_task_ = std::make_shared<mc_tasks::EndEffectorTask>(
      "Rhand_Link0_Plan2", Eigen::Vector3d(0, 0, -0.14),
      ctl.robots(), 0, 5.0, 500.0);
  ctl.solver().addTask(right_hand_task_);
  right_hand_task_->reset();

  // initialize with the nominal CoM
  target_com_ = ctl.robot().com();
  current_com_ = target_com_;

  // initialize external wrenches
  ext_wrenches_.resize(2);
  ext_wrenches_[0].second = sva::ForceVecd::Zero();
  ext_wrenches_[1].second = sva::ForceVecd::Zero();

  // close gripper
  for (auto & g : ctl.robot().grippers()) {
    g.get().percentVMAX(0.6);
    g.get().setTargetOpening(0.0);
  }

  // setup logger and GUI
  setupLogger(ctl);
  setupGui(ctl);

  output("OK");
}

void states::RunStabilizer::runState()
{
  auto & ctl = controller();
  double dt = ctl.timeStep;

  // calculate the current value by interpolating the target value
  current_com_ +=
      (target_com_ - current_com_).cwiseMax(
          -1 * dt * com_vel_limit_).cwiseMin(
              dt * com_vel_limit_);

  current_left_hand_force_ +=
      (target_left_hand_force_ - current_left_hand_force_).cwiseMax(
          -1 * dt * hand_force_rate_limit_).cwiseMin(
              dt * hand_force_rate_limit_);
  current_right_hand_force_ +=
      (target_right_hand_force_ - current_right_hand_force_).cwiseMax(
          -1 * dt * hand_force_rate_limit_).cwiseMin(
              dt * hand_force_rate_limit_);

  // calculate the target CoM
  if (first_) {
    prev_com_ = current_com_;
    prev_com_vel_.setZero();
  }
  Eigen::Vector3d com_vel = (current_com_ - prev_com_) / dt;
  Eigen::Vector3d com_acc = (com_vel - prev_com_vel_) / dt;
  prev_com_ = current_com_;
  prev_com_vel_ = com_vel;

  // update the external wrenches
  ext_wrenches_[0].first = left_hand_task_->get_ef_pose().translation();
  ext_wrenches_[0].second.force() = current_left_hand_force_;
  ext_wrenches_[1].first = right_hand_task_->get_ef_pose().translation();
  ext_wrenches_[1].second.force() = current_right_hand_force_;
  stabilizer()->setExtWrenches(ext_wrenches_);

  // update the target of the end-effector tasks
  switch (reach_phase_) {
    case 1:
      left_hand_task_->set_ef_pose(
          sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.3, 0.5, 1.0}});
      right_hand_task_->set_ef_pose(
          sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.3, -0.5, 1.0}});
      reach_phase_ = 2;
      break;
    case 2:
      if (left_hand_task_->speed().norm() < 2e-3 && right_hand_task_->speed().norm() < 2e-3) {
        reach_phase_ = 3;
      }
      break;
    case 3:
      left_hand_task_->set_ef_pose(
          sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.6, 0.5, 1.0}});
      right_hand_task_->set_ef_pose(
          sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.6, -0.5, 1.0}});
      reach_phase_ = 0;
      break;
  }
}

void states::RunStabilizer::teardown()
{
  auto & ctl = controller();
  ctl.solver().removeTask(stabilizer());
}

bool states::RunStabilizer::checkTransitions()
{
  return false;
}

void states::RunStabilizer::setupLogger(mc_control::fsm::Controller & ctl)
{
  ctl.logger().addLogEntry("HandForceTest_TargetWrench_LeftHand",
                           [this]() -> const sva::ForceVecd
                           { return ext_wrenches_[0].second; });

  ctl.logger().addLogEntry("HandForceTest_TargetWrench_RightHand",
                           [this]() -> const sva::ForceVecd
                           { return ext_wrenches_[1].second; });

  ctl.logger().addLogEntry("HandForceTest_MeasuredWrench_LeftHand",
                           [&ctl]() -> const sva::ForceVecd
                           { return ctl.robot().forceSensor(
                               "LeftHandForceSensor").worldWrench(ctl.robot()); });

  ctl.logger().addLogEntry("HandForceTest_MeasuredWrench_RightHand",
                           [&ctl]() -> const sva::ForceVecd
                           { return ctl.robot().forceSensor(
                               "RightHandForceSensor").worldWrench(ctl.robot()); });
}

void states::RunStabilizer::setupGui(mc_control::fsm::Controller & ctl)
{
  // currently default reach_phase_ = 1, so reaching is automatically starts
  // ctl.gui()->addElement(
  //     {"HandForceTest"},
  //     mc_rtc::gui::Button(
  //         "ReachToWall", [this]() {
  //           reach_phase_ = 1;
  //           reach_start_time_ = ros::Time::now();
  //         }));

  ctl.gui()->addElement(
      {"HandForceTest", "Contact"},
      mc_rtc::gui::Button(
          "Add contacts of both hands", [this, &ctl]() {
            ctl.addContact(left_hand_contact_);
            ctl.addContact(right_hand_contact_);
          }));

  ctl.gui()->addElement(
      {"HandForceTest", "Contact"},
      mc_rtc::gui::Button(
          "Remove contacts of both hands", [this, &ctl]() {
            ctl.removeContact(left_hand_contact_);
            ctl.removeContact(right_hand_contact_);
          }));

  ctl.gui()->addElement(
      {"HandForceTest", "Force"},
      mc_rtc::gui::NumberInput(
          "Hand force rate limit",
          [this]() { return hand_force_rate_limit_; },
          [this](double v) {
            hand_force_rate_limit_ = v;
            mc_rtc::log::info("hand_force_rate_limit is changed to {}.", hand_force_rate_limit_);
          }));

  ctl.gui()->addElement(
      {"HandForceTest", "Force"},
      mc_rtc::gui::ArrayInput(
          "Both hands force",
          {"x", "y", "z"},
          [this]() -> const Eigen::Vector3d { return Eigen::Vector3d::Zero(); },
          [this](const Eigen::Vector3d& v) {
            target_left_hand_force_ = v;
            target_right_hand_force_ = v;
            mc_rtc::log::info("both_hands_force is changed to {}.",
                              target_left_hand_force_.transpose());
          }));

  ctl.gui()->addElement(
      {"HandForceTest", "Force"},
      mc_rtc::gui::ArrayInput(
          "Left hand force",
          {"x", "y", "z"},
          [this]() { return target_left_hand_force_; },
          [this](const Eigen::Vector3d& v) {
            target_left_hand_force_ = v;
            mc_rtc::log::info("left_hand_force is changed to {}.",
                              target_left_hand_force_.transpose());
          }));

  ctl.gui()->addElement(
      {"HandForceTest", "Force"},
      mc_rtc::gui::ArrayInput(
          "Right hand force",
          {"x", "y", "z"},
          [this]() { return target_right_hand_force_; },
          [this](const Eigen::Vector3d& v) {
            target_right_hand_force_ = v;
            mc_rtc::log::info("right_hand_force is changed to {}.",
                              target_right_hand_force_.transpose());
          }));

  ctl.gui()->addElement(
      {"HandForceTest", "CoM"},
      mc_rtc::gui::ArrayInput(
          "Nominal CoM",
          {"x", "y", "z"},
          [this]() { return target_com_; },
          [this](const Eigen::Vector3d& v) {
            target_com_ = v;
            mc_rtc::log::info("nominal_com is changed to {}.",
                              target_com_.transpose());
          }));
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("RunStabilizer", lipm_walking::states::RunStabilizer)
