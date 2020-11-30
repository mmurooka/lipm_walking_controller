/*
 * Copyright (c) 2020, CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "RunStabilizer.h"

namespace lipm_walking
{

void states::RunStabilizer::start()
{
  // add stabilizer task
  auto & ctl = controller();
  ctl.solver().addTask(stabilizer());
  stabilizer()->setApplyComOffset(true);

  // add admittance task
  left_admit_task_ = std::make_shared<mc_tasks::force::AdmittanceTask>(
      "LeftHand",
      ctl.robots(),
      ctl.robot().robotIndex());
  ctl.solver().addTask(left_admit_task_);
  left_admit_task_->reset();
  left_admit_task_->admittance(sva::ForceVecd::Zero());
  right_admit_task_ = std::make_shared<mc_tasks::force::AdmittanceTask>(
      "RightHand",
      ctl.robots(),
      ctl.robot().robotIndex());
  ctl.solver().addTask(right_admit_task_);
  right_admit_task_->reset();
  right_admit_task_->admittance(sva::ForceVecd::Zero());

  // initialize external wrenches
  ext_wrenches_.resize(2);
  ext_wrenches_[0].second = sva::ForceVecd::Zero();
  ext_wrenches_[1].second = sva::ForceVecd::Zero();

  // make contact instance
  Eigen::Vector6d hand_contact_dof = Eigen::Vector6d::Ones();
  hand_contact_dof[5] = 0;
  left_hand_contact_ = mc_control::fsm::Contact(
      "hrp5_p", "ground", "LeftHand", "AllGround",
      mc_rbdyn::Contact::defaultFriction,
      hand_contact_dof);
  right_hand_contact_ = mc_control::fsm::Contact(
      "hrp5_p", "ground", "RightHand", "AllGround",
      mc_rbdyn::Contact::defaultFriction,
      hand_contact_dof);

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
  current_left_hand_force_ +=
      (target_left_hand_force_ - current_left_hand_force_).cwiseMax(
          -1 * dt * hand_force_rate_limit_).cwiseMin(
              dt * hand_force_rate_limit_);
  current_right_hand_force_ +=
      (target_right_hand_force_ - current_right_hand_force_).cwiseMax(
          -1 * dt * hand_force_rate_limit_).cwiseMin(
              dt * hand_force_rate_limit_);

  // update the external wrenches
  ext_wrenches_[0].first = left_admit_task_->surfacePose().translation();
  ext_wrenches_[0].second.force() = current_left_hand_force_;
  ext_wrenches_[1].first = right_admit_task_->surfacePose().translation();
  ext_wrenches_[1].second.force() = current_right_hand_force_;
  stabilizer()->setExtWrenches(ext_wrenches_);
  left_admit_task_->targetWrenchW(ext_wrenches_[0].second);
  right_admit_task_->targetWrenchW(ext_wrenches_[1].second);

  // update the target pose of the admittance tasks
  switch (reach_phase_) {
    case 1:
      left_admit_task_->targetPose(
          sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.3, 0.5, 1.0}});
      right_admit_task_->targetPose(
          sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.3, -0.5, 1.0}});
      reach_phase_ = 2;
      break;
    case 2:
      if (left_admit_task_->speed().norm() < 1e-2 && right_admit_task_->speed().norm() < 1e-2) {
        reach_phase_ = 3;
      }
      break;
    case 3:
      left_admit_task_->targetPose(
          sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.6, 0.5, 1.0}});
      right_admit_task_->targetPose(
          sva::PTransformd{sva::RotY(-M_PI/2), Eigen::Vector3d{0.6, -0.5, 1.0}});
      reach_phase_ = 4;
      break;
    case 4:
      if (left_admit_task_->eval().norm() < 1e-2 && right_admit_task_->eval().norm() < 1e-2) {
        reach_phase_ = 5;
      }
      break;
    case 5:
      // enable admittance to the z direction
      sva::ForceVecd admit_gain(Eigen::Vector3d::Zero(), {0, 0, 0.01});
      sva::MotionVecd admit_stiffness({1., 1., 1.}, {1., 1., 1.});
      sva::MotionVecd admit_damping({1., 1., 1.}, {1., 1., 100.});
      left_admit_task_->admittance(admit_gain);
      left_admit_task_->stiffness(admit_stiffness);
      left_admit_task_->damping(admit_damping);
      right_admit_task_->admittance(admit_gain);
      right_admit_task_->stiffness(admit_stiffness);
      right_admit_task_->damping(admit_damping);

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
            mc_rtc::log::info("hand_force_rate_limit is changed to {}.",
                              hand_force_rate_limit_);
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
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("RunStabilizer", lipm_walking::states::RunStabilizer)
