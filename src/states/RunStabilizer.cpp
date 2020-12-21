/*
 * Copyright (c) 2020, CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <eigen_conversions/eigen_msg.h>
#include <hwm_msgs/CnoidExternalForceArray.h>

#include "RunStabilizer.h"

namespace lipm_walking
{

states::RunStabilizer::RunStabilizer():
    nh_(mc_rtc::ROSBridge::get_node_handle())
{
  ext_force_pub_ = nh_->advertise<hwm_msgs::CnoidExternalForceArray>(
      "cnoid_external_force", 1, true);
}

void states::RunStabilizer::start()
{
  auto & ctl = controller();

  // read configuration
  if (ctl.config().has("RunStabilizerConfig")) {
    const auto stabilizer_config = ctl.config()("RunStabilizerConfig");
    stabilizer_config("EnableAdmittance", enamble_admittance_);
    stabilizer_config("AdmittanceGain", admit_gain_);
    stabilizer_config("AdmittanceStiffness", admit_stiffness_);
    stabilizer_config("AdmittanceDamping", admit_damping_);
  }

  // add stabilizer task
  ctl.solver().addTask(stabilizer());
  stabilizer()->setApplyComOffset(true);

  // add admittance task
  left_admit_task_ = std::make_shared<mc_tasks::force::AdmittanceTask>(
      "LeftHand",
      ctl.robots(),
      ctl.robot().robotIndex());
  ctl.solver().addTask(left_admit_task_);
  left_admit_task_->reset();
  right_admit_task_ = std::make_shared<mc_tasks::force::AdmittanceTask>(
      "RightHand",
      ctl.robots(),
      ctl.robot().robotIndex());
  ctl.solver().addTask(right_admit_task_);
  right_admit_task_->reset();

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

  current_cnoid_ext_force_offset_ +=
      (target_cnoid_ext_force_offset_ - current_cnoid_ext_force_offset_).cwiseMax(
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

  // publish the external wrenches to the choreonoid
  hwm_msgs::CnoidExternalForceArray ext_force_arr_msg;
  std::vector<std::string> hand_link_names = {"LHDY", "RHDY"};
  std::vector<std::shared_ptr<mc_tasks::force::AdmittanceTask> > admit_tasks =
      {left_admit_task_, right_admit_task_};
  for (auto i : {0, 1}) {
    hwm_msgs::CnoidExternalForce ext_force_msg;
    ext_force_msg.tm = ros::Duration(100.0);
    ext_force_msg.robot = "HRP5P";
    ext_force_msg.link = hand_link_names[i];
    // ext_force_msg.link = ctl.robot().surface(admit_tasks[i]).bodyName();
    tf::pointEigenToMsg(Eigen::Vector3d::Zero(), ext_force_msg.position);
    tf::vectorEigenToMsg(
        ext_wrenches_[i].second.force() + current_cnoid_ext_force_offset_, ext_force_msg.force);
    ext_force_arr_msg.forces.push_back(ext_force_msg);
  }
  ext_force_pub_.publish(ext_force_arr_msg);

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
      if (left_admit_task_->eval().norm() < 1e-1 && right_admit_task_->eval().norm() < 1e-1) {
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
      if (enamble_admittance_) {
        left_admit_task_->admittance({Eigen::Vector3d::Zero(), admit_gain_});
        left_admit_task_->stiffness({{1., 1., 1.}, admit_stiffness_});
        left_admit_task_->damping({{1., 1., 1.}, admit_damping_});
        right_admit_task_->admittance({Eigen::Vector3d::Zero(), admit_gain_});
        right_admit_task_->stiffness({{1., 1., 1.}, admit_stiffness_});
        right_admit_task_->damping({{1., 1., 1.}, admit_damping_});
      }

      // save target pose relative to foot midpose
      {
        sva::PTransformd foot_midpose = footMidpose(ctl);
        rel_target_poses_ = {
          left_admit_task_->targetPose() * foot_midpose.inv(),
          right_admit_task_->targetPose() * foot_midpose.inv()};
      }

      reach_phase_ = 6;
      break;
    case 6:
      {
        sva::PTransformd foot_midpose = footMidpose(ctl);
        left_admit_task_->targetPose(rel_target_poses_[0] * foot_midpose);
        right_admit_task_->targetPose(rel_target_poses_[1] * foot_midpose);
      }
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
                               "LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()); });

  ctl.logger().addLogEntry("HandForceTest_MeasuredWrench_RightHand",
                           [&ctl]() -> const sva::ForceVecd
                           { return ctl.robot().forceSensor(
                               "RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()); });
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

  ctl.gui()->addElement(
      {"HandForceTest", "Force"},
      mc_rtc::gui::ArrayInput(
          "Cnoid external force offset",
          {"x", "y", "z"},
          [this]() -> const Eigen::Vector3d { return target_cnoid_ext_force_offset_; },
          [this](const Eigen::Vector3d& v) {
            target_cnoid_ext_force_offset_ = v;
            mc_rtc::log::info("cnoid_ext_force_offset is changed to {}.",
                              target_cnoid_ext_force_offset_.transpose());
          }));

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
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("RunStabilizer", lipm_walking::states::RunStabilizer)