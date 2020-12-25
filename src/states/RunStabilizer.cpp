/*
 * Copyright (c) 2020, CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <eigen_conversions/eigen_msg.h>
#include <hwm_msgs/CnoidExternalForceArray.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "RunStabilizer.h"

namespace lipm_walking
{

states::RunStabilizer::RunStabilizer():
    nh_(mc_rtc::ROSBridge::get_node_handle())
{
  // make contact instance
  {
    Eigen::Vector6d hand_contact_dof = Eigen::Vector6d::Ones();
    hand_contact_dof[5] = 0;
    hand_contacts_.clear();
    for (auto arm : BOTH_ARMS) {
      hand_contacts_.emplace(
          arm,
          mc_control::fsm::Contact(
              "hrp5_p", "ground", surface_names_.at(arm), "AllGround",
              mc_rbdyn::Contact::defaultFriction,
              hand_contact_dof));
    }
  }

  // setup ROS
  cnoid_ext_force_pub_ = nh_->advertise<hwm_msgs::CnoidExternalForceArray>(
      "cnoid/external_force", 1, true);
  cnoid_wall_force_pub_ = nh_->advertise<geometry_msgs::Vector3Stamped>(
      "cnoid/wall_force_command", 1, true);
}

void states::RunStabilizer::start()
{
  auto & ctl = controller();

  // read configuration
  if (ctl.config().has("RunStabilizerConfig")) {
    const auto config = ctl.config()("RunStabilizerConfig");
    config("AutoMode", auto_mode_);
    config("HandForceRateLimit", hand_force_rate_limit_);
    config("ApproachDist", approach_dist_);
    config("PublishCnoid", publish_cnoid_);
  }

  // setup target hand poses
  {
    std::string reach_type = "wall";
    sva::PTransformd obj_pose = sva::PTransformd::Identity();
    sva::PTransformd obj_to_hand_pose_left = sva::PTransformd::Identity();
    sva::PTransformd obj_to_hand_pose_right = sva::PTransformd::Identity();
    double rolling_radius = 1.0;
    double rolling_angle = 0.0;
    if (ctl.config().has("RunStabilizerConfig")) {
      const auto config = ctl.config()("RunStabilizerConfig");
      if (config.has("Obj")) {
        const auto obj_config = config("Obj");
        obj_config("ReachType", reach_type);
        obj_config("GraspObj", grasp_obj_);
        loadPTransformd(obj_config, "ObjPose", obj_pose);
        loadPTransformd(obj_config, "ObjToHandPoseLeft", obj_to_hand_pose_left);
        loadPTransformd(obj_config, "ObjToHandPoseRight", obj_to_hand_pose_right);
        obj_config("RollingRadius", rolling_radius);
        obj_config("RollingAngle", rolling_angle);
      }
    }
    sva::PTransformd hand_midpose = obj_pose;
    if (reach_type == "wall") {
    } else if (reach_type == "bobbin") {
      hand_midpose =
          sva::PTransformd((-rolling_radius * Eigen::Vector3d::UnitX()).eval()) *
          sva::PTransformd(sva::RotY(rolling_angle)) * obj_pose;
    } else {
      mc_rtc::log::error_and_throw<std::runtime_error>("[RunStabilizer] Unknown reach_type: {}", reach_type);
    }
    target_hand_poses_ = {
      {Arm::Left, obj_to_hand_pose_left * hand_midpose},
      {Arm::Right, obj_to_hand_pose_right * hand_midpose}};
  }

  // setup logger and GUI
  setupLogger(ctl);
  setupGui(ctl);

  // setup StabilizerTask
  {
    ctl.solver().addTask(stabilizer());

    Eigen::Vector3d ext_wrench_gain = Eigen::Vector3d::Ones();
    if (ctl.config().has("RunStabilizerConfig")) {
      const auto config = ctl.config()("RunStabilizerConfig");
      if (config.has("Obj")) {
        const auto obj_config = config("Obj");
        obj_config("extWrenchGain", ext_wrench_gain);
      }
    }
    // use only the specified direction component of measured wrench to make it robust against contact errors
    stabilizer()->extWrenchGain(sva::MotionVecd(Eigen::Vector3d::Zero(), ext_wrench_gain));
  }

  // setup ImpedanceTask
  {
    Eigen::Vector3d impMPos = Eigen::Vector3d::Constant(10.0);
    Eigen::Vector3d impDPos = Eigen::Vector3d::Constant(1500.0);
    Eigen::Vector3d impKPos = Eigen::Vector3d::Constant(500.0);
    if (ctl.config().has("RunStabilizerConfig")) {
      const auto config = ctl.config()("RunStabilizerConfig");
      if (config.has("Obj")) {
        const auto obj_config = config("Obj");
        obj_config("impMPos", impMPos);
        obj_config("impDPos", impDPos);
        obj_config("impKPos", impKPos);
      }
    }
    sva::ForceVecd impM(Eigen::Vector3d::Constant(2.0), impMPos);
    sva::ForceVecd impD(Eigen::Vector3d::Constant(200.0), impDPos);
    sva::ForceVecd impK(Eigen::Vector3d::Constant(200.0), impKPos);

    imp_tasks_.clear();
    for (auto arm : BOTH_ARMS) {
      auto imp_task = std::make_shared<mc_tasks::force::ImpedanceTask>(
          surface_names_.at(arm),
          ctl.robots(),
          ctl.robot().robotIndex());
      imp_tasks_.emplace(arm, imp_task);
      imp_task->impedance(impM, impD, impK);
      imp_task->reset();
      ctl.solver().addTask(imp_task);
      // temporarily, make the wrench gain of orientation zero
      imp_task->wrenchGain(sva::MotionVecd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones()));
    }
  }

  for (auto & g : ctl.robot().grippers()) {
    // update gripper parameter
    g.get().percentVMAX(0.6);
    g.get().releaseSafetyOffset(mc_rtc::constants::toRad(0));
    g.get().actualCommandDiffTrigger(mc_rtc::constants::toRad(3));
    // open or close gripper
    if (grasp_obj_) {
      g.get().setTargetOpening(1.0);
    } else {
      g.get().setTargetOpening(0.0);
    }
  }

  mc_rtc::log::success("[RunStabilizer] started RunStabilizer state.");
}

void states::RunStabilizer::runState()
{
  auto & ctl = controller();
  double dt = ctl.timeStep;

  // interpolate the goal value
  for (auto arm : BOTH_ARMS) {
    interp_hand_forces_.at(arm) +=
        (goal_hand_forces_.at(arm) - interp_hand_forces_.at(arm)).cwiseMax(
            -1 * dt * hand_force_rate_limit_).cwiseMin(
                dt * hand_force_rate_limit_);
  }

  interp_cnoid_ext_force_offset_ +=
      (goal_cnoid_ext_force_offset_ - interp_cnoid_ext_force_offset_).cwiseMax(
          -1 * dt * hand_force_rate_limit_).cwiseMin(
              dt * hand_force_rate_limit_);

  // update the external wrenches of StabilizerTask and ImpedanceTask
  std::vector<std::pair<std::string, sva::ForceVecd> > ext_wrenches;
  for (auto arm : BOTH_ARMS) {
    target_hand_wrenches_.at(arm).force() = interp_hand_forces_.at(arm);
    imp_tasks_.at(arm)->targetWrench(target_hand_wrenches_.at(arm));
    ext_wrenches.emplace_back(surface_names_.at(arm), target_hand_wrenches_.at(arm));
  }
  stabilizer()->setExtWrenches(ext_wrenches);

  // update the target pose of the impedance tasks
  switch (phase_) {
    case Phase::StandBy:
      // requests go_next_ even in auto_mode_ at the start
      if (gripperCompleted(ctl) && go_next_) {
        goToNextPhaseWithCheck();
      }
      break;
    case Phase::ReachWayPoint:
      if (phase_switched_) {
        updateGuiStartReach(ctl);
        for (auto arm : BOTH_ARMS) {
          imp_tasks_.at(arm)->desiredPose(
              sva::PTransformd((approach_dist_ * Eigen::Vector3d::UnitZ()).eval()) * target_hand_poses_.at(arm));
        }
        phase_switched_ = false;
      } else if (handReached(1e-1, 1e-2)) {
        goToNextPhaseWithCheck();
      }
      break;
    case Phase::ReachTarget:
      if (phase_switched_) {
        for (auto arm : BOTH_ARMS) {
          imp_tasks_.at(arm)->desiredPose(target_hand_poses_.at(arm));
        }
        phase_switched_ = false;
      } else if (handReached()) {
        goToNextPhaseWithCheck();
      }
      break;
    case Phase::GraspObj:
      if (phase_switched_) {
        if (grasp_obj_) {
          for (auto & g : ctl.robot().grippers()) {
            g.get().setTargetOpening(0.0);
          }
        }
        phase_switched_ = false;
      } else if (gripperCompleted(ctl)) {
        goToNextPhase();
      }
      break;
    case Phase::Hold:
      if (phase_switched_) {
        // store hand pose relative to foot midpose
        rel_target_hand_poses_.clear();
        for (auto arm : BOTH_ARMS) {
          rel_target_hand_poses_.emplace(
              arm,
              imp_tasks_.at(arm)->desiredPose() * footMidpose(ctl).inv());
        }
        updateGuiStartHold(ctl);
        phase_switched_ = false;
      } else {
        for (auto arm : BOTH_ARMS) {
          imp_tasks_.at(arm)->desiredPose(rel_target_hand_poses_.at(arm) * footMidpose(ctl));
        }
        if (finish_locomanip_) {
          updateGuiFinishHold(ctl);
          goToNextPhaseWithCheck();
        }
      }
      break;
    case Phase::UngraspObj:
      if (phase_switched_) {
        if (grasp_obj_) {
          for (auto & g : ctl.robot().grippers()) {
            g.get().setTargetOpening(1.0);
          }
        }
        phase_switched_ = false;
      } else if (gripperCompleted(ctl)) {
        goToNextPhaseWithCheck();
      }
      break;
    case Phase::ReleaseWayPoint:
      if (phase_switched_) {
        for (auto arm : BOTH_ARMS) {
          imp_tasks_.at(arm)->desiredPose(
              sva::PTransformd((approach_dist_ * Eigen::Vector3d::UnitZ()).eval()) * imp_tasks_.at(arm)->desiredPose());
        }
        phase_switched_ = false;
      } else if (handReached(1e-1, 1e-2)) {
        goToNextPhaseWithCheck();
      }
      break;
    case Phase::NominalPosture:
      for (auto arm : BOTH_ARMS) {
        imp_tasks_.at(arm)->weight(0);
      }

      // close gripper to finalize
      for (auto & g : ctl.robot().grippers()) {
        g.get().setTargetOpening(0.0);
      }

      goToNextPhase();
      break;
    case Phase::End:
      break;
  }

  // publish the external wrenches to the choreonoid
  if (publish_cnoid_ ) {
    hwm_msgs::CnoidExternalForceArray ext_force_arr_msg;
    geometry_msgs::Vector3Stamped wall_force_command_msg;
    Eigen::Vector3d wall_force_command = Eigen::Vector3d::Zero();
    for (auto arm : BOTH_ARMS) {
      // find parent-side of the hand surface because the joint need to be specified in choreonoid
      std::string joint_name = "";
      {
        const auto & mb = ctl.robot().mb();
        const auto & body_name = ctl.robot().surface(surface_names_.at(arm)).bodyName();
        int body_idx = mb.bodyIndexByName(body_name);
        for (int joint_idx = 0; joint_idx < mb.nrJoints(); joint_idx++) {
          if (body_idx == mb.successor(joint_idx)) {
            joint_name = mb.joint(joint_idx).name();
            break;
          }
        }
        if (joint_name.empty()) {
          mc_rtc::log::error_and_throw<std::runtime_error>("[RunStabilizer] Parent-side joint of body {} not found.", body_name);
        }
      }

      hwm_msgs::CnoidExternalForce ext_force_msg;
      ext_force_msg.tm = ros::Duration(100.0);
      ext_force_msg.robot = "HRP5P";
      ext_force_msg.link = joint_name;
      tf::pointEigenToMsg(Eigen::Vector3d::Zero(), ext_force_msg.position);
      sva::PTransformd surfacePose = ctl.realRobot().surfacePose(surface_names_.at(arm));
      sva::PTransformd T_s_0(Eigen::Matrix3d(surfacePose.rotation().transpose()));
      Eigen::Vector3d forceW = T_s_0.dualMul(target_hand_wrenches_.at(arm)).force();
      tf::vectorEigenToMsg(forceW + interp_cnoid_ext_force_offset_, ext_force_msg.force);
      ext_force_arr_msg.forces.push_back(ext_force_msg);
      wall_force_command += forceW + interp_cnoid_ext_force_offset_;
    }
    tf::vectorEigenToMsg(wall_force_command, wall_force_command_msg.vector);
    cnoid_ext_force_pub_.publish(ext_force_arr_msg);
    cnoid_wall_force_pub_.publish(wall_force_command_msg);
  }
}

void states::RunStabilizer::teardown()
{
  auto & ctl = controller();
  ctl.solver().removeTask(stabilizer());

  mc_rtc::log::success("[RunStabilizer] teared down RunStabilizer state.");
}

bool states::RunStabilizer::checkTransitions()
{
  // this state does not break
  return false;
}

void states::RunStabilizer::setupLogger(mc_control::fsm::Controller & ctl)
{
  ctl.logger().addLogEntry("Locomanip_phase",
                           [this]() { return static_cast<int>(phase_); });
  ctl.logger().addLogEntry("Locomanip_TargetWrench_LeftHand",
                           [this]() -> const sva::ForceVecd
                           { return target_hand_wrenches_.at(Arm::Left); });

  ctl.logger().addLogEntry("Locomanip_TargetWrench_RightHand",
                           [this]() -> const sva::ForceVecd
                           { return target_hand_wrenches_.at(Arm::Right); });

  ctl.logger().addLogEntry("Locomanip_MeasuredWrench_LeftHand",
                           [&ctl]() -> const sva::ForceVecd
                           { return ctl.robot().surfaceWrench("LeftHand"); });

  ctl.logger().addLogEntry("Locomanip_MeasuredWrench_RightHand",
                           [&ctl]() -> const sva::ForceVecd
                           { return ctl.robot().surfaceWrench("RightHand"); });
}

void states::RunStabilizer::setupGui(mc_control::fsm::Controller & ctl)
{
  std::string button_name = "Go next";
  if (auto_mode_) {
    button_name = "Start auto";
  }
  ctl.gui()->addElement(
      {"Locomanip"}, mc_rtc::gui::ElementsStacking::Horizontal,
      mc_rtc::gui::Label("Current", [this]() { return toString(phase_); }),
      mc_rtc::gui::Label("Next", [this]() { return toString(nextPhase(phase_)); }),
      mc_rtc::gui::Button(button_name, [this]() { go_next_ = true; }));

  ctl.gui()->addElement(
      {"Locomanip", "ExtWrench"},
      mc_rtc::gui::NumberInput(
          "Hand force rate limit",
          [this]() { return hand_force_rate_limit_; },
          [this](double v) {
            hand_force_rate_limit_ = v;
            mc_rtc::log::info("[RunStabilizer] hand_force_rate_limit is changed to {}.",
                              hand_force_rate_limit_);
          }),
      mc_rtc::gui::ArrayInput(
          "Both hands target force",
          {"x", "y", "z"},
          [this]() -> const Eigen::Vector3d { return interp_hand_forces_.at(Arm::Left); },
          [this](const Eigen::Vector3d& v) {
            goal_hand_forces_.at(Arm::Left) = v;
            goal_hand_forces_.at(Arm::Right) = v;
            mc_rtc::log::info("[RunStabilizer] both_hands_force is changed to {}.",
                              goal_hand_forces_.at(Arm::Left).transpose());
          }),
      mc_rtc::gui::ArrayInput(
          "Left hand target force",
          {"x", "y", "z"},
          [this]() { return interp_hand_forces_.at(Arm::Left); },
          [this](const Eigen::Vector3d& v) {
            goal_hand_forces_.at(Arm::Left) = v;
            mc_rtc::log::info("[RunStabilizer] left_hand_force is changed to {}.",
                              goal_hand_forces_.at(Arm::Left).transpose());
          }),
      mc_rtc::gui::ArrayInput(
          "Right hand target force",
          {"x", "y", "z"},
          [this]() { return interp_hand_forces_.at(Arm::Right); },
          [this](const Eigen::Vector3d& v) {
            goal_hand_forces_.at(Arm::Right) = v;
            mc_rtc::log::info("[RunStabilizer] right_hand_force is changed to {}.",
                              goal_hand_forces_.at(Arm::Right).transpose());
          }));

  if (publish_cnoid_) {
    ctl.gui()->addElement(
        {"Locomanip", "ExtWrench"},
        mc_rtc::gui::ArrayInput(
            "Cnoid external force offset",
            {"x", "y", "z"},
            [this]() -> const Eigen::Vector3d { return interp_cnoid_ext_force_offset_; },
            [this](const Eigen::Vector3d& v) {
              goal_cnoid_ext_force_offset_ = v;
              mc_rtc::log::info("[RunStabilizer] cnoid_ext_force_offset is changed to {}.",
                                goal_cnoid_ext_force_offset_.transpose());
            }));
  }

  ctl.gui()->addElement(
      {"Locomanip", "Impedance"},
      mc_rtc::gui::ArrayLabel(
          "Left hand filtered force",
          {"x", "y", "z"},
          [this]() { return imp_tasks_.at(Arm::Left)->filteredMeasuredWrench().force(); }),
      mc_rtc::gui::ArrayLabel(
          "Right hand filtered force",
          {"x", "y", "z"},
          [this]() { return imp_tasks_.at(Arm::Right)->filteredMeasuredWrench().force(); }),
      mc_rtc::gui::ArrayInput(
          "impedancePosition",
          {"M", "D", "K"},
          [this]() {
            return Eigen::Vector3d(
                imp_tasks_.at(Arm::Left)->impedanceM().force()[0],
                imp_tasks_.at(Arm::Left)->impedanceD().force()[0],
                imp_tasks_.at(Arm::Left)->impedanceK().force()[0]);
          },
          [this](const Eigen::Vector3d& v) {
            for (auto arm : BOTH_ARMS) {
              imp_tasks_.at(arm)->impedancePosition(
                  Eigen::Vector3d::Constant(v[0]),
                  Eigen::Vector3d::Constant(v[1]),
                  Eigen::Vector3d::Constant(v[2]));
            }
            mc_rtc::log::info("[RunStabilizer] impedance is changed to {}.",
                              v.transpose());
          }),
      mc_rtc::gui::ArrayInput(
          "impedancePosition Z-axis",
          {"M", "D", "K"},
          [this]() {
            return Eigen::Vector3d(
                imp_tasks_.at(Arm::Left)->impedanceM().force()[2],
                imp_tasks_.at(Arm::Left)->impedanceD().force()[2],
                imp_tasks_.at(Arm::Left)->impedanceK().force()[2]);
          },
          [this](const Eigen::Vector3d& v) {
            Eigen::Vector3d impM = imp_tasks_.at(Arm::Left)->impedanceM().force();
            Eigen::Vector3d impD = imp_tasks_.at(Arm::Left)->impedanceD().force();
            Eigen::Vector3d impK = imp_tasks_.at(Arm::Left)->impedanceK().force();
            impM[2] = v[0];
            impD[2] = v[1];
            impK[2] = v[2];
            for (auto arm : BOTH_ARMS) {
              imp_tasks_.at(arm)->impedancePosition(impM, impD, impK);
            }
            mc_rtc::log::info("[RunStabilizer] Z-Axis impedance is changed to {}.",
                              v.transpose());
          }));

  ctl.gui()->addElement(
      {"Locomanip", "Utility"}, mc_rtc::gui::ElementsStacking::Horizontal,
      mc_rtc::gui::Button(
          "Open gripper", [this, &ctl]() {
            for (auto & g : ctl.robot().grippers()) {
              g.get().setTargetOpening(1.0);
            }
          }),
      mc_rtc::gui::Button(
          "Close gripper", [this, &ctl]() {
            for (auto & g : ctl.robot().grippers()) {
              g.get().setTargetOpening(0.0);
            }
          }));
  ctl.gui()->addElement(
      {"Locomanip", "Utility"}, mc_rtc::gui::ElementsStacking::Horizontal,
      mc_rtc::gui::Button(
          "Add contacts of both hands", [this, &ctl]() {
            for (const auto c : hand_contacts_) {
              ctl.addContact(c.second);
            }
          }),
      mc_rtc::gui::Button(
          "Remove contacts of both hands", [this, &ctl]() {
            for (const auto c : hand_contacts_) {
              ctl.removeContact(c.second);
            }
          }));

  ctl.gui()->addElement(
      {"Locomanip", "Configuration"},
      mc_rtc::gui::Label("AutoMode", [this]() { return auto_mode_; }),
      mc_rtc::gui::Label("GraspObj", [this]() { return grasp_obj_; }),
      mc_rtc::gui::Label("PublishCnoid", [this]() { return publish_cnoid_; }));
}

void states::RunStabilizer::updateGuiStartReach(mc_control::fsm::Controller & ctl)
{
  if (std::find(once_flags_.begin(), once_flags_.end(), "updateGuiStartReach") != once_flags_.end()) {
    return;
  }

  if (auto_mode_) {
    ctl.gui()->removeElement({"Locomanip"}, "Start auto");
  }
  once_flags_.push_back("updateGuiStartReach");
}

void states::RunStabilizer::updateGuiStartHold(mc_control::fsm::Controller & ctl)
{
  if (std::find(once_flags_.begin(), once_flags_.end(), "updateGuiStartHold") != once_flags_.end()) {
    return;
  }

  ctl.gui()->addElement(
      {"Locomanip"},
      mc_rtc::gui::Button(
          "Finish locomanip", [this]() { finish_locomanip_ = true; }));
  once_flags_.push_back("updateGuiStartHold");
}

void states::RunStabilizer::updateGuiFinishHold(mc_control::fsm::Controller & ctl)
{
  if (std::find(once_flags_.begin(), once_flags_.end(), "updateGuiFinishHold") != once_flags_.end()) {
    return;
  }

  ctl.gui()->removeElement({"Locomanip"}, "Finish locomanip");
  once_flags_.push_back("updateGuiFinishHold");
}

} // namespace lipm_walking

EXPORT_SINGLE_STATE("RunStabilizer", lipm_walking::states::RunStabilizer)
