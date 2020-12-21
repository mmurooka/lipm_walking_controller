/*
 * Copyright (c) 2020, CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <cmath>
#include <unordered_map>

#include <lipm_walking/Controller.h>
#include <lipm_walking/State.h>

#include <mc_rtc/ros.h>
#include <mc_tasks/ImpedanceTask.h>
#include <ros/ros.h>

namespace lipm_walking
{

namespace states
{

/** \brief Adds/removes the StabilizerTask and ImpedanceTask to the solver. Set external wrenches. */
struct RunStabilizer : State
{
 public:
  /** \brief Left/right arm */
  enum class Arm
  {
    Left,
    Right
  };
  // both arms
  const std::vector<Arm> BOTH_ARMS = {Arm::Left, Arm::Right};

  /** \brief Phase */
  enum class Phase
  {
    StandBy,
    ReachWayPointInit,
    ReachWayPointWait,
    ReachTargetInit,
    ReachTargetWait,
    Grasp,
    Hold,
    Ungrasp,
    ReleaseWayPointInit,
    ReleaseWayPointWait,
    NominalPosture,
    End
  };
  std::string toString(Phase phase) const
  {
    switch (phase) {
      case Phase::StandBy:
        return "Phase::StandBy";
      case Phase::ReachWayPointInit:
        return "Phase::ReachWayPointInit";
      case Phase::ReachWayPointWait:
        return "Phase::ReachWayPointWait";
      case Phase::ReachTargetInit:
        return "Phase::ReachTargetInit";
      case Phase::ReachTargetWait:
        return "Phase::ReachTargetWait";
      case Phase::Grasp:
        return "Phase::Grasp";
      case Phase::Hold:
        return "Phase::Hold";
      case Phase::Ungrasp:
        return "Phase::Ungrasp";
      case Phase::ReleaseWayPointInit:
        return "Phase::ReleaseWayPointInit";
      case Phase::ReleaseWayPointWait:
        return "Phase::ReleaseWayPointWait";
      case Phase::NominalPosture:
        return "Phase::NominalPosture";
      case Phase::End:
        return "Phase::End";
    }
    return "Phase::Unknown";
  }

  RunStabilizer();

  void start() override;
  void teardown() override;
  void runState() override;
  bool checkTransitions() override;

 protected:
  void setupLogger(mc_control::fsm::Controller & ctl);

  void setupGui(mc_control::fsm::Controller & ctl);

  inline Phase nextPhase(Phase phase) const
  {
    if (phase == Phase::End) {
      return Phase::End;
    }
    return static_cast<Phase>(static_cast<int>(phase) + 1);
  }

  inline void goToNextPhase()
  {
    phase_ = nextPhase(phase_);
    mc_rtc::log::info("[RunStabilizer] go to {}.", toString(phase_));
  }

  inline void goToNextPhaseWithCheck()
  {
    if (auto_mode_ || go_next_) {
      goToNextPhase();
      go_next_ = false;
    }
  }

  inline bool handReached(double thre = 1e-2)
  {
    return (imp_tasks_.at(Arm::Left)->eval().norm() < thre) &&
        (imp_tasks_.at(Arm::Right)->eval().norm() < thre);
  }

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

 protected:
  Phase phase_ = Phase::StandBy;
  bool go_next_ = false;

  // surface name of hands
  std::unordered_map<Arm, std::string> surface_names_ = {
    {Arm::Left, "LeftHand"}, {Arm::Right, "RightHand"}};

  // impedance task of hands
  std::unordered_map<Arm, std::shared_ptr<mc_tasks::force::ImpedanceTask> > imp_tasks_;

  // target hand wrenches
  std::unordered_map<Arm, sva::ForceVecd> target_hand_wrenches_ = {
    {Arm::Left, sva::ForceVecd::Zero()}, {Arm::Right, sva::ForceVecd::Zero()}};

  // target position of hands
  std::unordered_map<Arm, Eigen::Vector3d> target_hand_poss_;

  // target pose of hands relative to foot middle pose
  std::unordered_map<Arm, sva::PTransformd> rel_target_hand_poses_;

  // hand contacts
  std::unordered_map<Arm, mc_control::fsm::Contact> hand_contacts_;

  // goal hand forces
  std::unordered_map<Arm, Eigen::Vector3d> goal_hand_forces_ = {
    {Arm::Left, Eigen::Vector3d::Zero()}, {Arm::Right, Eigen::Vector3d::Zero()}};
  // interpolated hand forces
  std::unordered_map<Arm, Eigen::Vector3d> interp_hand_forces_ = {
    {Arm::Left, Eigen::Vector3d::Zero()}, {Arm::Right, Eigen::Vector3d::Zero()}};

  // goal cnoid external force offset
  Eigen::Vector3d goal_cnoid_ext_force_offset_ = Eigen::Vector3d::Zero();
  // interpolated cnoid external force offset
  Eigen::Vector3d interp_cnoid_ext_force_offset_ = Eigen::Vector3d::Zero();

  // Configuration
  bool auto_mode_ = false;
  bool grasp_ = true;
  bool enable_impedance_ = true;
  double hand_force_rate_limit_ = 20;
  double approach_dist_ = 0.2;
  bool publish_cnoid_ = false;

  // ROS variables
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher cnoid_ext_force_pub_;
  ros::Publisher cnoid_wall_force_pub_;
};

} // namespace states

} // namespace lipm_walking
