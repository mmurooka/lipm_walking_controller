#! /usr/bin/python

import numpy as np
import sva
import eigen as e


class CalcFootstep(object):
    def __init__(self):
        self.both_foot = ["left", "right"]
        self.surface_names = {"left": "LeftFootCenter", "right": "RightFootCenter"}
        self.foot_poses = {"left": None, "right": None}
        self.nominal_foot_dist = 0.21
        self.turn_radius = 1.2
        self.template_txt = """\
        - pose: {}
          surface: {}
        - pose: {}
          surface: {}"""

    def calcFootPose(self, lr, center_pose):
        sgn = 1 if lr == "left" else -1
        local_foot_pose = sva.PTransformd(e.Vector3d(0, 0.5 * sgn * self.nominal_foot_dist, 0))
        foot_pose = local_foot_pose * center_pose
        quat = e.Quaterniond(foot_pose.rotation())
        pos = foot_pose.translation()
        return [quat.w(), quat.x(), quat.y(), quat.z(), pos.x(), pos.y(), pos.z()]

    def printFootstepSequenceTurn(self, goal_angle, angle_step=np.deg2rad(7.5)):
        angle = 0.
        center_pose = sva.PTransformd.Identity()
        while True:
            center_pose = \
                sva.PTransformd(-self.turn_radius * e.Vector3d.UnitX()) * sva.PTransformd(sva.RotZ(angle)) * \
                sva.PTransformd(self.turn_radius * e.Vector3d.UnitX())
            for foot in self.both_foot:
                self.foot_poses[foot] = self.calcFootPose(foot, center_pose)
            if goal_angle < 0:
                print self.template_txt.format(
                    self.foot_poses["left"], self.surface_names["left"], self.foot_poses["right"], self.surface_names["right"])
            else:
                print self.template_txt.format(
                    self.foot_poses["right"], self.surface_names["right"], self.foot_poses["left"], self.surface_names["left"])
            if np.abs(angle) >= np.abs(goal_angle):
                break
            angle = np.sign(goal_angle) * np.min([np.abs(goal_angle), np.abs(angle) + angle_step])

if __name__ == "__main__":
    cf = CalcFootstep()

    ## bobbin_turn_left_1
    # cf.printFootstepSequenceTurn(np.deg2rad(-7.5))
    ## bobbin_turn_left_2
    cf.printFootstepSequenceTurn(np.deg2rad(-15))
    ## bobbin_turn_left_4
    # cf.printFootstepSequenceTurn(np.deg2rad(-30))

    ## bobbin_turn_right_1
    # cf.printFootstepSequenceTurn(np.deg2rad(7.5))
    ## bobbin_turn_right_2
    # cf.printFootstepSequenceTurn(np.deg2rad(15))
    ## bobbin_turn_right_4
    # cf.printFootstepSequenceTurn(np.deg2rad(30))
