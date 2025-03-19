#! /usr/bin/env python3

import numpy as np
import os
import rospkg
import rospy

from timeit import default_timer as timer
from geometry_msgs.msg import Pose
from relaxed_ik_ros1.msg import EEPoseGoals

from scipy.spatial.transform import Rotation as R


class Franka3DVConverter:
    def __init__(self):
        subscriber = rospy.Subscriber(
            "/franka_3dv/move_to_pose", Pose, self.pose_callback
        )
        self.publisher = rospy.Publisher(
            "/relaxed_ik/ee_pose_goals", EEPoseGoals, queue_size=5
        )

    def change_frame(self, pose):
        # add 10.5 cm to the z axis while considering the orientation

        # convert the orientation to a rotation matrix
        orientation = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )

        rotation = R.from_quat(orientation).as_matrix()
        offset = np.array([0, 0, -0.107])
        offset = np.dot(rotation, offset)

        fix_mat = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

        # orientation = R.from_matrix(rotation.T).as_quat()
        # orientation = R.from_matrix(rotation).as_quat()

        pose.position.x += offset[0]
        pose.position.y += offset[1]
        pose.position.z += offset[2]

        # pose.orientation.x = orientation[0]
        # pose.orientation.y = orientation[1]
        # pose.orientation.z = orientation[2]
        # pose.orientation.w = orientation[3]

        return pose

    def pose_callback(self, pose):
        pose_goals = EEPoseGoals()
        pose_goals.header.stamp = rospy.Time.now()
        pose_goal = Pose()

        pose = self.change_frame(pose)

        pose_goal.position = pose.position
        pose_goal.orientation = pose.orientation
        pose_goals.ee_poses.append(pose_goal)
        self.publisher.publish(pose_goals)


if __name__ == "__main__":
    rospy.init_node("Franka3DVConverter")
    franka_3dv_converter = Franka3DVConverter()
    rospy.spin()
