#! /usr/bin/env python
"""Test fk and ik using kdl library"""

import rospy
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

# Brings in the SimpleActionClient
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import numpy as np


# goal message 

'''
    trajectory_msgs/JointTrajectory trajectory
     std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
     string[] joint_names
     trajectory_msgs/JointTrajectoryPoint[] points
      float64[] positions
      float64[] velocities
      float64[] accelerations
      float64[] effort
      duration time_from_start
     control_msgs/JointTolerance[] path_tolerance
      string name
      float64 position
      float64 velocity
      float64 acceleration
     control_msgs/JointTolerance[] goal_tolerance
      string name
      float64 position
      float64 velocity
      float64 acceleration
     duration goal_time_tolerance
'''



class KinematicsTest(object):

    """Test kinematics using KDL"""

    def __init__(self):
        """Read robot describtion"""
        self.robot_ = URDF.from_parameter_server() 
        self.kdl_kin_ = KDLKinematics(self.robot_, 'base_link', 'end_effector')
        self.cur_pose_ = None
        self.cur_jnt_ = None
        # Creates the SimpleActionClient, passing the type of the action
        self.client_ = actionlib.SimpleActionClient('/mitsubishi_arm/mitsubishi_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        self.client_.wait_for_server()
        self.goal_ = control_msgs.msg.FollowJointTrajectoryGoal()
        #time_from_start=rospy.Duration(10)
        self.goal_.trajectory.joint_names=['j1','j2','j3','j4','j5','j6']
        #To be reached 1 second after starting along the trajectory
        trajectory_point=trajectory_msgs.msg.JointTrajectoryPoint()
        trajectory_point.positions=[0]*6
        trajectory_point.time_from_start=rospy.Duration(0.1)
        self.goal_.trajectory.points.append(trajectory_point)

        rospy.Subscriber('joint_states', sensor_msgs.msg.JointState, self.fwd_kinematics_cb)
        rospy.Subscriber('command', geometry_msgs.msg.Pose, self.inv_kinematics_cb)

    
    def fwd_kinematics_cb(self, data):
        """Compute forward kinematics and print it out

        :data: TODO
        :returns: TODO

        """
        self.cur_jnt_ = data.position
        self.cur_pose_ = self.kdl_kin_.forward(data.position)
        q = np.array(data.position) 
        new_pose = self.cur_pose_.copy()
        new_pose[0,3] += 0.01
        #print "cp:", np.ravel(self.cur_pose_[:3,3])
        #print "np:", np.ravel(new_pose[:3,3])
        #q_ik = self.kdl_kin_.inverse(new_pose, q)
        #print "Q:", q
        #print "Q_ik:", q_ik

    def inv_kinematics_cb(self, data):
        """calculate the ik given a pose (only xyz right  now)

        :data: TODO
        :returns: TODO

        """
        # preserve the rotation
        new_pose = self.cur_pose_.copy()
        new_pose[0,3] = data.position.x
        new_pose[1,3] = data.position.y
        new_pose[2,3] = data.position.z
        q_ik = self.kdl_kin_.inverse(new_pose, self.cur_jnt_)

         
        print "cur:", np.ravel(self.cur_pose_[:3,3])
        print "req:", np.ravel(new_pose[:3,3])
        print "Q:", self.cur_jnt_
        print "Q_ik:", q_ik
        if q_ik is not None:
            print "Sending goal"
            self.goal_.trajectory.points[0].positions = q_ik
            self.client_.send_goal(self.goal_)

    
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('mitsubishi_kinematics_test')
        print "Init"
        kt = KinematicsTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
