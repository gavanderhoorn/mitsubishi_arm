#! /usr/bin/env python
"""Sends the robot to home(0,0,0,0,0,0).
WARNING: This might move the robot at dangerously high speeds. Use this script
with extreme care!!!"""

import rospy

# Brings in the SimpleActionClient
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
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



def JointTrajectoryActionClient():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/mitsubishi_arm/mitsubishi_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)

    # Waits until the action server has started up and started
    # listening for goals.

    print "waiting for server"
    client.wait_for_server()
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    #time_from_start=rospy.Duration(10)
    goal.trajectory.joint_names=['j1','j2','j3','j4','j5','j6']
    #To be reached 1 second after starting along the trajectory
    trajectory_point=trajectory_msgs.msg.JointTrajectoryPoint()
    trajectory_point.positions=[0]*6
    trajectory_point.time_from_start=rospy.Duration(0.1)
    goal.trajectory.points.append(trajectory_point)

    print "Sending goal"
    client.send_goal_and_wait(goal)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('mitsubishi_joint_trajectory_client')
        print "Init"
        result = JointTrajectoryActionClient()
        #print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
