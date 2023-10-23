#!/usr/bin/env python3
"""
Moveit Client: The client that provides goal for the robotic arms in this case being uf850.
"""
import sys
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class ManipulatorMoveit(object):

  def __init__(self, manipulator_group_name):
    super(ManipulatorMoveit, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = manipulator_group_name
    group = moveit_commander.MoveGroupCommander(group_name)

    group.allow_replanning(True)
    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(1.0)

    group.set_planner_id("TRRT")
   

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()


    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

 
  
  def go_to_pose_goal(self, x =0.0, y =0.0, z =0.0, R=0.0 , P=180.0, Y=-180.0, frame='link_eef'):

    R, P, Y = R*((22/7)/180), P*((22/7)/180), Y*((22/7)/180)

    q1 = np.sin(R/2) * np.cos(P/2) * np.cos(Y/2) - np.cos(R/2) * np.sin(P/2) * np.sin(Y/2)
    q2 = np.cos(R/2) * np.sin(P/2) * np.cos(Y/2) + np.sin(R/2) * np.cos(P/2) * np.sin(Y/2)
    q3 = np.cos(R/2) * np.cos(P/2) * np.sin(Y/2) - np.sin(R/2) * np.sin(P/2) * np.cos(Y/2)
    q4 = np.cos(R/2) * np.cos(P/2) * np.cos(Y/2) + np.sin(R/2) * np.sin(P/2) * np.sin(Y/2)

    group = self.group
    group.clear_pose_targets()

    pose_goal = geometry_msgs.msg.Pose()
    
    pose_goal.orientation.x = q1
    pose_goal.orientation.y = q2
    pose_goal.orientation.z = q3
    pose_goal.orientation.w = q4

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    group.set_start_state(self.robot.get_current_state())
    
    group.set_pose_target(pose_goal, frame) # original
    # group.set_joint_value_target(pose_goal, frame, False) # approximate 
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.05)

  



def main():

    uf850 = ManipulatorMoveit('uf850')
  
    
    # Arm control
    for i in range(1):

      # Planner execution
       result = uf850.go_to_pose_goal(x =0.500, y =0.10,  z =0.20, R=0 , P=180, Y=0)
       result = uf850.go_to_pose_goal(x =0.500, y =0.10,  z =0.100, R=0 , P=180, Y=0)
       result = uf850.go_to_pose_goal(x =0.500, y =-0.40, z =0.100, R=0 , P=180, Y=0)
       result = uf850.go_to_pose_goal(x =0.500, y =-0.50, z =0.150, R=0 , P=180, Y=0)

     

if __name__ == '__main__':
  rospy.init_node('move_group_python_interface', anonymous=True)
  main()