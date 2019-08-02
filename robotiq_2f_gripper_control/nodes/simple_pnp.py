#!/usr/bin/env python

import sys
import copy
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from time import sleep
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gripper_to_position import gripper_to_pos
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
    SpawnModelRequest,
    SpawnModelResponse
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from copy import deepcopy
from tf.transformations import quaternion_from_euler

## END_SUB_TUTORIAL

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

joint_state_topic = ['joint_states:=/robot/joint_states']

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('simple_pnp',
                    anonymous=True, disable_signals=False)
    moveit_commander.roscpp_initialize(sys.argv)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "right_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_pose_goal(self, ox, oy, oz, ow, px, py, pz):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = ox
    pose_goal.orientation.y = oy
    pose_goal.orientation.z = oz
    pose_goal.orientation.w = ow
    pose_goal.position.x = px
    pose_goal.position.y = py
    pose_goal.position.z = pz
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
   
    group.clear_pose_targets()

    
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

   

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

   
    group.execute(plan, wait=True)

def main():
  try:
    #moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('simple_pnp',
                    anonymous=True, disable_signals=False)
    #moveit_commander.roscpp_initialize(sys.argv)
    #tester = MoveGroupPythonIntefaceTutorial()
    pose_list = []
    # 255 = closed, 0 = open
    gripper_to_pos(255, 60, 200, False)    # ACTIVATION STEP
    gripper_to_pos(255, 60, 200, False)
    """tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,
                             0.74, 0.02, 0.25)

    rospy.sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,
                             0.74, 0.02, 0.1)

    gripper_to_pos(100, 60, 200, False)

    rospy.sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,
                             0.74, 0.02, 0.25)
    
    rospy.sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,
                             0.9, 0.02, 0.25)

    rospy.sleep(1.0)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,
                             0.9, 0.02, 0.1)

    gripper_to_pos(0, 60, 200, False)

    tester.go_to_pose_goal(1.0, 0.0, 0.0, 0.0000463,
                             0.74, 0.02, 0.25)"""
  except rospy.ROSInterruptException:
    delete_gazebo_models()
    return
  except KeyboardInterrupt:
    delete_gazebo_models()
    return

if __name__ == '__main__':
  main()
