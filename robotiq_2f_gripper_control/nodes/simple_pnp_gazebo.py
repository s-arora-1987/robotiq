#!/usr/bin/env python

import sys
import copy
import rospy
import rospkg
import os
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from time import sleep
from std_msgs.msg import String, Int8MultiArray, Int64, Int64MultiArray
from moveit_commander.conversions import pose_to_list
from gripper_to_position import gripper_to_pos, reset_gripper, activate_gripper
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

import intera_interface
from robotiq_2f_gripper_control.srv import move_robot,move_robotResponse,update_state,update_stateResponse
import numpy
from two_scara_collaboration.msg import cylinder_blocks_poses
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal: A list of floats, a Pose or a PoseStamped
  @param: actual: list of floats, a Pose or a PoseStamped
  @param: tolerance: A float
  """
  
  all_equal = True
  if type(goal) is list:
    for index in [0,1,2]:
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    # print "pose_to_list(goal):"+str(pose_to_list(goal))
    # print "pose_to_list(actual):"+str(pose_to_list(actual))
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

joint_state_topic = ['joint_states:=/robot/joint_states']

# a global class object
limb = 'right'
# intermediate variable needed for avoiding collisions
step_distance = 0.1 # meters
tip_name = "right_gripper_tip"
# at HOME position, orientation of gripper frame w.r.t world x=0.7, y=0.7, z=0.0, w=0.0 or [ rollx: -3.1415927, pitchy: 0, yawz: -1.5707963 ]
#(roll about an X-axis w.r.t home) / (subsequent pitch about the Y-axis) / (subsequent yaw about the Z-axis)
rollx = 3.30
pitchy = 0.0
yawz = -1.57
# use q with moveit because giving quaternion.x doesn't work.
q = quaternion_from_euler(rollx,pitchy,yawz)
overhead_orientation_moveit = Quaternion(
                         x=q[0],
                         y=q[1],
                         z=q[2],
                         w=q[3])
MOTION_SAMPLE_TIME=0.025

class PickAndPlace(object):
  def __init__(self,limb,step_distance,tip_name):
    super(PickAndPlace, self).__init__()

    moveit_commander.roscpp_initialize(joint_state_topic)

    rospy.init_node('simple_pnp_gazebo',
                    anonymous=True, disable_signals=False)

    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "right_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()

    eef_link = group.get_end_effector_link()

    group_names = robot.get_group_names()

    print robot.get_current_state()

    # now for intera
    print("Getting robot state... ")
    self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    self._init_state = self._rs.state().enabled
    print("Enabling robot... ")
    self._rs.enable()

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self._limb_name = limb # string
    self._limb = intera_interface.Limb(limb)
    self._step_distance = step_distance
    self._tip_name=tip_name

  def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles, timeout=2.0)
        return True

  def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            #move_to_joint_positions(self, positions, timeout=15.0,threshold=settings.JOINT_ANGLE_TOLERANCE,test=None)
            self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

  def _approach_with_leeway(self, pose):
      approach = copy.deepcopy(pose)
      # approach with a pose the hover-distance above the requested pose
      approach.position.z = approach.position.z + self._step_distance
      print "finding ik for "+str((approach.position.x,approach.position.y,approach.position.z))
      joint_angles = self._limb.ik_request(approach, self._tip_name)
      # print str(joint_angles)+" is output from limb.ik_request "
      self._limb.set_joint_position_speed(0.001) 
      self._guarded_move_to_joint_position(joint_angles,timeout=1.0) 
      self._limb.set_joint_position_speed(0.1)
      if joint_angles:
        return True
      else:
        return False

  def _approach(self, pose):
      approach = copy.deepcopy(pose)
      # approach with a pose the hover-distance above the requested pose
      joint_angles = self._limb.ik_request(approach, self._tip_name)
      self._limb.set_joint_position_speed(0.001) 
      self._guarded_move_to_joint_position(joint_angles,timeout=5.0) 
      self._limb.set_joint_position_speed(0.1)
      if joint_angles:
        return True
      else:
        return False

  def _servo_to_pose(self, current_pose, pose, time=4.0, steps=400.0):
      ''' An *incredibly simple* linearly-interpolated Cartesian move '''
      r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
      # current_pose = self._limb.endpoint_pose()
      print "current_pose: "+str((current_pose['position'].x,current_pose['position'].y,current_pose['position'].z))
      ik_delta = Pose()
      ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps
      ik_delta.position.y = (current_pose['position'].y - pose.position.y) / steps
      ik_delta.position.z = (current_pose['position'].z - pose.position.z) / steps
      ik_delta.orientation.x = (current_pose['orientation'].x - pose.orientation.x) / steps
      ik_delta.orientation.y = (current_pose['orientation'].y - pose.orientation.y) / steps
      ik_delta.orientation.z = (current_pose['orientation'].z - pose.orientation.z) / steps
      ik_delta.orientation.w = (current_pose['orientation'].w - pose.orientation.w) / steps
      for d in range(int(steps), -1, -1):
          if rospy.is_shutdown():
              return
          ik_step = Pose()
          ik_step.position.x = d*ik_delta.position.x + pose.position.x
          ik_step.position.y = d*ik_delta.position.y + pose.position.y
          ik_step.position.z = d*ik_delta.position.z + pose.position.z
          ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
          ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
          ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
          ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
          print "finding angles for "+str((ik_step.position.x,ik_step.position.y,ik_step.position.z))
          joint_angles = self._limb.ik_request(ik_step, self._tip_name)
          while joint_angles==False:
              r.sleep()
              r.sleep()
              joint_angles = self._limb.ik_request(ik_step, self._tip_name)
          self._limb.set_joint_positions(joint_angles)
          r.sleep()
          # print("These are the joint angles I got: ",joint_angles)
          # if joint_angles:
          #     self._limb.set_joint_positions(joint_angles)
          # else:
          #     rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
          r.sleep()
      rospy.sleep(1.0)
      return True

  def moveit_servo_to_pose(self, current_pose, pose, time=4.0, steps=400.0):
      ''' An *incredibly simple* linearly-interpolated Cartesian move '''
      r = rospy.Rate(1/(time/steps)) # Defaults to 100Hz command rate
      # current_pose = self.group.get_current_pose().pose
      ik_delta = Pose()
      ik_delta.position.x = (current_pose.position.x - pose.position.x) / steps
      ik_delta.position.y = (current_pose.position.y - pose.position.y) / steps
      ik_delta.position.z = (current_pose.position.x - pose.position.z) / steps
      ik_delta.orientation.x = (current_pose.orientation.x - pose.orientation.x) / steps
      ik_delta.orientation.y = (current_pose.orientation.y - pose.orientation.y) / steps
      ik_delta.orientation.z = (current_pose.orientation.z - pose.orientation.z) / steps
      ik_delta.orientation.w = (current_pose.orientation.w - pose.orientation.w) / steps
      for d in range(int(steps), -1, -1):
          if rospy.is_shutdown():
              return
          ik_step = Pose()
          ik_step.position.x = d*ik_delta.position.x + pose.position.x
          ik_step.position.y = d*ik_delta.position.y + pose.position.y
          ik_step.position.z = d*ik_delta.position.z + pose.position.z
          ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
          ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
          ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
          ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
          flag=False
          while not flag:
            self.go_to_pose_goal(q[0],q[1],q[2],q[3],
              ik_step.position.x,ik_step.position.y,ik_step.position.z)
            # self.go_to_pose_goal(ik_step.orientation.x,ik_step.orientation.y,
            #   ik_step.orientation.z,ik_step.orientation.w,
              # ik_step.position.x,ik_step.position.y,ik_step.position.z)
            # sleep(1.0)
            current_pose_step = self.group.get_current_pose().pose
            flag=(not all_close(ik_step, current_pose_step, 0.01))
            r.sleep()
          # print("These are the joint angles I got: ",joint_angles)
          # if joint_angles:
          #     self._limb.set_joint_positions(joint_angles)
          # else:
          #     rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
          # r.sleep()
      rospy.sleep(1.0)
      return True

  def go_to_joint_goal(self, angles, allow_replanning=True, planning_time=5.0,
    goal_tol=0.02,orientation_tol=0.02):

    group = self.group
    # Allow some leeway in position(meters) and orientation (radians)
    group.set_goal_position_tolerance(goal_tol)
    group.set_goal_orientation_tolerance(goal_tol) 
    group.allow_replanning(allow_replanning)
    group.set_planning_time(planning_time) 
    group.go(angles, wait=True)
    group.stop()
    return True

  def go_to_pose_goal(self, ox, oy, oz, ow, px, py, pz, allow_replanning=True, planning_time=5.0):
    """
    Movement method to go to desired end effector pose
    @param: ox: Pose orientation for the x-axis (part of Quaternion)
    @param: oy: Pose orientation for the y-axis (part of Quaternion)
    @param: oz: Pose orientation for the z-axis (part of Quaternion)
    @param: ow: Pose orientation for the w (part of Quaternion)
    @param: px: Coordinate on the x-axis 
    @param: py: Coordinate on the y-axis
    @param: pz: Coordinate on the z-axis
    """
    group = self.group
    # Allow some leeway in position(meters) and orientation (radians)
    group.set_goal_position_tolerance(0.02)
    group.set_goal_orientation_tolerance(0.02) 

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = ox
    pose_goal.orientation.y = oy
    pose_goal.orientation.z = oz
    pose_goal.orientation.w = ow
    pose_goal.position.x = px
    pose_goal.position.y = py
    pose_goal.position.z = pz
    group.set_pose_target(pose_goal)
    group.allow_replanning(allow_replanning)
    group.set_planning_time(planning_time) 
    # group.set_planning_time(0.5) 
    plan = group.go(wait=True)
    # rospy.sleep(1)
    group.stop()
   
    group.clear_pose_targets()
    
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.02)

  def display_trajectory(self, plan):
    """
    Display a movement plan / trajectory
    @param: plan: Plan to be displayed
    """
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
  
    display_trajectory_publisher.publish(display_trajectory);


############################## Need a global class object ###########################################
pnp = PickAndPlace(limb,step_distance,tip_name)

############################## YET TO CLEANUP ###########################################


def create_cube_request(modelname, px, py, pz, rr, rp, ry, sx, sy, sz):
    """
    Create a cube to be spawned in gazebo
    @param: px: Position coordinate on x-axis
    @param: py: Position coordinate on y-axis
    @param: pz: Position coordinate on z-axis
    @param: rr: Roll rotation
    @param: rp: Pitch rotation
    @param: ry: Yaw rotation
    @param: sx: Cube size on x-axis
    @param: sy: Cube size on y-axis
    @param: sz: Cube size on z-axis
    """

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('robotiq_2f_gripper_control')+"/models/"

    with open(model_path + 'cube.sdf', 'r') as file:
      sdf_cube = file.read().replace('\n', '')

    cube = deepcopy(sdf_cube)
    # Replace size of model
    size_str = str(round(sx, 3)) + " " + \
        str(round(sy, 3)) + " " + str(round(sz, 3))
    cube = cube.replace('SIZEXYZ', size_str)
    # Replace modelname
    cube = cube.replace('MODELNAME', str(modelname))

    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = cube
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


def delete_gazebo_models():
    del_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel) # Handle to model spawner
    rospy.wait_for_service('gazebo/delete_model')
    del_model("cube1") # Remove from Gazebo
    rospy.sleep(1)

##################################################################################################


target_location_x = -100
target_location_y = -100
target_location_z = -100
onion_index = -1
attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

req = AttachRequest()
robot_pose_x = -100
robot_pose_y = -100
robot_pose_z = -100
location_EE = -1
location_claimed_onion = -1
onion_attached = 0
hover_plane_height = 0.14 
picked = False 

def handle_update_state(req_update_state):
  global location_claimed_onion, location_EE, \
  ConveyorWidthRANGE_LOWER_LIMIT, ObjectPoseZ_RANGE_UPPER_LIMIT
  # read robot's pose and update a global variable
  global pnp, robot_pose_x, robot_pose_y, robot_pose_z, location_EE
  current_position = pnp.group.get_current_pose().pose.position
  robot_pose_x = current_position.x
  robot_pose_y = current_position.y
  robot_pose_z = current_position.z
  if robot_pose_x >= ConveyorWidthRANGE_LOWER_LIMIT and \
    robot_pose_z <= (ObjectPoseZ_RANGE_UPPER_LIMIT-0.78):# 0.06
    location_EE = 0 # on conveyor
  else:
    if (robot_pose_x < 0.5 and robot_pose_y < 0.2 and\
      robot_pose_z > -0.1 and robot_pose_z <= 0.18) or \
      (robot_pose_x >= (ConveyorWidthRANGE_LOWER_LIMIT-0.03) and \
      robot_pose_z > (ObjectPoseZ_RANGE_UPPER_LIMIT-0.78) and\
      robot_pose_z <= (hover_plane_height+0.05)):# y > 0.06 and y <= 0.14+0.05
      location_EE = 3 # at home
    else:
      if robot_pose_x > 0.5 and robot_pose_x < 0.69 and \
        robot_pose_y < 0.2 and\
        robot_pose_z >= 0.3 and robot_pose_z <= 0.5:
        location_EE = 1 # in front
      else:
        if robot_pose_x < 0.13 and \
          robot_pose_y > 0.57 and robot_pose_y < 0.62 and \
          robot_pose_z > -0.07 and robot_pose_z < 0.07:
          location_EE = 2 # at bin
        else:
          location_EE = -1 # either deleted or still changing

  loc = location_EE
  if loc == 0:
    str_loc = "Conv"
  else:
    if loc == 1:
      str_loc = "InFront"
    else:
      if loc == 2:
        str_loc = "AtBin"
      else:
        if loc == 3:
          str_loc = "Home"
        else:
          str_loc = "Unknown"

  # print "robot_pose_x:" + str(robot_pose_x)
  # print "robot_pose_y:" + str(robot_pose_y)
  # print "robot_pose_z:" + str(robot_pose_z)

  # print "location_EE:" + str(str_loc)
  array = [location_claimed_onion, location_EE]
  return update_stateResponse(array)

def handle_move_sawyer(req_move_robot):
  # "home","bin","hover","lowergripper","lookNrotate","roll"
  # index chosen to track
  global onion_index
  onion_index = req_move_robot.chosen_index
  argument = req_move_robot.primitive
  switcher = {
  "home": goto_home,
  "bin": goto_bin,
  "hover": hover,
  "lowergripper": lowergripper,
  "liftgripper": liftgripper,
  "lookNrotate": lookNrotate,
  "roll": roll,
  "attach": attach,
  "detach": detach,
  "conveyorCenter": goto_conveyorCenter,
  "detach_notgrasped": detach_notgrasped,
  "perturbStartBin": reach_waypoint_bw_bin_and_hover_region
  }
  # Get the function from switcher dictionary
  func = switcher.get(argument, lambda: "Invalid input to move_sawyer service")
  if argument not in switcher.keys():
    return move_robotResponse(False)
  # Execute the function
  result=func()
  return move_robotResponse(result) 

def goto_home(tolerance=0.3,goal_tol=0.1,orientation_tol=0.1):
  global pnp
  print "goto_home"
  home_joint_angles = [-0.041662954890248294,-1.0258291091425074, 0.0293680414401436,
  2.17518162913313,-0.06703022873354225,0.3968371433926965,1.7659649178699421]
  # pnp.go_to_joint_goal(home_joint_angles,True,5.0)
  # sleep(10.0)

  joint_angles = {'right_j0': -0.041662954890248294,
  'right_j1': -1.0258291091425074,
  'right_j2': 0.0293680414401436,
  'right_j3': 2.17518162913313,
  'right_j4': -0.06703022873354225,
  'right_j5': 0.3968371433926965,
  'right_j6': 1.7659649178699421} 
  current_joints = pnp.group.get_current_joint_values()
  tol=tolerance
  diff= abs(joint_angles['right_j0']-current_joints[0])>tol or \
  abs(joint_angles['right_j1']-current_joints[1])>tol or \
  abs(joint_angles['right_j2']-current_joints[2])>tol or \
  abs(joint_angles['right_j3']-current_joints[3])>tol or \
  abs(joint_angles['right_j4']-current_joints[4])>tol or \
  abs(joint_angles['right_j5']-current_joints[5])>tol or \
  abs(joint_angles['right_j6']-current_joints[6])>tol
  print "diff:"+str(diff)

  while diff:
    pnp.go_to_joint_goal(home_joint_angles,True,5.0,goal_tol=goal_tol,\
      orientation_tol=orientation_tol)
    sleep(3.0)
    #measure after movement
    current_joints = pnp.group.get_current_joint_values()
    
    diff= abs(joint_angles['right_j0']-current_joints[0])>tol or \
    abs(joint_angles['right_j1']-current_joints[1])>tol or \
    abs(joint_angles['right_j2']-current_joints[2])>tol or \
    abs(joint_angles['right_j3']-current_joints[3])>tol or \
    abs(joint_angles['right_j4']-current_joints[4])>tol or \
    abs(joint_angles['right_j5']-current_joints[5])>tol or \
    abs(joint_angles['right_j6']-current_joints[6])>tol
    if diff:
      pnp.move_to_start(joint_angles)
      sleep(3.0)

    print "diff:"+str(diff)

  print ("reached home")
  return True

def perturbStartBin():
  global q,pnp
  current_position = pnp.group.get_current_pose().pose.position
  
  allow_replanning = True
  planning_time = 0.5
  reached=pnp.go_to_pose_goal(q[0],q[1],q[2],q[3],current_position.x-0.04,\
    current_position.y,current_position.z,\
    allow_replanning,planning_time)
  rospy.sleep(0.5)

  allow_replanning = True
  planning_time = 0.5
  reached=pnp.go_to_pose_goal(q[0],q[1],q[2],q[3],current_position.x,\
    current_position.y,current_position.z-0.03,\
    allow_replanning,planning_time)
  rospy.sleep(0.5)

  return reached

displacement_for_picking = 0.0

def reach_waypoint_bw_bin_and_hover_region():
  global q,pnp,hover_plane_height,target_location_y,displacement_for_picking

  reached = False
  allow_replanning = True
  planning_time = 2.0
  # cuboid for bin
  #      if robot_pose_x < 0.13 and \
  #        robot_pose_y > 0.57 and robot_pose_y < 0.62 and \
  #        robot_pose_z > -0.07 and robot_pose_z < 0.07:
  # 
  location_x = 0.6
  location_y = target_location_y
  location_z = hover_plane_height+0.3
  while reached == False:
    reached=pnp.go_to_pose_goal(q[0],q[1],q[2],q[3],location_x,\
      target_location_y+displacement_for_picking+0.01,location_z,\
      allow_replanning,planning_time)
    rospy.sleep(3.0)

  return True

def hover(hovertime=0.3):

    global q,pnp,target_location_x,target_location_y,hover_plane_height,displacement_for_picking
    
    if target_location_x==-100 or target_location_y > (SAWYERRANGE_UPPER_LIMIT-gap_needed_to_pick):
      if target_location_x==-100:
        print "target_location_x==-100"
      else:
        print "target_location_y > (SAWYERRANGE_UPPER_LIMIT-gap_needed_to_pick)"
      # not enough time to pick
      return False

    # pnp._limb.set_joint_position_speed(40)
    print "hover to"+str((target_location_x,\
      target_location_y+displacement_for_picking+0.01,hover_plane_height))
    allow_replanning = True
    planning_time = 2.0
    reached=pnp.go_to_pose_goal(q[0],q[1],q[2],q[3],target_location_x,\
      target_location_y+displacement_for_picking+0.01,hover_plane_height,\
      allow_replanning,planning_time)
    rospy.sleep(1.0)
    print "hover reached "+str(reached)


    if reached == False: 
      return False

    allow_replanning=False
    planning_time=MOTION_SAMPLE_TIME
    t=0
    count = int(hovertime / MOTION_SAMPLE_TIME);
    for t in range(0,count):
      # joint_angles = pnp._limb.ik_request(Pose(Point(x=target_location_x,y=target_location_y,z=0.12),\
      #   Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])), pnp._tip_name)
      # pnp._guarded_move_to_joint_position(joint_angles,timeout=MOTION_SAMPLE_TIME) 
      reached=pnp.go_to_pose_goal(q[0],q[1],q[2],q[3],target_location_x,\
        target_location_y+displacement_for_picking,hover_plane_height,\
        allow_replanning,planning_time)
      rospy.sleep(2*MOTION_SAMPLE_TIME)

    print "hover reached "+str(reached)

    if reached == False: 
      return False

    return True

def lowergripper(displacement_for_picking=0.02):
    # approx centers of onions at 0.82, width of onion is 0.038 m. table is at 0.78  
    # length of gripper is 0.163 m The gripper should not go lower than 
    #(height_z of table w.r.t base+gripper-height/2+tolerance) = 0.78-0.93+0.08+0.01=-0.24
    # pnp._limb.endpoint_pose returns {'position': (x, y, z), 'orientation': (x, y, z, w)}
    # moving from z=-.02 to z=-0.1

    if target_location_y > (SAWYERRANGE_UPPER_LIMIT-gap_needed_to_pick):
      # object out of picking range
      return False

    global q,overhead_orientation,pnp,target_location_x,target_location_y,attach_srv
    z_array=[0.1,0.07,0.04,0.03]

    global pnp
    current_position = pnp.group.get_current_pose().pose.position
    lastpose_z = current_position.z

    allow_replanning=False
    planning_time=MOTION_SAMPLE_TIME
    for z in z_array:
      # calculating how much is base frame off
      reached=pnp.go_to_pose_goal(q[0],q[1],q[2],q[3],target_location_x-0.02,\
        target_location_y+displacement_for_picking,z,\
      allow_replanning,planning_time)
      # print "at z="+str(z)

    # Did it move at all? 
    current_position = pnp.group.get_current_pose().pose.position
    pose_z = current_position.z
    if abs(pose_z - lastpose_z) < 0.05:
      return False

    return True

def detach_notgrasped():
  global picked

  print "picked:"+str(picked)
  if picked == False:
    res = detach()
    print "res = detach():"+str(res)
    return res
  else:
    return False

def liftgripper(displacement_for_picking=0.015):
    # approx centers of onions at 0.82, width of onion is 0.038 m. table is at 0.78  
    # length of gripper is 0.163 m The gripper should not go lower than 
    #(height_z of table w.r.t base+gripper-height/2+tolerance) = 0.78-0.93+0.08+0.01=-0.24
    # pnp._limb.endpoint_pose returns {'position': (x, y, z), 'orientation': (x, y, z, w)}
    # moving from z=-.02 to z=-0.1

    global q,overhead_orientation,pnp,target_location_x,target_location_y
    z_array=[0.07,0.1,0.14,.16]

    global pnp
    current_position = pnp.group.get_current_pose().pose.position
    lastpose_z = current_position.z

    allow_replanning=False
    planning_time=MOTION_SAMPLE_TIME
    for z in z_array:
      reached=pnp.go_to_pose_goal(q[0],q[1],q[2],q[3],target_location_x-0.02,\
        target_location_y,z,\
      allow_replanning,planning_time)
      # print "at z="+str(z)

    # Did it move at all? 
    current_position = pnp.group.get_current_pose().pose.position
    pose_z = current_position.z
    if abs(pose_z - lastpose_z) < 0.05:
      return False

    return True

def lookNrotate():
  # go to inspection position and rotate

    goto_home(0.4,goal_tol=0.1,orientation_tol=0.1)
    # sleep(5.0)

    joint_angles = {}
    joint_goal=[-0.04, -1.015, 0.019, 2.15, -0.06, -2, 1.76]
    joint_angles['right_j0']=joint_goal[0]
    joint_angles['right_j1']=joint_goal[1]
    joint_angles['right_j2']=joint_goal[2]
    joint_angles['right_j3']=joint_goal[3]
    joint_angles['right_j4']=joint_goal[4]
    joint_angles['right_j5']=joint_goal[5]
    joint_angles['right_j6']=joint_goal[6]
    pnp._guarded_move_to_joint_position(joint_angles, timeout=3.0)
    # pnp.go_to_joint_goal(joint_goal,True,5.0,goal_tol=0.03,orientation_tol=0.03)
    # sleep(2.0)
    print ("object up")

    joint_goal = pnp.group.get_current_joint_values()
    joint_angles['right_j0']=joint_goal[0]
    joint_angles['right_j1']=joint_goal[1]
    joint_angles['right_j2']=joint_goal[2]
    joint_angles['right_j3']=joint_goal[3]
    joint_angles['right_j4']=joint_goal[4]
    joint_angles['right_j5']=joint_goal[5]
    joint_angles['right_j6']=joint_goal[6]
    current_j6=joint_angles['right_j6']

    if abs(current_j6)==current_j6:
      sign=-1
    else:
      sign=+1

    # 10 x 0.3 = 3.0 rad; rouhgly rotating 180 degrees
    for i in range(0,10):
      # joint_angles['right_j6']=joint_angles['right_j6']+sign*0.1
      # pnp._guarded_move_to_joint_position(joint_angles, timeout=MOTION_SAMPLE_TIME)
      joint_goal[6]=joint_goal[6]+sign*0.3
      pnp.go_to_joint_goal(joint_goal,False,MOTION_SAMPLE_TIME/2\
        ,goal_tol=0.02,orientation_tol=0.02)
      sleep(MOTION_SAMPLE_TIME)
    
    joint_goal[6]=current_j6
    pnp.go_to_joint_goal(joint_goal,True,1.0)

    # sleep(2.0)
    # goto_home(0.3)

    return True

def goto_bin():
    global q,pnp
    print "goto_bin"
    allow_replanning=True
    planning_time=5*MOTION_SAMPLE_TIME
    reached =False
    counter = 0

    while reached == False and counter <= 3:
      reached=pnp.go_to_pose_goal(q[0],q[1],q[2],q[3],0.1-0.02,0.6,0.03,\
      allow_replanning,planning_time)
      sleep(10*MOTION_SAMPLE_TIME)
      counter = counter+1

    return reached

def goto_conveyorCenter():

    global q,pnp

    loc_x = 0.75-0.02
    loc_y = 0.01
    allow_replanning=True
    planning_time=0.5
    reached=pnp.go_to_pose_goal(q[0],q[1],q[2],q[3],loc_x,loc_y,0.14,\
      allow_replanning,planning_time)
    rospy.sleep(0.2)
    allow_replanning=True
    planning_time=0.5
    reached=pnp.go_to_pose_goal(q[0],q[1],q[2],q[3],loc_x,loc_y,0.1,\
      allow_replanning,planning_time)
    rospy.sleep(0.2)

    return reached


def roll():
  # Height of rolling
  # sawyer base is 0.11 away from approx centers of onions at 0.82 
  # read from poses topic. max height of onion from its center is 0.076/2=0.038. 
  # then tops of onions are approx at 0.11-0.038 = 0.072 m from sawyer base. 
  # distance between edge and center of gripper is approx 0.075/2 = 0.037 m. 
  # so we lower gripper to z = -0.072+0.037 = -0.035 for rolling.
  # what about x and y locations of trajectory? length of gripper = 0.163 m, 
  # width of belt = 0.398 m. not enough space to make a circle. 
  # may be a straight line from starting object location to a fixed center
  # on right side boundary x=0.75,y=-0.31 of active region.

    global pnp,target_location_x,target_location_y,target_location_z
    print "roll()"

    belt_width = 0.3
    along_x_direction = 0
    along_y_direction = 1
    rotate_on_top = 0
    ## 1) move bottom to up along x direction
    if (along_x_direction == 1):
      target_location_loc_x = 0.75-belt_width/4 
    ## 2) move left to right along y direction 
    if (along_y_direction == 1):
      target_location_loc_x = 0.75-0.13 #+belt_width/4 #-belt_width+0.02
    ## 3)
    if (rotate_on_top == 1):
      target_location_loc_x = 0.75-0.02 #+belt_width/4 #-belt_width+0.02

    if (along_y_direction != 1):
      while target_location_y == -100:
        sleep(MOTION_SAMPLE_TIME)

    q_local=[-0.5875820126184493, 0.568398218589682, -0.39805895433046473, 0.416196963502457]
    allow_replanning=True
    planning_time=1.0
    # height_object_tip=-0.035
    height_object_tip=-0.0205

    print "reaching position to start rolling"
    print str(target_location_y+0.01)
    reached=False
    start_height=0.05
    start_y = 0.1
    while not reached:
      if (along_x_direction == 1):
        reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
          target_location_loc_x-0.02,target_location_y,\
          start_height,allow_replanning,planning_time)
        sleep(1.0)
      if (along_y_direction == 1):
        ## for moving objects
        # reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
          # target_location_loc_x-0.02,target_location_y+0.025,\
          # start_height, allow_replanning, planning_time)
        ## for stationary objects
        reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
          target_location_loc_x-0.02,start_y,\
          start_height,allow_replanning,planning_time)
        sleep(1.0)
      if (rotate_on_top == 1):
        reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
          target_location_loc_x-0.02,target_location_y,\
          start_height+0.05,allow_replanning,planning_time)
        sleep(1.0)

    allow_replanning=True
    planning_time=MOTION_SAMPLE_TIME
    stepcount = 30

    ## 1) move bottom to up along x direction
    if (along_x_direction == 1):    
      last_x=None
      step = (belt_width)/stepcount
      for i in range(1,stepcount):
        reached=False
        while not reached:
          reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
            target_location_loc_x-0.02+i*step,target_location_y,\
            height_object_tip,allow_replanning,planning_time)
          sleep(4*MOTION_SAMPLE_TIME)
          # reached = True

    ## 2) move left to right along y direction
    if (along_y_direction == 1):
      # last_y=None
      step = 0.7/(stepcount)
      for i in range(1,stepcount):
        reached=False
        while not reached:
          reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
            target_location_loc_x,start_y-i*step,\
            height_object_tip,allow_replanning,planning_time)
          sleep(2*MOTION_SAMPLE_TIME)
        while not reached:
          reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
            target_location_loc_x,start_y-i*step,\
            height_object_tip+0.01,allow_replanning,planning_time)
          sleep(2*MOTION_SAMPLE_TIME)
      last_y = start_y-i*step

    ## 3) go down on object
    if (rotate_on_top == 1):
      # last_y=None
      # step = (2*belt_width)/stepcount
      step = (start_height-height_object_tip)/stepcount
      for i in range(1,stepcount):
        reached=False
        while not reached:
          # reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
          #   target_location_loc_x-0.02+i*step,target_location_y,\
          #   height_object_tip,allow_replanning,planning_time)
          reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
            target_location_loc_x-0.02,target_location_y+0.01,\
            start_height-i*step,allow_replanning,planning_time)
          if reached== True:
            print "reached "+str(start_height-i*step)
          sleep(MOTION_SAMPLE_TIME)
          reached = True

      allow_replanning=True
      planning_time=2.0
      reached = False
      while reached:
        reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
          target_location_x-0.02,target_location_y+0.01,\
          height_object_tip+0.01,allow_replanning,planning_time)
        sleep(1.0)

      joint_goal = pnp.group.get_current_joint_values()
      joint_angles = {}
      joint_angles['right_j0']=joint_goal[0]
      joint_angles['right_j1']=joint_goal[1]
      joint_angles['right_j2']=joint_goal[2]
      joint_angles['right_j3']=joint_goal[3]
      joint_angles['right_j4']=joint_goal[4]
      joint_angles['right_j5']=joint_goal[5]
      joint_angles['right_j6']=joint_goal[6]
      current_j6=joint_angles['right_j6']

      if abs(current_j6)==current_j6:
        sign=-1
      else:
        sign=+1

      for i in range(0,5):
        # joint_angles['right_j6']=joint_angles['right_j6']+sign*0.1
        # pnp._guarded_move_to_joint_position(joint_angles, timeout=MOTION_SAMPLE_TIME)
        joint_goal[6]=joint_goal[6]+sign*0.6
        pnp.go_to_joint_goal(joint_goal,False,MOTION_SAMPLE_TIME/2\
          ,goal_tol=0.02,orientation_tol=0.02)
        sleep(MOTION_SAMPLE_TIME)
      
      joint_goal[6]=current_j6
      pnp.go_to_joint_goal(joint_goal,True,1.0)


    allow_replanning=True
    planning_time=2.0
    reached=pnp.go_to_pose_goal(q_local[0],q_local[1],q_local[2],q_local[3],\
      0.75-0.02,0.0,\
      hover_plane_height,allow_replanning,planning_time)
    sleep(2.0)

    # used to compute  desired q_local
    # goto_home(0.2)
    # joint_goal = pnp.group.get_current_joint_values()
    # joint_goal[5]=joint_goal[5]-1.4
    # joint_goal[6]=joint_goal[6]-1.57
    # pnp.go_to_joint_goal(joint_goal,False,0.5,\
    #   goal_tol=0.02,orientation_tol=0.02)
    # sleep(5.0)
    # current_pose = pnp.group.get_current_pose()
    # q_local=[0,0,0,0]
    # q_local[0] = current_pose.pose.orientation.x
    # q_local[1] = current_pose.pose.orientation.y
    # q_local[2] = current_pose.pose.orientation.z
    # q_local[3] = current_pose.pose.orientation.w
    # print "q_local:"
    # print q_local

    return False

def attach():
  global req, attach_srv,onion_index,onion_attached
  if target_location_y == -100:
    # object out of picking range
    return False
  try:
    # print "called attach"
    AttachResponse=attach_srv(req.model_name_1,req.link_name_1,req.model_name_2,req.link_name_2)
    sleep(0.5)
    if AttachResponse.ok == True:
      # publish index of attached model
      onion_attached = 1
      return True
    else:
      return False
  except rospy.ServiceException, e: 
    print "Service call failed: %s"%e 
    return False

def detach():
  global req, detach_srv, onion_attached
  if target_location_y == -100:
    # object out of picking range
    return False
  try:
    AttachResponse=detach_srv(req.model_name_1,req.link_name_1,req.model_name_2,req.link_name_2)
    sleep(0.5)
    if AttachResponse.ok == True:
      # publish index of attached model
      onion_attached = 0
      return True
    else:
      return False
  except rospy.ServiceException, e: 
    print "Service call failed: %s"%e 
    return False

def callback_poses(cylinder_poses_msg):

  current_cylinder_x = cylinder_poses_msg.x
  current_cylinder_y = cylinder_poses_msg.y
  current_cylinder_z = cylinder_poses_msg.z
  global target_location_x, target_location_y,target_location_z
  if onion_index < len(current_cylinder_x) and onion_index != -1:
    target_location_x = current_cylinder_x[onion_index]
    target_location_y = current_cylinder_y[onion_index]
    target_location_z = current_cylinder_z[onion_index]

  # read robot's pose and update a global variable
  global pnp, robot_pose_x, robot_pose_y, robot_pose_z, location_EE, picked
  current_position = pnp.group.get_current_pose().pose.position
  robot_pose_x = current_position.x
  robot_pose_y = current_position.y
  robot_pose_z = current_position.z
  if robot_pose_x >= ConveyorWidthRANGE_LOWER_LIMIT and \
    robot_pose_z <= hover_plane_height:
    location_EE = 0 # on conveyor
  else:
    if robot_pose_x < 0.5 and robot_pose_y < 0.2 and\
      robot_pose_z > -0.1 and robot_pose_z <= 0.18:
      location_EE = 3 # at home
    else:
      if robot_pose_x > 0.5 and robot_pose_x < 0.69 and \
        robot_pose_y < 0.2 and\
        robot_pose_z >= 0.3 and robot_pose_z <= 0.5:
        location_EE = 1 # in front
      else:
        if robot_pose_x < 0.11 and \
          robot_pose_y > 0.57 and robot_pose_y < 0.62 and \
          robot_pose_z > -0.05 and robot_pose_z < 0.05:
          location_EE = 2 # at bin
        else:
          location_EE = -1 # either deleted or still changing


  loc = location_EE
  if loc == 0:
    str_loc = "Conv"
  else:
    if loc == 1:
      str_loc = "InFront"
    else:
      if loc == 2:
        str_loc = "AtBin"
      else:
        if loc == 3:
          str_loc = "Home"
        else:
          str_loc = "Unknown"

  # print "location_EE:" + str(str_loc)
  last_picked = picked
  global ObjectPoseZ_RANGE_UPPER_LIMIT
  if abs(target_location_y - robot_pose_y) < 0.05 and \
  robot_pose_z > -0.05 and target_location_z > ObjectPoseZ_RANGE_UPPER_LIMIT:
    picked = True
  else:
    picked = False

  if picked != last_picked:
    print "picked: "+str(picked)

  # print "picked: "+str(picked)+",(abs(target_location_y - robot_pose_y) < 0.05):"\
  # +str((abs(target_location_y - robot_pose_y) < 0.05))+",(robot_pose_z> -0.05):"+\
  # str((robot_pose_z> -0.05))+",(target_location_z > ObjectPoseZ_RANGE_UPPER_LIMIT):"\
  # +str((target_location_z > ObjectPoseZ_RANGE_UPPER_LIMIT))

  return 

# print "target_location_x,target_location_y,target_location_z"+\
# str((target_location_x,target_location_y,target_location_z))
# global location_claimed_onion, hover_plane_height, \
# ConveyorWidthRANGE_LOWER_LIMIT,ObjectPoseZ_RANGE_UPPER_LIMIT
# if target_location_x >= ConveyorWidthRANGE_LOWER_LIMIT and \
#   target_location_z <= ObjectPoseZ_RANGE_UPPER_LIMIT+0.1:
#   location_claimed_onion = 0 # on conveyor
# else:
#   if target_location_x < 0.5 and target_location_y < 0.2 and\
#     target_location_z > 0.7 and target_location_z <= 0.99:
#     location_claimed_onion = 3 # at home
#   else:
#     if target_location_x > 0.5 and target_location_x < 0.69 and \
#       target_location_y < 0.2 and\
#       target_location_z >= 1.3 and target_location_z <= 1.55:
#       location_claimed_onion = 1 # in front
#     else:
#       if target_location_x < 0.11 and \
#         target_location_y > 0.57 and target_location_y < 0.62 and \
#         target_location_z > 0.77 and target_location_z < 0.82:
#         location_claimed_onion = 2 # at bin
#       else:
#         location_claimed_onion = -1 # either deleted or still changing

# loc = location_claimed_onion
# if loc == 0:
#   str_loc = "Conv"
# else:
#   if loc == 1:
#     str_loc = "InFront"
#   else:
#     if loc == 2:
#       str_loc = "AtBin"
#     else:
#       if loc == 3:
#         str_loc = "Home"
#       else:
#         str_loc = "Unknown"
# # print "location_claimed_onion:" + str(str_loc)

# at home?
# robot_pose_x:0.454616628708
# robot_pose_y:0.14990707423
# robot_pose_z:0.155911066303
# 0.4475151256001727
# 0.15926243800787734
# 0.9337766150197016
# at bin?
# robot_pose_x:0.0797689445644
# robot_pose_y:0.603479528927
# robot_pose_z:0.0284275474818
# target_location_x,target_location_y,target_location_z(0.09687150119130584, 0.5964004935909943, 0.8046527158738869)
# 0.10017616556865618
# 0.5969586705641233
# 0.8066861820137897
# in front?
# target_location_x,target_location_y,target_location_z(0.6401648310232856, 0.15462022214971552, 1.4419099346508792)
# robot_pose_x:0.547680406767
# robot_pose_y:0.15209515794
# robot_pose_z:0.393512184767
# at conveyor or at conveyorCenter?
# target_location_x,target_location_y,target_location_z(0.6482309294304253, 0.20426292541410887, 0.8127142542142765)
# within objectz_lowerlimit < z < hover plane upper limit

def callback_modelname(color_indices_msg):
  global req, onion_index
  if onion_index < len(color_indices_msg.data) and onion_index != -1:
    if (color_indices_msg.data[onion_index]==1):
      req.model_name_1 = "red_cylinder_" + str(onion_index);
    else:
      req.model_name_1 = "blue_cylinder_" + str(onion_index);    
  return


def main():
  global target_location_x,target_location_y,pnp,location_EE,\
  ConveyorWidthRANGE_LOWER_LIMIT, onion_attached, onion_index,\
  ObjectPoseZ_RANGE_UPPER_LIMIT, location_claimed_onion,\
  SAWYERRANGE_UPPER_LIMIT, gap_needed_to_pick
  
  ConveyorWidthRANGE_LOWER_LIMIT = rospy.get_param("/ConveyorWidthRANGE_LOWER_LIMIT")
  ObjectPoseZ_RANGE_UPPER_LIMIT = rospy.get_param("/ObjectPoseZ_RANGE_UPPER_LIMIT")
  SAWYERRANGE_UPPER_LIMIT = rospy.get_param("/SAWYERRANGE_UPPER_LIMIT")
  gap_needed_to_pick = rospy.get_param("/gap_needed_to_pick")

  reset_gripper()
  activate_gripper()
  gripper_to_pos(0, 60, 200, False) # OPEN GRIPPER  
  rospy.Subscriber("cylinder_blocks_poses", cylinder_blocks_poses, callback_poses)
  rospy.Subscriber("current_cylinder_blocks", Int8MultiArray, callback_modelname)
  # attach and detach service  
  attach_srv.wait_for_service() 
  detach_srv.wait_for_service()

  req.link_name_1 = "base_link"
  req.model_name_2 = "sawyer"
  req.link_name_2 = "right_l6"
  # pub_location_EE = rospy.Publisher('location_EE', Int64, queue_size=10)  
  # pub_location_Onion = rospy.Publisher('location_claimed_onion', Int64, queue_size=10)   
  srv_moverobot = rospy.Service('move_sawyer', move_robot, handle_move_sawyer)
  srv_updatestate = rospy.Service('update_state', update_state, handle_update_state)
  print "Ready to move sawyer."
  # rospy.spin()

  last_location_EE = -1
  last_location_claimed_onion = -1
  str_locEE = "Unknown"
  str_locO = "Unknown"

  # deciding orientation for roll action
  # global pnp
  # goto_home(0.3)
  # joint_goal = pnp.group.get_current_joint_values()
  # joint_goal[5]=joint_goal[5]-1.2
  # joint_goal[6]=joint_goal[6]-1.57
  # pnp.go_to_joint_goal(joint_goal,False,0.5,\
  #   goal_tol=0.02,orientation_tol=0.02)
  # sleep(5.0)
  # current_pose = pnp.group.get_current_pose()
  # q_local=[0,0,0,0]
  # q_local[0] = current_pose.pose.orientation.x
  # q_local[1] = current_pose.pose.orientation.y
  # q_local[2] = current_pose.pose.orientation.z
  # q_local[3] = current_pose.pose.orientation.w
  # print "q_local:"
  # print q_local

  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    global location_EE
    # print "onion_index,location_EE:"\
    #   +str((onion_index,location_EE))

    loc = location_EE
    if loc == 0:
      str_locEE = "Conv"
    else:
      if loc == 1:
        str_locEE = "InFront"
      else:
        if loc == 2:
          str_locEE = "AtBin"
        else:
          if loc == 3:
            str_locEE = "Home"
          else:
            str_locEE = "Unknown"

    if location_EE != last_location_EE:
      print "str_locEE:"+str_locEE
      # pub_location_EE.publish(location_EE)
      last_location_EE = location_EE

    rate.sleep()

  #### For refering to what param values worked first time #####

  # print "goto_home()"
  # goto_home(0.4,goal_tol=0.1,\
  #   orientation_tol=0.1)
  ## PICKING ##
  # hovertime=1.0
  # print "hover()"
  # success=hover(hovertime)
  # print "lowergripper()"
  # reset_gripper()
  # activate_gripper()
  # gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER
  # lowergripper()
  # sleep(5.0)
  # attach_srv.call(req)
  # sleep(2.0)
  # print "liftgripper()"
  # liftgripper()
  # sleep(2.0)
  ## INPECTION ##
  # print "lookNrotate()"
  # lookNrotate()
  ## PLACING ##
  # print "goto_bin()"
  # goto_bin()
  # sleep(5.0)
  # detach_srv.call(req)
  ## ROLLING ##
  # print "hover() on leftmost onion"
  # hover()
  # sleep(2.0)
  # print "roll()"
  # roll()
  # sleep(50.0)

  return

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass 
