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

def main():
  try:
    rospy.init_node("gripper_external")

    # 255 = closed, 0 = open
    gripper_to_pos(0, False)

    gripper_to_pos(255, False)

  except rospy.ROSInterruptException:
    delete_gazebo_models()
    return
  except KeyboardInterrupt:
    delete_gazebo_models()
    return

if __name__ == '__main__':
  main()
