// Copyright (c) 2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


/** @file robotiq_2f_gripper_joint_states.cpp
 *  Subscribes to Robotiq state messages on the input topic, converts the data to joint values,
 *  and publishes sensor_msgs/JointState messages on "joint_states" topic for Robotiq 2F gripper.
 * 
 *  'rosrun robotiq_2f_gripper_joint_state_publisher robotiq_2f_gripper_joint_states <gripper_prefix>'
 * 
 *  @author jack.thompson(at)utexas.edu
 *  @author karl.kruusamae(at)utexas.edu
 */

#include <vector>
#include <string>
#include <csignal>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>



std::vector<double> joint_positions;

double max_joint_rotation_radians;
int max_robotiq_joint_value;

/**
 * Callback function for "Robotiq3FGripperRobotInput" topic.
 */
void callback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr &msg) {
  int pos = msg->gPO;

  joint_positions.at(0)  = ( (double)(pos) / max_robotiq_joint_value) * max_joint_rotation_radians;
}

/**
 * Main method.
 */
int main(int argc, char *argv[]) {
  
  // ROS init, nodehandle, and rate
  ros::init(argc, argv, "robotiq_2f_gripper_joint_states");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate loop_rate(20);  // Hz

  // set user-specified prefix
  std::string gripper_prefix;
  pnh.param<std::string>("prefix", gripper_prefix, "");
  pnh.param<int>("max_robotiq_value", max_robotiq_joint_value, 229);
  pnh.param<double>("max_rotation_radians", max_joint_rotation_radians, 0.8);


  joint_positions.resize(1, 0.0);
  std::vector<std::string> joint_names(1, "");
  joint_names.at(0).assign(gripper_prefix + "finger_joint");


  // joint state publisher
  ros::Publisher joint_pub;
  joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

  // robotiq state message subscriber
  ros::Subscriber joint_sub;
  joint_sub = nh.subscribe("input", 10, &callback);
  
  // Output JointState message
  sensor_msgs::JointState joint_msg;
  
  // Joint names to JointState message
  joint_msg.name = joint_names;

  while (ros::ok()) {
    joint_msg.position = joint_positions;
    joint_msg.header.stamp = ros::Time::now();
    joint_pub.publish(joint_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
