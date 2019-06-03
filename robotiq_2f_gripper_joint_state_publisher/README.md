# robotiq_2f_gripper_joint_state_publisher

### For Robotiq 2F gripper

Subscribes to Robotiq state messages on "Robotiq2FGripperRobotInput" topic, converts the data to joint values, and publishes sensor_msgs/JointState messages on "joint_states" topic for Robotiq 2f-gripper.
Based upon the Robotiq 3f gripper joint state publisher package

#### Instructions
Run:

`rosrun robotiq_2f_gripper_joint_state_publisher robotiq_2f_gripper_joint_states`

or

`rosrun robotiq_2f_gripper_joint_state_publisher robotiq_2f_gripper_joint_states _prefix:=<gripper_prefix>`

