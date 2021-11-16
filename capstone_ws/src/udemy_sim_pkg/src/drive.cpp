#include "drive.h"

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

double Drive::speedIncrement = 0.0;

Drive::Drive(ros::NodeHandle & node_handle): drive_node_handle(node_handle), translate_speed(0.0), turn_speed(0.0)
{
  left_wheel_speed_pub = drive_node_handle.advertise<std_msgs::Float64>("my_robot_model/left_wheel_hinge_joint_velocity_controller/command", 10);
  left_wheel_2_speed_pub = drive_node_handle.advertise<std_msgs::Float64>("my_robot_model/left_wheel_2_hinge_joint_velocity_controller/command", 10);
  right_wheel_speed_pub = drive_node_handle.advertise<std_msgs::Float64>("my_robot_model/right_wheel_hinge_joint_velocity_controller/command", 10);
  right_wheel_2_speed_pub = drive_node_handle.advertise<std_msgs::Float64>("my_robot_model/right_wheel_2_hinge_joint_velocity_controller/command", 10);

  printSpeeds();
}

void Drive::move(MoveDirection direction)
{
  switch(direction)
  {
    case kForward:
      setSpeed(translate_speed, translate_speed);
      break;
    case kBackward:
      setSpeed(-translate_speed, -translate_speed);
      break;
    case kLeft:
      setSpeed(-translate_speed, translate_speed);
      break;
    case kRight:
      setSpeed(translate_speed, -translate_speed);
      break;
    default: // should not get here
      break;
  }
}

void Drive::stop()
{
  setSpeed(0.0, 0.0);
}

void Drive::increaseSpeed()
{
  drive_node_handle.getParam("speed_increment", speedIncrement);
  translate_speed = translate_speed + speedIncrement;
  turn_speed = turn_speed + speedIncrement;
  printSpeeds();
}

void Drive::decreaseSpeed()
{
  drive_node_handle.getParam("speed_increment", speedIncrement);
  translate_speed = translate_speed - speedIncrement;
  turn_speed = turn_speed - speedIncrement;
  printSpeeds();
}

void Drive::setSpeed(double left_speed_set, double right_speed_set)
{
  std_msgs::Float64 left_msg, right_msg;
  left_msg.data = left_speed_set;
  right_msg.data = right_speed_set;

  left_wheel_speed_pub.publish(left_msg);
  left_wheel_2_speed_pub.publish(left_msg);
  right_wheel_speed_pub.publish(right_msg);
  right_wheel_2_speed_pub.publish(right_msg);
}

void Drive::printSpeeds()
{
  std::cout << "Current speeds in gazebo normalized units:" << std::endl;
  std::cout << std::setw(3) << "Translate Speed: " << translate_speed << ", Turn Speed: " << turn_speed << std::endl;
}
