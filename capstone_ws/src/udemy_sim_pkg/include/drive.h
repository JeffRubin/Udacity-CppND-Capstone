#ifndef DRIVE
#define DRIVE

#include "ros/ros.h"

enum MoveDirection {kForward, kBackward, kLeft, kRight};

class Drive
{
  static double speedIncrement;

public:
  Drive(ros::NodeHandle & node_handle); // constructor
  void move(MoveDirection direction);
  void stop();
  void increaseSpeed();
  void decreaseSpeed();

private:
  void setSpeed(double left_speed_set, double right_speed_set);
  void printSpeeds();

  ros::NodeHandle & drive_node_handle;

  double translate_speed; // non-turning (i.e. translation) speed
  double turn_speed;

  ros::Publisher left_wheel_speed_pub;
  ros::Publisher left_wheel_2_speed_pub;
  ros::Publisher right_wheel_speed_pub;
  ros::Publisher right_wheel_2_speed_pub;
};

#endif
