#include <iostream>

#include "drive.h"

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_node");

  ros::NodeHandle node_handle;

  ros::Rate pub_rate(5); // 5 Hz

  Drive drive(node_handle);

  ROS_INFO("Drive Node Has Started");

  char desired_action; // direction or speed adjust
  char motion_action; // direction only
  char last_motion_action = 'x';

  while(ros::ok())
  {
    std::cout << "Enter a drive command (wasd direction, +- for speed adjust, x to stop, q to quit): ";
    std::cin >> desired_action;

    if(desired_action == '+')
    {
      drive.increaseSpeed();
      motion_action = last_motion_action;
    }
    else if(desired_action == '-')
    {
      drive.decreaseSpeed();
      motion_action = last_motion_action;
    }
    else
    {
      motion_action = desired_action;
    }

    switch(motion_action)
    {
      case 'w':
        drive.move(kForward);
        last_motion_action = motion_action;
        break;
      case 'a':
        drive.move(kLeft);
        last_motion_action = motion_action;
        break;
      case 's':
        drive.move(kBackward);
        last_motion_action = motion_action;
        break;
      case 'd':
        drive.move(kRight);
        last_motion_action = motion_action;
        break;
      case 'x':
        drive.stop();
        last_motion_action = motion_action;
        break;
      case 'q':
        ROS_INFO("Exiting Drive Node application");
        return 0;
      default: // ignore the input
        break;
    }

    std::cin.clear();

    pub_rate.sleep();
  }

  return 0;
}
