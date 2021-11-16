#include "ros/ros.h"
#include "control_msgs/JointControllerState.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64.h"
#include "udemy_sim_pkg/OperateCamera.h"

ros::Publisher camera_orientation_pub;
sensor_msgs::Image latest_camera_image;

void cameraImageCallback(const sensor_msgs::ImageConstPtr & msg)
{
  latest_camera_image = *msg;
}

bool getImage(udemy_sim_pkg::OperateCamera::Request & req,
              udemy_sim_pkg::OperateCamera::Response & res)
{
  std_msgs::Float64 msg;
  msg.data = req.angle;
  camera_orientation_pub.publish(msg);

  ros::Rate rate(1); // hz
  int wait_counter = 0; // 1 count/sec
  double wait_timeout;
  ros::NodeHandle get_image_node_handle;
  get_image_node_handle.getParam("camera_pid_timeout", wait_timeout);
  while(wait_counter < wait_timeout)
  {
    wait_counter++;
    rate.sleep();
  }

  boost::shared_ptr<sensor_msgs::Image const> image_shared_pointer;
  image_shared_pointer = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_raw", ros::Duration(10));
  if(image_shared_pointer)
  {
    res.camera_image = *image_shared_pointer;
  }
  else
  {
    ROS_INFO("Did not receive a camera image in operate_camera_server_node");
    return false;
  }

  return true;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "operate_camera_server_node");

  ros::NodeHandle node_handle;

  ros::ServiceServer service_server = node_handle.advertiseService("operate_camera", getImage);

  camera_orientation_pub = node_handle.advertise<std_msgs::Float64>("/my_robot_model/camera_shaft_joint_position_controller/command", 10);

  ros::Subscriber camera_image_sub = node_handle.subscribe("/camera/rgb/image_raw", 10, cameraImageCallback);

  ROS_INFO("Operate Camera Server Node Has Started");

  ros::spin();
}
