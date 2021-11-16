#include "ros/ros.h"
#include "udemy_sim_pkg/OperateCamera.h"

#include <iostream>
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "operate_camera_client_node");

  ros::NodeHandle node_handle;

  ros::ServiceClient service_client = node_handle.serviceClient<udemy_sim_pkg::OperateCamera>("operate_camera");
  udemy_sim_pkg::OperateCamera operate_camera_srv;

  ROS_INFO("Operate Camera Client Node Has Started");

  double input_angle;

  while(ros::ok())
  {
    std::cout << "Enter an angle in degrees (<-360 or >360 to quit): ";
    std::cin >> input_angle;

    if((input_angle < -360) || (input_angle > 360))
    {
      ROS_INFO("Exiting Operate Camera Application");
      return 0;
    }
    else
    {
      operate_camera_srv.request.angle = input_angle*(M_PI/180);
    }

    if(service_client.call(operate_camera_srv))
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(operate_camera_srv.response.camera_image, sensor_msgs::image_encodings::BGR8);
      cv::imshow("What The Robot Sees", cv_ptr->image);
      cv::waitKey(0);
      cv::destroyAllWindows();
    }
    else
    {
      ROS_ERROR("Operate Camera Service Call Failed");
      return 1;
    }

    std::cin.clear();
  }
}
