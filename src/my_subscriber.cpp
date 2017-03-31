#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <sys/time.h>

#include <sstream>
#include <iomanip>
#include <string>
#include <memory>

//#include <NVX/nvx.h>
//#include <NVX/nvx_timer.hpp>

//#include <NVX/Application.hpp>
//#include <NVX/ConfigParser.hpp>
//#include <OVX/FrameSourceOVX.hpp>
//#include <OVX/RenderOVX.hpp>
//#include <NVX/SyncTimer.hpp>
//#include <OVX/UtilityOVX.hpp>
//#include <NVX/nvx_opencv_interop.hpp>

//#include "stereo_matching.hpp"
//#include "color_disparity_graph.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img1,img2;
  cv::Rect myROI_1(0,0,960,800); // constant 
  cv::Rect myROI_2(0,800,960,800); // constant 
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img1 = cv_ptr->image(myROI_1);
    img2 = cv_ptr->image(myROI_2);

    cv::imshow("view1", img1);
    cv::imshow("view2", img2);  

    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
