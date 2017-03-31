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

  cv::Vec3d T;
  cv::Vec4d D1,D2;
  cv::Mat R1, R2, P1, P2, Q, K1, K2, R;
  cv::FileStorage fs("/home/nvidia/catkin_ws/src/ros_tegra_stereo/params/cam_stereo.yml",cv::FileStorage::READ);
  if (!fs.isOpened()) {
      std::cerr << "Failed to open calibration parameter file." << std::endl;
  	return 0;
  }
  fs["K1"] >> K1;
  fs["K2"] >> K2;
  fs["R"] >> R;
  fs["R1"] >> R1;
  fs["R2"] >> R2;
  fs["P1"] >> P1;
  fs["P2"] >> P2;
  fs["Q"] >> Q;
  fs["T"] >> T;
  fs["D1"] >> D1;
  fs["D2"] >> D2;
  double* data_k1c = reinterpret_cast<double*>(K1.data);
  cv::Matx33d K1_c(data_k1c);
  double* data_k2c = reinterpret_cast<double*>(K2.data);
  cv::Matx33d K2_c(data_k2c);
  double* data_R = reinterpret_cast<double*>(R.data);
  cv::Matx33d R_c(data_R);
  std::cout << "Finished reading in parameter values." << std::endl;

  cv::Size img_size;
  img_size.height = 800;
  img_size.width = 960;

  cv::Mat rmap[2][2];
  cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_16SC2, rmap[0][0], rmap[0][1]);
  cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_16SC2, rmap[1][0], rmap[1][1]);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
