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
#include <cstdio>
#include <sys/time.h>

#include "popt_pp.h"

#include <sstream>
#include <iomanip>
#include <string>
#include <memory>

#include <NVX/nvx.h>
#include <NVX/nvx_timer.hpp>
#include <NVX/nvx_opencv_interop.hpp>

#include <Application.hpp>
#include <ConfigParser.hpp>
#include <UtilityOVX.hpp>

#include "stereo_matching.hpp"

#define IMG_HEIGHT 800
#define IMG_WIDTH 960

cv::Mat rmap[2][2];
vx_image left_rect;
vx_image right_rect;
vx_image disparity;
StereoMatching::StereoMatchingParams params;
ovxio::ContextGuard context;
StereoMatching::ImplementationType implementationType;
int counter_global = 0;
int baseline_opt;
vx_uint32 plane_index = 0;
vx_rectangle_t rect = {0u,0u,960u,800u};

static bool read(const std::string &nf, StereoMatching::StereoMatchingParams &config, std::string &message)
{
    std::unique_ptr<nvxio::ConfigParser> parser(nvxio::createConfigParser());
    parser->addParameter("min_disparity",
                         nvxio::OptionHandler::integer(
                             &config.min_disparity,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("max_disparity",
                         nvxio::OptionHandler::integer(
                             &config.max_disparity,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("P1",
                         nvxio::OptionHandler::integer(
                             &config.P1,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("P2",
                         nvxio::OptionHandler::integer(
                             &config.P2,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("sad",
                         nvxio::OptionHandler::integer(
                             &config.sad,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(31)));
    parser->addParameter("bt_clip_value",
                         nvxio::OptionHandler::integer(
                             &config.bt_clip_value,
                             nvxio::ranges::atLeast(15) & nvxio::ranges::atMost(95)));
    parser->addParameter("max_diff",
                         nvxio::OptionHandler::integer(
                             &config.max_diff));
    parser->addParameter("uniqueness_ratio",
                         nvxio::OptionHandler::integer(
                             &config.uniqueness_ratio,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(100)));
    parser->addParameter("scanlines_mask",
                         nvxio::OptionHandler::integer(
                             &config.scanlines_mask,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("flags",
                         nvxio::OptionHandler::integer(
                             &config.flags,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(3)));
    parser->addParameter("ct_win_size",
                         nvxio::OptionHandler::integer(
                             &config.ct_win_size,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(5)));
    parser->addParameter("hc_win_size",
                         nvxio::OptionHandler::integer(
                             &config.hc_win_size,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(5)));
    message = parser->parse(nf);
    return message.empty();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  nvx::Timer read_rect_timer;
  read_rect_timer.tic();
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img1,img2,img3,r_img1,r_img2;
  cv::Rect myROI_3(0,0*IMG_HEIGHT,IMG_WIDTH,IMG_HEIGHT); // camera 3
  cv::Rect myROI_2(0,1*IMG_HEIGHT,IMG_WIDTH,IMG_HEIGHT); // camera 2
  cv::Rect myROI_1(0,2*IMG_HEIGHT,IMG_WIDTH,IMG_HEIGHT); // camera 1
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //img1 = cv_ptr->image(myROI_1); // camera 1
    //img2 = cv_ptr->image(myROI_2); // camera 2
    //img3 = cv_ptr->image(myROI_3); // camera 3

    img1 = cv_ptr->image(myROI_1);
    if (baseline_opt == 0)
	img2 = cv_ptr->image(myROI_2);
    else
	img2 = cv_ptr->image(myROI_3);
    

    cv::remap(img1,r_img1,rmap[0][0],rmap[0][1],cv::INTER_LINEAR); // camera 2 
    cv::remap(img2,r_img2,rmap[1][0],rmap[1][1],cv::INTER_LINEAR); // camera 3

    //cv::imshow("view1", r_img1);
    //cv::imshow("view2", r_img2);
    //cv::imshow("view3", r_img3);  

    left_rect = nvx_cv::createVXImageFromCVMat(context,r_img1);
    NVXIO_CHECK_REFERENCE(left_rect);
    right_rect = nvx_cv::createVXImageFromCVMat(context,r_img2);
    NVXIO_CHECK_REFERENCE(right_rect);

    disparity = vxCreateImage(context, IMG_WIDTH, IMG_HEIGHT, VX_DF_IMAGE_U8);
    NVXIO_CHECK_REFERENCE(disparity);
    std::unique_ptr<StereoMatching> stereo(StereoMatching::createStereoMatching(context,
					   params,implementationType,left_rect, right_rect, disparity));
    stereo->run(); 

    nvx_cv::VXImageToCVMatMapper map(disparity,plane_index,&rect,VX_READ_ONLY,VX_MEMORY_TYPE_HOST);
    cv::Mat disp = map.getMat();
    cv::imshow("disparity",disp);

    //char savefilename[50];
    //snprintf(savefilename,sizeof(savefilename),"/home/nvidia/saveddisp-3/disparity%05d.png",counter_global);
    //counter_global = counter_global + 1;
    //printf("%s",savefilename);
    //cv::imwrite(savefilename,disp);

    cv::waitKey(1);
    double timer = read_rect_timer.toc();
    std::cout << "Time Elapsed For Rect + SGBM : " << timer << " ms" << std::endl << std::endl;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}



int main(int argc, char **argv)
{
  char* b1_param_file;
  char* b2_param_file;
  char* param_file;

  static struct poptOption options[] = {
  { "b1_param_file",'c',POPT_ARG_STRING,&b1_param_file,0,"Destination for b1 calib file","STR" },
  { "b2_param_file",'f',POPT_ARG_STRING,&b2_param_file,0,"Destination for b2 calib file","STR" },
  { "baseline_opt",'b',POPT_ARG_INT,&baseline_opt,0,"Baseline option","NUM"},
  POPT_AUTOHELP
  { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  cv::startWindowThread();

  // Read in camera parameters 
  cv::Vec3d T;
  cv::Vec4d D1,D2;
  cv::Mat R1, R2, P1, P2, Q, K1, K2, R;

  if (baseline_opt == 0)
    param_file = b1_param_file;
  else
    param_file = b2_param_file;

  cv::FileStorage fs(param_file,cv::FileStorage::READ);
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

  // Calculate rectification re-mapping / warping
  cv::Size img_size;
  img_size.height = IMG_HEIGHT;
  img_size.width = IMG_WIDTH;

  cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, img_size, CV_16SC2, rmap[0][0], rmap[0][1]);
  cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, img_size, CV_16SC2, rmap[1][0], rmap[1][1]);
  std::cout << "Rectification remapping / warp calculated." << std::endl;

  nvxio::Application &app = nvxio::Application::get();
  app.setDescription("Stereo Matching for Disparity Estimation");

  app.setDescription("Stereo Matching for Disparity Estimation");
  std::string configFile = "/home/nvidia/catkin_ws/src/ros_tegra_stereo/data/stereo_matching_params.ini";

  implementationType = StereoMatching::HIGH_LEVEL_API;
  std::string error;
  if (!read(configFile, params, error)) {
      std::cerr << error;
      return 0;
  }
  std::cout << "SGBM Parameters loaded\n" ;

  vxDirective(context, VX_DIRECTIVE_ENABLE_PERFORMANCE);
  vxRegisterLogCallback(context, &ovxio::stdoutLogCallback, vx_false_e);

  left_rect = vxCreateImage(context, IMG_WIDTH, IMG_HEIGHT, VX_DF_IMAGE_RGBX);
  NVXIO_CHECK_REFERENCE(left_rect);
  right_rect = vxCreateImage(context, IMG_WIDTH, IMG_HEIGHT, VX_DF_IMAGE_RGBX);
  NVXIO_CHECK_REFERENCE(right_rect);
  disparity = vxCreateImage(context, IMG_WIDTH, IMG_HEIGHT, VX_DF_IMAGE_U8);
  NVXIO_CHECK_REFERENCE(disparity);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/image", 1, imageCallback);
  ros::spin();
}
