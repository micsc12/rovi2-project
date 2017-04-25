#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_image");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub_left = it.advertise("camera/left/image_raw", 1);
  //image_transport::Publisher pub_right = it.advertise("camera/right/image_raw", 1);
  image_transport::Publisher pub_left = it.advertise("image_decompressed_left", 1);
  image_transport::Publisher pub_right = it.advertise("image_decompressed_right", 1);
  
  //cv::Mat image_left = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  //cv::Mat image_right = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
  
  cv::Mat image_left = cv::imread("/home/bjarkips/catkin_ws/src/dummy_image/img/c1left.png", CV_LOAD_IMAGE_COLOR);
  cv::Mat image_right = cv::imread("/home/bjarkips/catkin_ws/src/dummy_image/img/c1right.png", CV_LOAD_IMAGE_COLOR);
  
  sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_left).toImageMsg();
  sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_right).toImageMsg();
//418 near
//454 far
  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub_left.publish(msg_left);
    pub_right.publish(msg_right);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
