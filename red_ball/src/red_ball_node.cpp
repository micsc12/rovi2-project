#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

using namespace cv;

class RedBall
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher xy_pub;
  
public:
  RedBall()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/left/image_raw", 1,
      &RedBall::imageCb, this);
    image_pub_ = it_.advertise("/red_ball/output_video", 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

		// Extract hue information
    Mat hsv_temp, hsv[3];
    cvtColor(cv_ptr->image, hsv_temp, CV_BGR2HSV); // convert image to HSV color space
    split(hsv_temp, hsv);    // split image into hue, saturation and value images
    Mat hue = hsv[1].clone();   // select hue channel
    
    // Perform hue thresholding
		int hue_tol = 5,
		    hue_red = 0,
		    lower = hue_red - hue_tol,
		    upper = hue_red + hue_tol;
		lower = lower >= 0 ? lower : 0;
		upper = upper < 180 ? upper : 180;
		Mat bin;
		inRange(hue, lower, upper, bin);
    
    /// Find contours
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    findContours(bin, contours, hierarchy,
            CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
            
    /// Get the moments
		std::vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++) {
            mu[i] = moments(contours[i], false);
    }
    
    /// Find the most circular contour in the binary image
    double maxRatio = 0, curRatio;
    Point2f maxCenter, curCenter;
    for (int i = 0; i < contours.size(); i++) {
      float radius;
      minEnclosingCircle(contours[i], curCenter, radius);
      curRatio = mu[i].m00 / radius;  // mu[i].m00 is the contour area
      if ( curRatio > maxRatio ) {
        maxRatio = curRatio;
        maxCenter = curCenter;
      }
    }
    
    // Draw ball position onto original stream
    Scalar color(0, 255, 0);
    circle( cv_ptr->image,
    				maxCenter,
            3,
            color,
            5,
            8 );
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    
    xy_pub = nh_.advertise<geometry_msgs::PoseStamped>("opencvtest/center", 1000);
    
    geometry_msgs::PoseStamped msg_center;
    msg_center.pose.position.x = maxCenter.x;
    msg_center.pose.position.y = maxCenter.y;
    msg_center.pose.position.z = 0;
    xy_pub.publish(msg_center);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "red_ball");
  RedBall rb;
  ros::spin();
  return 0;
}
