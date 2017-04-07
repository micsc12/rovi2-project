#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <red_ball/center.h>

using namespace cv;
using namespace std;

class red_ball_tracker
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_left;
  image_transport::Subscriber image_sub_right;
  image_transport::Publisher image_pub_left;
  image_transport::Publisher image_pub_right;
  ros::Publisher center_pub_left;
  ros::Publisher center_pub_right
  
public:
  red_ball_tracker()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_left = it_.subscribe("/image_decompressed_left", 1, &red_ball_tracker::leftCb, this);
    image_sub_right = it_.subscribe("/image_decompressed_right", 1, &red_ball_tracker::rightCb, this);
    //image_sub_ = it_.subscribe("/camera/left/image_raw", 1, &red_ball_tracker::imageCb, this);
      
    image_pub_left = it_.advertise("/red_ball/output_video_left", 1);
    image_pub_right = it_.advertise("/red_ball/output_video_right", 1);
  }

  void leftCb(const sensor_msgs::ImageConstPtr& msg)
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
    Mat hue = hsv[0].clone();   // select hue channel
    
    // Perform hue thresholding
		int hue_tol = 10,
		    hue_red = 0,
		    lower = hue_red - hue_tol,
		    upper = hue_red + hue_tol;
		lower = lower >= 0 ? lower : 0;
		upper = upper < 180 ? upper : 180;
		Mat bin;
		//inRange(hue, lower, upper, bin);
		//normalize(hue, hue, 0, 180, NORM_MINMAX);
		//normalize(hue,hue);
		//normalize(hue, hue, 1, NORM_L2);
    inRange(hue, lower, upper, bin);
    //cout << hue.channels() << endl;
    //inRange(hue, 0, 0, bin);
    Mat con = bin.clone();
    
    /// Find contours
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    findContours(con, contours, hierarchy,
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
    
    /*xy_pub = nh_.advertise<geometry_msgs::PoseStamped>("red_ball/center_left", 1000);
    
    geometry_msgs::PoseStamped msg_center;
    msg_center.pose.position.x = maxCenter.x;
    msg_center.pose.position.y = maxCenter.y;
    msg_center.pose.position.z = 0;
    xy_pub.publish(msg_center);*/
    
    center_pub = nh_.advertise<red_ball::center>("red_ball/center_left", 1000);
    
    red_ball::center msg_center;
    msg_center.x = maxCenter.x;
    msg_center.y = maxCenter.y;
    center_pub.publish(msg_center);
    
    //FileStorage fs("/home/bjarkips/workspace/test.yml", FileStorage::WRITE);
    //fs << "cameraMatrix" << hue;
		//fs.release();
    
    //imshow("bin", bin);
    //imshow("hue", hue);
    //cv::waitKey(0);
    
    double min, max;
    minMaxLoc(hue, &min, &max);
    //cout << "Min is "<<min<<", max is "<<max <<endl;
    //cout << "Hue lower:\t" << lower << "\tUpper:" << upper << endl;
    //cout << CV_8U << "Hue depth:\t" << hue.depth() << endl;
  }
  
  red_ball::center getCenter(const sensor_msgs::ImageConstPtr& msg) {
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "red_ball");
  red_ball_tracker rb;
  ros::spin();
  return 0;
}
