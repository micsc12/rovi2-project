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
  ros::Publisher center_pub_right;
  red_ball::center old_center;
  
public:
  red_ball_tracker()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    //image_sub_left = it_.subscribe("/image_decompressed_left", 1, &red_ball_tracker::leftCb, this);
    //image_sub_right = it_.subscribe("/image_decompressed_right", 1, &red_ball_tracker::rightCb, this);
    image_sub_left = it_.subscribe("/camera/left/image_raw", 1, &red_ball_tracker::leftCb, this);
    image_sub_right = it_.subscribe("/camera/right/image_raw", 1, &red_ball_tracker::rightCb, this);
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
    
    center_pub_left = nh_.advertise<red_ball::center>("red_ball/center_left", 1000);
    red_ball::center cent = getCenter(cv_ptr, Point2f(71, 286));
    //cout << cent << endl;
    if (!(cent.x == 0 && cent.y == 0)) {
    	center_pub_left.publish(cent);
    } else {
    	cent.x = -1;
    	cent.y = -1;
    	center_pub_left.publish(cent);
    }
    //center_pub_left.publish( getCenter(cv_ptr, Point2f(71, 286)) );
    
    // Output modified video stream
    image_pub_left.publish(cv_ptr->toImageMsg());
  }
  
   void rightCb(const sensor_msgs::ImageConstPtr& msg)
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
    
    center_pub_right = nh_.advertise<red_ball::center>("red_ball/center_right", 1000);
    red_ball::center cent = getCenter(cv_ptr, Point2f(17, 378));
    //cout << cent << endl;
    if (!(cent.x == 0 && cent.y == 0)) {
    	center_pub_right.publish(cent);
    } else {
    	cent.x = -1;
    	cent.y = -1;
    	center_pub_right.publish(cent);
    }
    
    // Output modified video stream
    image_pub_right.publish(cv_ptr->toImageMsg());
  }
  
  red_ball::center getCenter(cv_bridge::CvImagePtr cv_ptr, Point2f button) {
		// Extract hue information
    /*Mat hsv_temp, hsv[3];
    cvtColor(cv_ptr->image, hsv_temp, CV_BGR2HSV); // convert image to HSV color space
    split(hsv_temp, hsv);    // split image into hue, saturation and value images
    Mat hue = hsv[0].clone();   // select hue channel
    
    // Perform hue thresholding
		int hue_tol = 10,
		    hue_red = 0,
		    hue_blue = 120,
		    lower = hue_blue - hue_tol,
		    upper = hue_blue + hue_tol;
		lower = lower >= 0 ? lower : 0;
		upper = upper < 180 ? upper : 180;
		Mat bin;
		//normalize(hue, hue);
    inRange(hue, lower, upper, bin);
    Mat imgThresholdedRed = bin.clone();*/
    
    //cout << "Jeg er herinde" << endl;
    // red marker 3 times standard deviation
    int lowR=90; // sample 22
    int highR=255;
    int lowG=0; // see identify -verbose on the sample
    int highG=50; 
    int lowB=0;
    int highB=45;
    Mat img=cv_ptr->image.clone();
    //cvtColor(cv_ptr->image, img, CV_BGR);
    
    // Hide the red button
    //Point2f button(71, 286);
    //Point2f buttonRight(17, 378);
    circle( img,
    				button,
            10,
            Scalar(0,0,0),
            -1,
            8 );
    
    Mat imgThresholdedRed;
    inRange(img, Scalar(lowB, lowG, lowR), Scalar(highB, highG, highR), imgThresholdedRed);
    erode(imgThresholdedRed, imgThresholdedRed, getStructuringElement(MORPH_RECT, Size(5, 5)) );
    dilate( imgThresholdedRed, imgThresholdedRed, getStructuringElement(MORPH_RECT, Size(5, 5)) );
    
    
    
    Mat con = imgThresholdedRed.clone();
    
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
    bool found_center = false;
    for (int i = 0; i < contours.size(); i++) {
      float radius;
      minEnclosingCircle(contours[i], curCenter, radius);
      curRatio = mu[i].m00 / radius;  // mu[i].m00 is the contour area
      if ( curRatio > maxRatio ) {
        maxRatio = curRatio;
        maxCenter = curCenter;
        found_center = true;
      }
    }
    
    // Draw ball position onto original stream
    Scalar color(0, 255, 0);
    if (found_center) {
    	circle( cv_ptr->image,
		  				maxCenter,
		          3,
		          color,
		          5,
		          8 );
    }
    
    red_ball::center msg_center;
    //if (found_center) {
    	msg_center.x = maxCenter.x;
    	msg_center.y = maxCenter.y;
    //}
    /*else {
    	msg_center = old_center;
    	cout << "Couldn't find center, outputting " << msg_center << endl;
    }
    //imshow("bin", bin);
    //imshow("hue", hue);
    //waitKey(0);
    
    old_center = msg_center;*/
    
    return(msg_center);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "red_ball");
  red_ball_tracker rb;
  ros::spin();
  return 0;
}
