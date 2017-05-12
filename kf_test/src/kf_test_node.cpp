#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <stdio.h>
#include <red_ball/center.h>
#include <stereo_cam/point.h>

using namespace cv;
using namespace std;

class kf_test
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_left;
  image_transport::Subscriber image_sub_right;
  image_transport::Publisher image_pub_left;
  image_transport::Publisher image_pub_right;
  ros::Subscriber pos3d_sub;
  ros::Subscriber pred_sub;
  ros::Publisher center_pub_right;
  
  Mat pred = Mat(4,1,CV_64F);
  Mat pos = Mat(4,1,CV_64F);
  int pred_ack = 0,
  	pos_ack = 0;
  	
  Mat camera_l = Mat::zeros(3, 4, CV_64F);
  Mat camera_r = Mat::zeros(3, 4, CV_64F);

  Mat trans_l=Mat::eye(4, 4, CV_64F);
  Mat trans_r=Mat::eye(4, 4, CV_64F);

	Mat projection_l;
	Mat projection_r;
  
public:
  kf_test()
    : it_(nh_)
  {      
    image_sub_left = it_.subscribe("/red_ball/output_video_left", 1, &kf_test::leftCb, this);
    image_sub_right = it_.subscribe("/red_ball/output_video_right", 1, &kf_test::rightCb, this);
    pos3d_sub = nh_.subscribe("/stereo_cam/3D_Point", 1, &kf_test::posCb, this);
    pred_sub = nh_.subscribe("/kalman_filter/predicted_pos", 1, &kf_test::predCb, this);
    //center_sub_left = it_.subscribe("/red_ball/center_left", 1, &kf_test::cleftCb, this);
    //center_sub_right = it_.subscribe("/red_ball/center_right", 1, &kf_test::crightCb, this);
    
    image_pub_left = it_.advertise("/kf_test/output_video_left", 1);
    image_pub_right = it_.advertise("/kf_test/output_video_right", 1);
    
		// rows, col
		camera_l.at<double>(0,0)=1316.084092;
		camera_l.at<double>(0,2)=506.400063;
		camera_l.at<double>(1,1)=1316.417843;
		camera_l.at<double>(1,2)=372.835099;
		camera_l.at<double>(2,2)=1;

		// rows, col
		camera_r.at<double>(0,0)=1318.204959;
		camera_r.at<double>(0,2)=517.054064;
		camera_r.at<double>(1,1)=1318.239559;
		camera_r.at<double>(1,2)=463.621512;
		camera_r.at<double>(2,2)=1;
		
		trans_r.at<double>(0,3)=-0.12;
		
		projection_l=camera_l*trans_l;
		projection_r=camera_r*trans_r;
		
		cout << "projection left\n" << projection_l << endl;
		cout << "projection right\n" << projection_r << endl;
		
  }
  
  void predCb(const stereo_cam::point& msg) {
  	if (!pred_ack) {
			pred.at<double>(0,0) = msg.x;
			pred.at<double>(1,0) = msg.y;
			pred.at<double>(2,0) = msg.z;
			pred.at<double>(3,0) = 1;
			pred_ack = 2;
		}
		//cout << "Inside prediction callback" << endl;
  }  
  
  void posCb(const stereo_cam::point& msg) {
  	if (!pos_ack) {
			pos.at<double>(0,0) = msg.x;
			pos.at<double>(1,0) = msg.y;
			pos.at<double>(2,0) = msg.z;
			pos.at<double>(3,0) = 1;
			pos_ack = 2;
  	}
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
    //cout << "pred_ack: " << pred_ack << "\t pos_ack" << pos_ack << endl;
    if (pred_ack == 2 && pos_ack == 2) {
    	pred_ack = 1; pos_ack = 1;
    	
    	//TODO: project points onto left image
    	//cout << "Attempting 1" << endl;
    	//cout << pos << endl;
    	Mat pos_left = projection_l * pos;
    	pos_left = pos_left / pos_left.at<double>(2,0);
    	//cout << "Attempting 2" << endl;
    	Mat pred_left = projection_l * pred;
    	pred_left = pred_left / pred_left.at<double>(2,0);
    	//cout << "Left pos:" << pos_left << "\nLeft pred: " << pred_left << endl;
    	
    	Point2f center_pos(pos_left.at<double>(0), pos_left.at<double>(1));
    	Point2f center_pred(pred_left.at<double>(0), pred_left.at<double>(1));
    	//cout << "Stereo pos: " << center_pos << "\nKalman pred: " << center_pred << endl;
    	
    	Point2f center_dummy(500.5,500.5);
    	
    	Scalar blue(255, 0, 0);
    	Scalar cyan(255, 255, 0);
    	/*circle( cv_ptr->image,
    				center_pos,
            3,
            blue,
            5,
            8 );*/
      circle( cv_ptr->image,
    				center_pred,
            3,
            blue,
            5,
            8 );
    	
		  // Output modified video stream
    }
    
		image_pub_left.publish(cv_ptr->toImageMsg());
    
    //center_pub_left = nh_.advertise<red_ball::center>("red_ball/center_left", 1000);
    //center_pub_left.publish(getCenter(cv_ptr));
    
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
    
    if (pred_ack == 1 && pos_ack == 1) {
    	pred_ack = 0; pos_ack = 0;
    
		  //TODO: project points onto right image
		  
		  // Output modified video stream
		  image_pub_right.publish(cv_ptr->toImageMsg());
		}
		  //center_pub_right = nh_.advertise<red_ball::center>("red_ball/center_right", 1000);
		  //center_pub_right.publish(getCenter(cv_ptr));
  }
  /*
  void cleftCb(const red_ball::center& msg) {}
  void crightCb(const red_ball::center& msg) {}
  /**/
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kf_test");
  kf_test kft;
  ros::spin();
  return 0;
}
