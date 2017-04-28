#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>


#include </home/jesper/catkin_ws/devel/include/red_ball/center.h>
#include </home/jesper/catkin_ws/devel/include/stereo_cam/point.h>


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;

class stereopsis
{

public:
  /*void leftCb(const red_ball::center::ConstPtr& msg)
  {
      
  }*/
  stereopsis()
  {
      // subcribe to center coordinates
      //center_sub_right = nh_.subscribe("red_ball/center_right", 1,stereopsis::rightCB, this); // 1000 queue size normally
      //center_sub_left = nh_.subscribe("red_ball/center_left", 1,stereopsis::leftCB, this); // 1000 queue size normally
      center_sub_left = nh_.subscribe("red_ball/center_left", 1000,&stereopsis::leftCb,this);
      center_sub_right = nh_.subscribe("red_ball/center_right", 1000,&stereopsis::rightCb, this);
      position_3D = nh_.advertise<stereo_cam::point>("red_ball/3D_Point",100);
  }

  void leftCb(const red_ball::center& msg)
  {
    
    left=msg;
    left_center=true;
    do_stereo_calc();
    
  }
  
  
   void rightCb(const red_ball::center& msg)
  {
    right=msg;
    right_center=true;
    do_stereo_calc();
  }
  
  void do_stereo_calc()
  {
      if(right_center==true && left_center==true)
      {
        right_center=false;
        left_center=false;
        Mat projection_l = Mat::zeros(3, 4, CV_64F);
        projection_l.at<double>(0,0)=1473.179202;
        projection_l.at<double>(0,2)=500.873203;
        projection_l.at<double>(1,1)=1473.179202;
        projection_l.at<double>(1,2)=421.267643;
        projection_l.at<double>(2,2)=1;
        
       // cout << "projection_l = "<< endl << " "  << projection_l << endl << endl;
        
        
        Mat projection_r = Mat::zeros(3, 4, CV_64F);
        projection_r.at<double>(0,0)=1473.179202;
        projection_r.at<double>(0,2)=500.873203;
        projection_r.at<double>(1,1)=1473.179202;
        projection_r.at<double>(1,2)=421.267643;
        projection_r.at<double>(2,2)=1;
        projection_r.at<double>(0,3)=-175.374968;
        
        //cout << "projection_r = "<< endl << " "  << projection_r << endl << endl;
        
        Mat pnts3D(1, 1, CV_64FC4);
        Mat cam0pnts(1, 1, CV_64FC2);
        Mat cam1pnts(1, 1, CV_64FC2);
        cam0pnts.at<Vec2d>(0)[0] = left.x;
        cam0pnts.at<Vec2d>(0)[1] = left.y;
        cam1pnts.at<Vec2d>(0)[0] = right.x;
        cam1pnts.at<Vec2d>(0)[1] = right.y;
        triangulatePoints(projection_l, projection_r, cam0pnts, cam1pnts, pnts3D);
        cout << "OpenCV triangulation" << endl;
        cout << "Image points: " << cam0pnts << "\t" << cam1pnts << endl << endl;
        cout << "Triangulated point (normalized): " << endl << pnts3D / pnts3D.at<double>(3, 0) << endl << endl;
        pnts3D=pnts3D / pnts3D.at<double>(3, 0);
        double euclidian=sqrt(pow(pnts3D.at<double>(0, 0),2)+pow(pnts3D.at<double>(1, 0),2)+pow(pnts3D.at<double>(2, 0),2));
        cout << "Euclidian distance is: " << euclidian << endl << endl;
        stereo_cam::point result;
        result.x=(pnts3D.at<double>(0, 0) / pnts3D.at<double>(3, 0))*1000; // result in meters
        result.y=(pnts3D.at<double>(1, 0) / pnts3D.at<double>(3, 0))*1000;
        result.z=(pnts3D.at<double>(2, 0) / pnts3D.at<double>(3, 0))*1000;
        
        position_3D.publish(result);
        //cout << "Wow " << endl;
      }
  }
private:
     ros::NodeHandle nh_;

  ros::Subscriber center_sub_left;
  ros::Subscriber center_sub_right;
  
  ros::Publisher position_3D;
  
  bool right_center=false;
  bool left_center=false;
  red_ball::center left;
  red_ball::center right;
};
  
  
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_cam");
  stereopsis rb;
  ros::spin();
  return 0;
}
