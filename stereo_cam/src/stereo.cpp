#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <red_ball/center.h>
#include <stereo_cam/point.h>


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
    stereopsis()
    {
        // subcribe to center coordinates
        //center_sub_right = nh_.subscribe("red_ball/center_right", 1,stereopsis::rightCB, this); // 1000 queue size normally
        //center_sub_left = nh_.subscribe("red_ball/center_left", 1,stereopsis::leftCB, this); // 1000 queue size normally
        center_sub_left = nh_.subscribe("red_ball/center_left", 1000,&stereopsis::leftCb,this);
        center_sub_right = nh_.subscribe("red_ball/center_right", 1000,&stereopsis::rightCb, this);
        position_3D = nh_.advertise<stereo_cam::point>("stereo_cam/3D_Point",100);
        setup();
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
  
    void setup()
    {
        Mat camera_l = Mat::zeros(3, 4, CV_64F);
        // rows, col
        camera_l.at<double>(0,0)=1316.084092;
        camera_l.at<double>(0,2)=506.400063;
        camera_l.at<double>(1,1)=1316.417843;
        camera_l.at<double>(1,2)=372.835099;
        camera_l.at<double>(2,2)=1;

        Mat camera_r = Mat::zeros(3, 4, CV_64F);
        // rows, col
        camera_r.at<double>(0,0)=1318.204959;
        camera_r.at<double>(0,2)=517.054064;
        camera_r.at<double>(1,1)=1318.239559;
        camera_r.at<double>(1,2)=463.621512;
        camera_r.at<double>(2,2)=1;

        //cout << camera_r << endl;
        Mat trans_l=Mat::eye(4, 4, CV_64F);
        Mat trans_r=Mat::eye(4, 4, CV_64F);
        trans_r.at<double>(0,3)=-0.12;

        projection_l=camera_l*trans_l;
        projection_r=camera_r*trans_r;
    }
  
    Mat get_A_i(Mat projection,Mat point)
    {
        Mat A_i;
        Mat Q1=projection.row(0);
        Q1=Q1.colRange(0,3);
        Mat Q2=projection.row(1);
        Q2=Q2.colRange(0,3);
        Mat Q3=projection.row(2);
        Q3=Q3.colRange(0,3);
        
        A_i = Mat::zeros(2, 3, CV_64F);
        
        Mat first=Q1-point.at<Vec2d>(0)[0]*Q3;
        Mat second=Q2-point.at<Vec2d>(0)[1]*Q3;

        first.copyTo(A_i(Rect(0, 0, first.cols, first.rows)));
        second.copyTo(A_i(Rect(0, 1, second.cols, second.rows)));
        
        //cout << A_i << endl;
        
        return A_i;
    }

    Mat get_b_i(Mat projection,Mat point)
    {
        Mat b_i;
        b_i = Mat::zeros(2, 1, CV_64F);
        b_i.at<double>(0,0) = point.at<Vec2d>(0)[0]*projection.at<double>(2,3)-projection.at<double>(0,3);
        b_i.at<double>(1,0) = point.at<Vec2d>(0)[1]*projection.at<double>(2,3)-projection.at<double>(1,3);
        
        
        //cout << b_i << endl;
        return b_i;
    }
  
    void do_stereo_calc()
    {
        if(right_center==true && left_center==true)
        {
            right_center=false;
            left_center=false;
            
            stereo_cam::point result;
            
            if (right.x == -1 || left.x == -1) {
            	result.x = -1;
            	result.y = -1;
            	result.z = -1;
            	position_3D.publish(result);
            	return;
            }

            
            Mat pnts3D(1, 1, CV_64FC4);
            Mat cam0pnts(1, 1, CV_64FC2);
            Mat cam1pnts(1, 1, CV_64FC2);
            cam0pnts.at<Vec2d>(0)[0] = left.x;
            cam0pnts.at<Vec2d>(0)[1] = left.y;
            cam1pnts.at<Vec2d>(0)[0] = right.x;
            cam1pnts.at<Vec2d>(0)[1] = right.y;
            
            Mat A1=get_A_i(projection_l,cam0pnts);
            Mat A2=get_A_i(projection_r,cam1pnts);
            
            Mat A;
            A = Mat::zeros(4, 3, CV_64F);
            A1.copyTo(A(Rect(0, 0, A1.cols, A1.rows)));
            A2.copyTo(A(Rect(0, 2, A2.cols, A2.rows)));
           
            
            Mat b1=get_b_i(projection_l,cam0pnts);
            Mat b2=get_b_i(projection_r,cam1pnts);
            Mat b;
            b = Mat::zeros(4, 1, CV_64F);
            b1.copyTo(b(Rect(0, 0, b1.cols, b1.rows)));
            b2.copyTo(b(Rect(0, 2, b2.cols, b2.rows)));
            
            Mat part=A.t()*A;
            Mat M=part.inv()*A.t()*b;
        
            result.x=(M.at<double>(0, 0))*1000; // result in meters
            result.y=(M.at<double>(1, 0))*1000;
            result.z=(M.at<double>(2, 0))*1000;
            
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

    Mat projection_l;
    Mat projection_r;
};
  
  
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_cam");
  stereopsis rb;
  ros::spin();
  return 0;
}
