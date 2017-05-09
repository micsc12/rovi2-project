#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <string>

using namespace std;
using namespace cv;


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

cout << camera_r << endl;
Mat trans_l=Mat::eye(4, 4, CV_64F);
Mat trans_r=Mat::eye(4, 4, CV_64F);
trans_r.at<double>(0,3)=-0.12;

projection_l=camera_l*trans_l;
projection_r=camera_r*trans_r;


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

int main()
{
		setup();
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

		  stereo_cam::point result;
		  result.x=(M.at<double>(0, 0))*1000; // result in meters
		  result.y=(M.at<double>(1, 0))*1000;
		  result.z=(M.at<double>(2, 0))*1000;
		  
		  position_3D.publish(result);
		  //cout << "Wow " << endl;
}
