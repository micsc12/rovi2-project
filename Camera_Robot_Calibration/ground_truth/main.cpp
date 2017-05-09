#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <string>


using namespace cv;
using namespace std;

string pathleft="/home/bjarkips/workspace/rovi2-project/Camera_Robot_Calibration/left/image";
string pathright="/home/bjarkips/workspace/rovi2-project/Camera_Robot_Calibration/right/image";
string png=".png";
Mat img;

bool click=false;

vector<Point2f> vision_algorithm, ground_truth;

Mat camera_l = Mat::zeros(3, 4, CV_64F);
// rows, col

Mat camera_r = Mat::zeros(3, 4, CV_64F);
// rows, col

//cout << camera_r << endl;
Mat trans_l=Mat::eye(4, 4, CV_64F);
Mat trans_r=Mat::eye(4, 4, CV_64F);

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

void getCenter() {
		// Extract hue information

    // red marker 3 times standard deviation
    int lowR=84; // sample 22
    int highR=255;
    int lowG=0; // see identify -verbose on the sample
    int highG=20+14*3; 
    int lowB=0;
    int highB=3+3*11;
    //cvtColor(cv_ptr->image, img, CV_BGR);
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
    for (int i = 0; i < contours.size(); i++) {
      float radius;
      minEnclosingCircle(contours[i], curCenter, radius);
      curRatio = mu[i].m00 / radius;  // mu[i].m00 is the contour area
      if ( curRatio > maxRatio ) {
        maxRatio = curRatio;
        maxCenter = curCenter;
      }
    }
    
     vision_algorithm.push_back(maxCenter);

}



void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
        //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        Point2f clicked_center;
        clicked_center.x=x;
        clicked_center.y=y;
        ground_truth.push_back(clicked_center);
        click=true;
        
        getCenter();

     }
     
}
     




int main()
{
    int n=10;
    namedWindow( "Display window", WINDOW_AUTOSIZE );
    //set the callback function for any mouse event
    setMouseCallback("Display window", CallBackFunc, NULL);
    
    
	camera_l.at<double>(0,0)=1316.084092;
	camera_l.at<double>(0,2)=506.400063;
	camera_l.at<double>(1,1)=1316.417843;
	camera_l.at<double>(1,2)=372.835099;
	camera_l.at<double>(2,2)=1;
	
	camera_r.at<double>(0,0)=1318.204959;
	camera_r.at<double>(0,2)=517.054064;
	camera_r.at<double>(1,1)=1318.239559;
	camera_r.at<double>(1,2)=463.621512;
	camera_r.at<double>(2,2)=1;		
	
	trans_r.at<double>(0,3)=-0.12;
	
	Mat projection_l=camera_l*trans_l;
	Mat projection_r=camera_r*trans_r;
    for (int i=0; i<100 ; i++)
    {
    	//cout << "Does anyone hear me?"<< endl;
    	//cout << i%2 << endl;
    	//cout << i/2+1 << endl;
    		if (!(i%2)) {
		      //cout << "Left image " << i/2+1 << endl;
		      //cout << pathleft+to_string(i/2+1)+png << endl;
		      //if( i < 10)
		          //cout << path+"00"+to_string(i)+png << endl;
		        //  img=imread(pathleft+"00"+to_string(i/2+1)+png);
		      //else if( i > 10 && i<100)
		        //  img=imread(pathleft+"0"+to_string(i/2+1)+png);
		      //else if( i > 100 && i<1000)
		          img=imread(pathleft+to_string(i/2+1)+png);
		      
		      imshow( "Display window", img );
		      waitKey(0);
        } else {
		      //cout << "Right image" << i/2+1 << endl;
		      //cout << pathright+to_string(i/2+1)+png << endl;
		      //if( i < 10)
		          //cout << path+"00"+to_string(i)+png << endl;
		        //  img=imread(pathright+"00"+to_string(i/2+1)+png);
		      //else if( i > 10 && i<100)
		        //  img=imread(pathright+"0"+to_string(i/2+1)+png);
		      //else if( i > 100 && i<1000)
		          img=imread(pathright+to_string(i/2+1)+png);
		      imshow( "Display window", img );
		      waitKey(0);
       	}
        /*while(click==false)
        {
        }
        click=false;*/
        //waitKey(0);
        
        for(int i=0 ; i< ground_truth.size() ; i+= 2)
        {
            //cout << "ground_truth is x: " << ground_truth[i].x << " y: " << ground_truth[i].y << endl;
            //cout << "vision_algorithm is x: " << vision_algorithm[i].x << " y: " << vision_algorithm[i].y << endl;
            Mat pnts3D(1, 1, CV_64FC4);
						Mat cam0pnts(1, 1, CV_64FC2);
						Mat cam1pnts(1, 1, CV_64FC2);
						cam0pnts.at<Vec2d>(0)[0] = ground_truth[i].x;
						cam0pnts.at<Vec2d>(0)[1] = ground_truth[i].y;
						cam1pnts.at<Vec2d>(0)[0] = ground_truth[i+1].x;
						cam1pnts.at<Vec2d>(0)[1] = ground_truth[i+1].y;
		
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
						
						cout << "Image pair " << i/2+1 << ":" << endl;
						cout << M << endl;

						//stereo_cam::point result;
						//result.x=(M.at<double>(0, 0))*1000; // result in meters
						//result.y=(M.at<double>(1, 0))*1000;
						//result.z=(M.at<double>(2, 0))*1000;
		
						//position_3D.publish(result);
        }
    }
	
	
  //cout << "Wow " << endl;

    /*
    // Code that peter doesn't need
    Mat view_left;
    
    view_left=Mat::zeros(ground_truth.size(), 2, CV_64F);
    for(int i=0 ; i<ground_truth.size() ;i++)
    {
        view_left.at<double>(i,0)=vision_algorithm[i].x-ground_truth[i].x;
        view_left.at<double>(i,1)=vision_algorithm[i].y-ground_truth[i].y;
    }
    cout << view_left << endl;
    
    Mat Sigma_left=Mat::zeros(ground_truth.size(), 2, CV_64F);
    Sigma_left=(1.0/((double)(ground_truth.size())-1.0))*view_left.t()*view_left;
    
    cout << Sigma_left << endl;
    
    /*double average=0;
    double max=0;
    for(int i=0 ; i< ground_truth.size() ; i++)
    {
        double x=ground_truth[i].x-vision_algorithm[i].x;
        double y=ground_truth[i].y-vision_algorithm[i].y;
        double euclidian=sqrt(pow(x,2)+pow(y,2));
        average=average+euclidian;
        if(euclidian>max)
            max=euclidian;
    }
    average=average/ground_truth.size();
    double variance=0;
    for(int i=0 ; i< ground_truth.size() ; i++)
    {
        double x=ground_truth[i].x-vision_algorithm[i].x;
        double y=ground_truth[i].y-vision_algorithm[i].y;
        double euclidian=sqrt(pow(x,2)+pow(y,2));
        variance = variance + pow(euclidian-average,2);
    }
    variance=variance/ground_truth.size();*/
    
    /*cout << "Max error is: " << max << endl;
    cout << "Average is: " << average << endl;
    cout << "Variance is: " << variance << endl;*/
 }
