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

string path_left="/home/jesper/hep/test_bag_4/left/left0";
string path_right="/home/jesper/hep/test_bag_4/right/right0";
string png=".png";
Mat img;


bool click=false;

vector<Point2f> vision_algorithm, ground_truth;

void getCenter() {
		// Extract hue information

    // red marker 3 times standard deviation
    int lowR=118; // sample 22
    int highR=255;
    int lowG=0; // see identify -verbose on the sample
    int highG=30+29*3; 
    int lowB=0;
    int highB=6+3*18;
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
    
    //cout << maxCenter.x << " " << maxCenter.y << endl;
    
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
        //cout << "Clicked: " << clicked_center.x << " " << clicked_center.y << endl;

     }
     
}







int main()
{
    int n=30;
    namedWindow( "Display window", WINDOW_AUTOSIZE );
    //set the callback function for any mouse event
    setMouseCallback("Display window", CallBackFunc, NULL);
    
    for (int i=42; i<42+n ; i++)
    {
        cout << "New image" << endl;
        if( i < 10)
            //cout << path+"00"+to_string(i)+png << endl;
            img=imread(path_left+"00"+std::to_string(i)+png);
        else if( i > 10 && i<100)
            img=imread(path_left+"0"+std::to_string(i)+png);
        else if( i > 100 && i<1000)
            img=imread(path_left+std::to_string(i)+png);
        
        imshow( "Display window", img );
        waitKey(0);
    }
    
    
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
    
    
    
    // ######### right
    ground_truth.clear();
    vision_algorithm.clear();
    
    for (int i=42; i<42+n ; i++)
    {
        cout << "New image" << endl;
        if( i < 10)
            //cout << path+"00"+to_string(i)+png << endl;
            img=imread(path_right+"00"+to_string(i)+png);
        else if( i > 10 && i<100)
            img=imread(path_right+"0"+to_string(i)+png);
        else if( i > 100 && i<1000)
            img=imread(path_right+to_string(i)+png);
        
        imshow( "Display window", img );
        waitKey(0);
        
    }
    Mat view_right;
    
    view_right=Mat::zeros(ground_truth.size(), 2, CV_64F);
    for(int i=0 ; i<ground_truth.size() ;i++)
    {
        view_right.at<double>(i,0)=vision_algorithm[i].x-ground_truth[i].x;
        view_right.at<double>(i,1)=vision_algorithm[i].y-ground_truth[i].y;
    }
    cout << view_left << endl;
    Mat Sigma_right=Mat::zeros(ground_truth.size(), 2, CV_64F);
    Sigma_right=(1.0/((double)(ground_truth.size())-1.0))*view_right.t()*view_right;
    
    cout << Sigma_right << endl;
    
    
    
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