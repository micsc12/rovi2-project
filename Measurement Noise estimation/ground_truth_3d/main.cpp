#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>


using namespace std;
using namespace cv;

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



// ############# TRANSPOSED SECTION
// For a specific A_i
Mat dev_AiT_ui(Mat p)
{
    Mat projection = p.clone();
    Mat Q3=projection.row(2);
    Q3=Q3.colRange(0,3);
    Q3=-Q3;
    Q3=Q3.t();
    
    Mat result;
    result = Mat::zeros(3, 2, CV_64F);
    Q3.copyTo(result(Rect(0, 0, Q3.cols, Q3.rows)));
    
    return result;
}

// For the entire A matrix
Mat dev_AT_ui(Mat p1, Mat p2, int i)
{
    
    Mat result= Mat::zeros(3, 4, CV_64F);
    if(i==1)
    {    
        Mat dev_A1=dev_AiT_ui(p1); 
        dev_A1.copyTo(result(Rect(0, 0, dev_A1.cols, dev_A1.rows)));
    }
    else if(i==2)
    {
        Mat dev_A2=dev_AiT_ui(p2); 
        dev_A2.copyTo(result(Rect(2, 0, dev_A2.cols, dev_A2.rows)));
    }
    
    
    return result;
    // # DO THIS FOR ALL!! Including b 
}



// For a specific A_i
Mat dev_AiT_vi(Mat p)
{
    Mat projection = p.clone();
    Mat Q3=projection.row(2);
    Q3=Q3.colRange(0,3);
    Q3=-Q3;
    Q3=Q3.t();
    
    Mat result;
    result = Mat::zeros(3, 2, CV_64F);
    Q3.copyTo(result(Rect(1, 0, Q3.cols, Q3.rows)));
    
    return result;
}


// For the entire A matrix
Mat dev_AT_vi(Mat p1, Mat p2, int i)
{
    
    Mat result= Mat::zeros(3, 4, CV_64F);

    
    if(i==1)
    {    
        Mat dev_A1=dev_AiT_vi(p1); 
        dev_A1.copyTo(result(Rect(0, 0, dev_A1.cols, dev_A1.rows)));
    }
    else if(i==2)
    {
        Mat dev_A2=dev_AiT_vi(p2); 
        dev_A2.copyTo(result(Rect(2, 0, dev_A2.cols, dev_A2.rows)));
    }
    return result;
    // # DO THIS FOR ALL!! Including b 
}


// ############################## NORMAL SECTION 


// For a specific A_i
Mat dev_Ai_ui(Mat p)
{
    Mat projection = p.clone();
    Mat Q3=projection.row(2);
    Q3=Q3.colRange(0,3);
    Q3=-Q3;
    
    Mat result;
    result = Mat::zeros(2, 3, CV_64F);
    Q3.copyTo(result(Rect(0, 0, Q3.cols, Q3.rows)));
    
    return result;
}

// For the entire A matrix
Mat dev_A_ui(Mat p1, Mat p2, int i)
{
    
    Mat result= Mat::zeros(4, 3, CV_64F);
    if(i==1)
    {    
        Mat dev_A1=dev_Ai_ui(p1); 
        dev_A1.copyTo(result(Rect(0, 0, dev_A1.cols, dev_A1.rows)));
    }
    else if(i==2)
    {
        Mat dev_A2=dev_Ai_ui(p2); 
        dev_A2.copyTo(result(Rect(0, 2, dev_A2.cols, dev_A2.rows)));
    }
    
    
    return result;
    // # DO THIS FOR ALL!! Including b 
}

// For a specific A_i
Mat dev_Ai_vi(Mat p)
{
    Mat projection = p.clone();
    Mat Q3=projection.row(2);
    Q3=Q3.colRange(0,3);
    Q3=-Q3;
    
    Mat result;
    result = Mat::zeros(2, 3, CV_64F);
    Q3.copyTo(result(Rect(0, 1, Q3.cols, Q3.rows)));
    
    return result;
}


// For the entire A matrix
Mat dev_A_vi(Mat p1, Mat p2, int i)
{
    
    Mat result= Mat::zeros(4, 3, CV_64F);

    
    if(i==1)
    {    
        Mat dev_A1=dev_Ai_vi(p1); 
        dev_A1.copyTo(result(Rect(0, 0, dev_A1.cols, dev_A1.rows)));
    }
    else if(i==2)
    {
        Mat dev_A2=dev_Ai_vi(p2); 
        dev_A2.copyTo(result(Rect(0, 2, dev_A2.cols, dev_A2.rows)));
    }
    return result;
    // # DO THIS FOR ALL!! Including b 
}


Mat dev_bi_ui(Mat projection)
{
    Mat result;
    result = Mat::zeros(2, 1, CV_64F);
    result.at<double>(0,0)=projection.at<double>(2,3);
    return result;
    
}

Mat dev_b_ui(Mat p1, Mat p2, int i)
{
    Mat result;
    result = Mat::zeros(4, 1, CV_64F);
    Mat b1=dev_bi_ui(p1);
    Mat b2=dev_bi_ui(p2);
    
    if(i==1)
    {    
        b1.copyTo(result(Rect(0, 0, b1.cols, b1.rows)));
    }
    else if(i==2)
    {
        b2.copyTo(result(Rect(0, 2, b2.cols, b2.rows)));
    }

    return result;
    
}

Mat dev_bi_vi(Mat projection)
{
    Mat result;
    result = Mat::zeros(2, 1, CV_64F);
    result.at<double>(0,1)=projection.at<double>(2,3);
    return result;
}

Mat dev_b_vi(Mat p1, Mat p2, int i)
{
    Mat result;
    result = Mat::zeros(4, 1, CV_64F);
    Mat b1=dev_bi_vi(p1);
    Mat b2=dev_bi_vi(p2);
    if(i==1)
        b1.copyTo(result(Rect(0, 0, b1.cols, b1.rows)));
    else if(i==2)
        b2.copyTo(result(Rect(0, 2, b2.cols, b2.rows)));
    
    //result.at<double>(0,0)=projection.at<double>(2,3);
    return result;
    
}

Mat dev_ATA_inv_ui(Mat A,Mat p1, Mat p2, int i)
{
    Mat part=A.t()*A;
    
//     cout << "Part: " << part.rows <<"x" << part.cols << endl;
//     cout << "dev_Ai_ui: " << dev_A_ui(p1,p2).rows <<"x" << dev_A_ui(p1,p2).cols << endl;
//     cout << "dev_AiT_ui: " << dev_AT_ui(p1,p2).rows <<"x" << dev_A_ui(p1,p2).cols << endl;
//     cout << "A: " << A.rows <<"x" << A.cols << endl;
    
    Mat result=-part.inv()*(dev_AT_ui(p1,p2,i)*A+A.t()*dev_A_ui(p1,p2,i))*part.inv();
    return result;
}

Mat dev_ATA_inv_vi(Mat A,Mat p1, Mat p2, int i)
{
    Mat part=A.t()*A;
//     cout << "Part: " << part.rows <<"x" << part.cols << endl;
//     cout << "dev_Ai_vi: " << dev_A_vi(p1,p2,i).rows <<"x" << dev_A_vi(p1,p2,i).cols << endl;
//     cout << "dev_AiT_vi: " << dev_AT_vi(p1,p2,i).rows <<"x" << dev_AT_vi(p1,p2,i).cols << endl;
//     cout << "A: " << A.rows <<"x" << A.cols << endl;
    
    
    Mat result=-part.inv()*(dev_AT_vi(p1,p2,i)*A+A.t()*dev_A_vi(p1,p2,i))*part.inv();
    return result;
}

Mat Jm_i_u(Mat A, Mat b, Mat p1, Mat p2,int i)
{
    Mat dev_ATA_inv_u=dev_ATA_inv_ui(A,p1,p2,i);
    Mat ATA=A.t()*A;
    Mat dev_A_u=dev_A_ui(p1,p2,i);
    Mat dev_b_u=dev_b_ui(p1,p2,i);
    
    Mat part1=dev_ATA_inv_u*A.t()*b;
    Mat part2=ATA.inv()*dev_A_u.t()*b;
    Mat part3=ATA.inv()*A.t()*dev_b_u;
//     cout << "Jm_i_u " << endl;
//     cout << "Part 1: " << endl << part1 << endl;
//     cout << "Part 2: " << endl << part2 << endl;
//     cout << "Part 3: " << endl << part3 << endl;
    Mat result=part1+part2+part3;
    return result;
}


Mat Jm_i_v(Mat A, Mat b, Mat p1, Mat p2,int i)
{
    Mat dev_ATA_inv_v=dev_ATA_inv_vi(A,p1,p2,i);
    Mat ATA=A.t()*A;
    Mat dev_A_v=dev_A_vi(p1,p2,i);
    Mat dev_b_v=dev_b_vi(p1,p2,i);
    
    Mat part1=dev_ATA_inv_v*A.t()*b;
    Mat part2=ATA.inv()*dev_A_v.t()*b;
    Mat part3=ATA.inv()*A.t()*dev_b_v;
//     cout << "Jm_i_v " << endl;
//     cout << "Part 1: " << endl << part1 << endl;
//     cout << "Part 2: " << endl << part2 << endl;
//     cout << "Part 3: " << endl << part3 << endl;
    Mat result=part1+part2+part3;
    return result;
}

     
Mat J_m(Mat A, Mat b, Mat p1, Mat p2)
{
    Mat Jm_i_u1=Jm_i_u(A,b,p1,p2,1);
    Mat Jm_i_u2=Jm_i_u(A,b,p1,p2,2);
    Mat Jm_i_v1=Jm_i_v(A,b,p1,p2,1);
    Mat Jm_i_v2=Jm_i_v(A,b,p1,p2,2);
    
    Mat result = Mat::zeros(3, 4, CV_64F);
    Jm_i_u1.copyTo(result(Rect(0, 0, Jm_i_u1.cols, Jm_i_u1.rows)));
    Jm_i_v1.copyTo(result(Rect(1, 0, Jm_i_v1.cols, Jm_i_v1.rows)));
    Jm_i_u2.copyTo(result(Rect(2, 0, Jm_i_u2.cols, Jm_i_u2.rows)));
    Jm_i_v2.copyTo(result(Rect(3, 0, Jm_i_v2.cols, Jm_i_v2.rows)));
    
    return result;
    
}
     



int main()
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
    
    cout << camera_r << endl;
    Mat trans_l=Mat::eye(4, 4, CV_64F);
    Mat trans_r=Mat::eye(4, 4, CV_64F);
    trans_r.at<double>(0,3)=-0.12;
    
    Mat projection_l=camera_l*trans_l;
    Mat projection_r=camera_r*trans_r;
    
    Mat pnts3D(1, 1, CV_64FC4);
    Mat cam0pnts(1, 1, CV_64FC2);
    Mat cam1pnts(1, 1, CV_64FC2);
    cam0pnts.at<Vec2d>(0)[0] = 579;
    cam0pnts.at<Vec2d>(0)[1] = 481;
    cam1pnts.at<Vec2d>(0)[0] = 505;
    cam1pnts.at<Vec2d>(0)[1] = 575;
    
    Mat A1=get_A_i(projection_l,cam0pnts);
    Mat A2=get_A_i(projection_r,cam1pnts);
    
    Mat A;
    A = Mat::zeros(4, 3, CV_64F);
    A1.copyTo(A(Rect(0, 0, A1.cols, A1.rows)));
    A2.copyTo(A(Rect(0, 2, A2.cols, A2.rows)));
    
    cout << A << endl;
    
    
    Mat b1=get_b_i(projection_l,cam0pnts);
    Mat b2=get_b_i(projection_r,cam1pnts);
    Mat b;
    b = Mat::zeros(4, 1, CV_64F);
    b1.copyTo(b(Rect(0, 0, b1.cols, b1.rows)));
    b2.copyTo(b(Rect(0, 2, b2.cols, b2.rows)));
    
    cout << b << endl;
    
    Mat part=A.t()*A;
    Mat M=part.inv()*A.t()*b;
    bool debug=true;
    if(debug)
    {
        cout << "Results - smaller parts" << endl; 
        //cout << "Projection : " << endl << projection_l << endl;
                
        cout << dev_bi_ui(projection_l) << endl;
        cout << dev_bi_ui(projection_r) << endl;
        cout << dev_bi_vi(projection_l) << endl;
        cout << dev_bi_vi(projection_r) << endl;
    
        cout << dev_Ai_ui(projection_l) << endl;
        cout << dev_Ai_ui(projection_r) << endl;
        
        cout << dev_Ai_vi(projection_l) << endl;
        cout << dev_Ai_vi(projection_r) << endl;
        
        cout << "TRansposed part of Ai " << endl;
        cout << dev_AiT_ui(projection_l) << endl;
        cout << dev_AiT_ui(projection_r) << endl;
        
        cout << dev_AiT_vi(projection_l) << endl;
        cout << dev_AiT_vi(projection_r) << endl;
        
        cout << "dev_A_ui" << endl;
        cout << dev_A_ui(projection_l,projection_r,1) << endl;
        cout << dev_A_ui(projection_l,projection_r,2) << endl;
        cout << "dev_A_vi" << endl;
        cout << dev_A_vi(projection_l,projection_r,1) << endl;
        cout << dev_A_vi(projection_l,projection_r,2) << endl;
        
        cout << "dev_b_ui b" << endl;
        cout << dev_b_ui(projection_l,projection_r,1) << endl;
        cout << dev_b_ui(projection_l,projection_r,2) << endl;
        cout << "dev_b_vi b" << endl;
        cout << dev_b_vi(projection_l,projection_r,1) << endl;
        cout << dev_b_vi(projection_l,projection_r,2) << endl;
        
        cout << "dev_ATA_inv_ui" << endl; 
        cout << dev_ATA_inv_ui(A,projection_l,projection_r,1) << endl;
        cout << dev_ATA_inv_ui(A,projection_l,projection_r,2) << endl;
        cout << "dev_ATA_inv_vi" << endl;
        cout << dev_ATA_inv_vi(A,projection_l,projection_r,1) << endl;
        cout << dev_ATA_inv_vi(A,projection_l,projection_r,2) << endl;
        
        cout << "Results version Jm_i_u " << endl; 
        cout <<  Jm_i_u(A,b,projection_l,projection_r,1) << endl;
        cout <<  Jm_i_u(A,b,projection_l,projection_r,2) << endl;
        
        cout << "Results version Jm_i_v " << endl;
        cout <<  Jm_i_v(A,b,projection_l,projection_r,1) << endl;
        cout <<  Jm_i_v(A,b,projection_l,projection_r,2) << endl;
        
        cout << "Resulting J: " << endl;
        cout << J_m(A,b,projection_l, projection_r) << endl;
    }
    Mat Jm = J_m(A,b,projection_l, projection_r);
    Mat_<double> Sigma_m;
    /*Sigma_m = (Mat_<double>(4, 4) <<	4.21657513261869, -1.120145845997693, 0, 0 
                                        -1.120145845997693, 8.448339736289826, 0, 0,
                                        0, 0, 4.133011885212156, 0.586795704471397,
                                        0, 0, 0.586795704471397, 8.752700357216185);*/
    Sigma_m = (Mat_<double>(4, 4) <<	4.216, -1.120,  0,      0, 
                                        -1.120, 8.448,  0,      0,
                                        0,      0,      4.133, 0.586,
                                        0,      0,      0.586, 8.752);
    /*Sigma_m = (Mat_<double>(4, 4) <<	4.216,  0,  0,      0, 
                                        0,      8.448,  0,      0,
                                        0,      0,      4.133, 0,
                                        0,      0,      0,      8.752);*/
    /*Sigma_m = (Mat_<double>(4, 4) <<	4.21657513261869, -1.120145845997693, 0, 0 
                                        -1.120145845997693, 8.448339736289826, 0, 0,
                                        0,0,4.21657513261869, -1.120145845997693, 
                                        0,0,-1.120145845997693, 8.448339736289826);*/
    Mat wow=Jm*Sigma_m*Jm.t();
    //wow=Jm*Jm.t();
    cout << "Lovely measurement covariance" << endl;
    cout << wow << endl;
    
    
    return 0;
    
}