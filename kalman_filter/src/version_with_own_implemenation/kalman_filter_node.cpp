#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stereo_cam/point.h>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;

class kalman_filter {

    ros::NodeHandle nh_;
    ros::Subscriber measPos_sub;
    ros::Publisher estPos_pub;
  
    //cv::KalmanFilter KF;
    std::vector<Point3f> measurev, kalmanv;
    Mat_<float> meas;


    Mat_<float> F;
    Mat_<float> F_trans;
    Mat_<float> x_cur;
    Mat_<float> x_last;
    Mat_<float> P_cur;
    Mat_<float> P_last;
    Mat_<float> Qk;
    Mat_<float> Rk;
    Mat_<float> H;
    Mat_<float> H_trans;
    float processNoise , measurementNoise, dt, v, a;

public:
    kalman_filter() {
        measPos_sub = nh_.subscribe("/stereo_cam/3D_Point", 100, &kalman_filter::measCb, this);

        estPos_pub = nh_.advertise<stereo_cam::point>("/kalman_filter/predicted_pos", 100);

        processNoise = .02; 
        measurementNoise = 5;

        dt = 1.0/15.0; // should have an update rate of 15 frames per second
        v = dt;
        a = .5*dt*dt;
        //KF = KalmanFilter(9,3,0);
        F = (Mat_<float>(9, 9) <<				1, 0, 0, v, 0, 0, a, 0, 0, 
                                        0, 1, 0, 0, v, 0, 0, a, 0, 
                                        0, 0, 1, 0, 0, v, 0, 0, a, 
                                        0, 0, 0, 1, 0, 0, v, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, v, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, v, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);

        P_last = (Mat_<float>(9, 9) <<	1, 0, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 1, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 1, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 1, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);
        P_cur = (Mat_<float>(9, 9) <<		1, 0, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 1, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 1, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 1, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);
        Qk = (Mat_<float>(9, 9) <<			1, 0, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 1, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 1, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 1, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);

        Rk = (Mat_<float>(3, 3) <<			8.911315337248085e-07, 2.449382784920743e-06, -9.040569567174915e-06,
                                        7.560304891441871e-06, 2.18248866464645e-05, 0.0001601961883781097,
                                        -1.403416158123503e-05, 0.0002530436460603804, 0.00185826111431261);
        cout << Rk << endl;
        // Qk is the processNoise
        Qk=Qk*processNoise;
        // Rk is the noise for our senser
        //Rk=Rk*measurementNoise;
        x_cur= (Mat_<float>(9, 1) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
        x_last= (Mat_<float>(9, 1) << 0, 0, 2, 0, 0, 0, 0, 0, 0);
        //H = (Mat_<float>(9, 1) << 1, 1, 1, 0, 0, 0, 0, 0, 0);
        H = (Mat_<float>(3,9) <<    1, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 1, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0, 0, 0, 0);
                                    
        transpose(F,F_trans);
        transpose(H,H_trans);
        /*KF.statePost.at<float>(1) = 0;
        KF.statePost.at<float>(2) = 0;
        KF.statePost.at<float>(3) = 0;
        KF.statePost.at<float>(4) = 0;
        KF.statePost.at<float>(5) = 0;
        KF.statePost.at<float>(6) = 0;
        KF.statePost.at<float>(7) = 0;
        KF.statePost.at<float>(8) = 0;
        KF.statePost.at<float>(9) = 0;

        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov, Scalar::all(processNoise));
        setIdentity(KF.measurementNoiseCov, Scalar::all(measurementNoise));
        */
        meas = Mat_<float>(3,1);
        meas.setTo(Scalar(0));
    }
  
    void measCb(const stereo_cam::point& msg) {      
        
        stereo_cam::point out;
        // inspired from http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
        // matrix dimensions are easily given by: www.swarthmore.edu/NatSci/echeeve1/Ref/Kalman/MatrixKalman.html
        // prediction
        x_cur=F*x_last; // no external influences
        P_cur=F*P_last*F_trans+Qk; // processNoise as uncertainty
        
//         cout << "Predicted x: \n" << x_cur << endl;
//         cout << "Predicted P: \n" << P_cur << endl;
        
        // get measurement
        meas(0) = msg.x;
        meas(1) = msg.y;
        meas(2) = msg.z;
        
        if(msg.x==-1 && msg.y==-1 && msg.z==-1) // if we do not see the ball - error message - just do prediction
        {
            P_last=P_cur;
            x_last = x_cur;
            
            out.x = x_cur.at<float>(0);
            out.y = x_cur.at<float>(1);
            out.z = x_cur.at<float>(2);
        }
        else{
            Mat_<float> zk;
            zk = (Mat_<float>(3, 1) << meas(0), meas(1), meas(2));        
            // correction step
            Mat part_of_K=(H*P_cur*H_trans)+Rk;
            Mat K=(P_cur*H_trans)*part_of_K.inv();
//             cout << "Kalman Gain: \n" << K << endl;
            

            Mat x_cur_cor=x_cur+K*(zk-H*x_cur);
            Mat P_cur_cor=P_cur-K*H*P_cur;
            
//             cout << "Current measurement: \n"  << zk << endl;
//             cout << "Corrected x: \n" << x_cur_cor << endl;
//             cout << "Corrected P: \n" << P_cur_cor << endl;
            
            P_last=P_cur_cor;
            x_last = x_cur_cor;
            
            out.x = x_cur_cor.at<float>(0);
            out.y = x_cur_cor.at<float>(1);
            out.z = x_cur_cor.at<float>(2);
        }
        
        estPos_pub.publish(out);
    }   

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalman_filter");
  kalman_filter kf;
  ros::spin();
  return 0;
}
