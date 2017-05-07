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
  
	cv::KalmanFilter KF;
	std::vector<Point3f> measurev, kalmanv;
	Mat_<float> meas;

	float processNoise , measurementNoise, dt, v, a;

public:
  kalman_filter() {
  	measPos_sub = nh_.subscribe("stereo_cam/3D_Point", 100, &kalman_filter::measCb, this);
  	
    estPos_pub = nh_.advertise<stereo_cam::point>("/kalman_filter/predicted_pos", 100);
    
		processNoise = .001;
		measurementNoise = 10;
		
		dt = .1;
		v = dt;
		a = .5*dt*dt;
    KF = KalmanFilter(9,3,0);
		KF.transitionMatrix = (Mat_<float>(9, 9) <<	1, 0, 0, v, 0, 0, a, 0, 0, 
																								0, 1, 0, 0, v, 0, 0, a, 0, 
																								0, 0, 1, 0, 0, v, 0, 0, a, 
																								0, 0, 0, 1, 0, 0, v, 0, 0, 
																								0, 0, 0, 0, 1, 0, 0, v, 0, 
																								0, 0, 0, 0, 0, 1, 0, 0, v, 
																								0, 0, 0, 0, 0, 0, 1, 0, 0, 
																								0, 0, 0, 0, 0, 0, 0, 1, 0, 
																								0, 0, 0, 0, 0, 0, 0, 0, 1);
																								
  	KF.statePost.at<float>(0) = 0;
		KF.statePost.at<float>(1) = 0;
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
		
		meas = Mat_<float>(3,1);
		meas.setTo(Scalar(0));
  }
  
  void measCb(const stereo_cam::point& msg) {
  
  	Mat pred = KF.predict();
		Point3f predictPt(pred.at<float>(0), pred.at<float>(1), pred.at<float>(2));
		kalmanv.push_back(predictPt);
		
		meas(0) = msg.x;
		meas(1) = msg.y;
		meas(2) = msg.z;
		Point3f measPt(meas(0), meas(1), meas(2));
		measurev.push_back(measPt);
		
		Mat est = KF.correct(meas);
		
		stereo_cam::point out;
		out.x = est.at<float>(0);
		out.y = est.at<float>(1);
		out.z = est.at<float>(2);

		Mat preCov = KF.errorCovPre;
		cout << "Prior Error Covariance:" << endl << preCov << endl << endl;		
		Mat postCov = KF.errorCovPost;
		cout << "Posterior Error Covariance:" << endl << postCov << endl << endl;
		
		//for (size_t i = 0, i < 9, i++) {
			//for (size_t j = 0, j < 9, j++) {
				//cout << postCov.at<float>()
			//}
		//}
		
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
