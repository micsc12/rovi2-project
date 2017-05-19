#include <ros/ros.h>
#include <wsg_50_common/Move.h>
//#include <wsg_50_common/MoveRequest.h>
#include <std_msgs/Bool.h>
#include <stdio.h>

using namespace std;

class gripper_ctrl {

  ros::NodeHandle nh_;
	ros::ServiceClient graspSC;	
	ros::ServiceClient releaseSC;	
	wsg_50_common::Move grasp_srv;
	wsg_50_common::Move release_srv;
  ros::Subscriber grasp_sub;
  bool old_grasp;
	
public :
	gripper_ctrl() { 
		graspSC = nh_.serviceClient<wsg_50_common::Move>("wsg_50_driver/grasp");
		releaseSC = nh_.serviceClient<wsg_50_common::Move>("wsg_50_driver/release");
  	grasp_sub = nh_.subscribe("/motion_planning/grasp", 1, &gripper_ctrl::graspCb, this);
  	grasp_srv.request.width = 65;
  	grasp_srv.request.speed = 100;
  	release_srv.request.width = 110;
  	release_srv.request.speed = 100;
  	old_grasp = 0;
	}
	void graspCb(const std_msgs::Bool::ConstPtr& msg) {
	
		if (msg->data && !old_grasp) {
			//cout << "[gripper_ctrl_node][INFO] Received grasp command." << endl;
			graspSC.call(grasp_srv);
			//cout << "[gripper_ctrl_node][INFO] Attempted to call grasp service." << endl;
		}
		else if (!msg->data) {
			releaseSC.call(release_srv);
		}
		old_grasp = msg->data;
	}
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_ctrl");
  gripper_ctrl gc;
  ros::spin();
  return 0;
}
