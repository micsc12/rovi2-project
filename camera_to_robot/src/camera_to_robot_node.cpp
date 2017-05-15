#include <ros/ros.h>
#include <rw/rw.hpp>
//#include <motion_planning/moveToTCP.h>
#include <stereo_cam/point.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "action_motion_planning/Target_PositionAction.h"

//using namespace cv;
using namespace std;

class camera_to_robot
{

    ros::NodeHandle nh_;
    ros::Subscriber sub_pos_in_cam_frame;
    
    //ros::ServiceClient  client_set_target_Pt;
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<action_motion_planning::Target_PositionAction> ac;
    ros::Publisher estPos_pub;
    
    rw::math::Transform3D<double> baseTcamera;
    
public:
    camera_to_robot() : ac( nh_, "/action_motion_planner", true)
    {
        sub_pos_in_cam_frame = nh_.subscribe("/throttled_pos", 100, &camera_to_robot::convert, this);
        
        //client_set_target_Pt = nh_.serviceClient<motion_planning::moveToTCP>("moveToTCP");
        
        // create the action client
        // true causes the client to spin its own thread
        //ac(actionlib::SimpleActionClient<action_motion_planning::Target_PositionAction>("action_motion_planner", true));
        
        rw::math::Vector3D<double> translation (0.601006, 0.60059, 1.35784);
        
        rw::math::Rotation3D<double> rotation (-0.932767, 0.165071, -0.320464,\
                                        0.359333, 0.496624, -0.790091,\
                                        0.0287291, -0.852125, -0.52255);

        baseTcamera = rw::math::Transform3D<double> (translation, rotation);

        
        ROS_INFO("Waiting for action server to start.");
        // wait for the action server to start
        ac.waitForServer(); //will wait for infinite time
        
        ROS_INFO("Action server started, sending goal.");
        //estPos_pub = nh_.advertise<stereo_cam::point>("/kalman_filter/predicted_pos", 100);
        
    }
    
    
    void convert(const stereo_cam::point& msg) {      
        //motion_planning:: moveToTCP srv;
        action_motion_planning::Target_PositionGoal goal;
        
        rw::math::Vector3D<double> p_Cam(msg.x/1000.0, msg.y/1000.0, msg.z/1000.0);
        /*
        p_Cam.push_back(msg.x);
        p_Cam.push_back(msg.y);
        p_Cam.push_back(msg.z);
        */
        
        //DEBUG Output
        //std::cout << p_Cam[0] << " " << p_Cam[1] << " " << p_Cam[2] << std::endl;
        
        rw::math::Vector3D<double> p_Rob_Base = baseTcamera * p_Cam;
        
        
        
        goal.x = (p_Rob_Base[0]);
        goal.y = (p_Rob_Base[1]);
        goal.z = (p_Rob_Base[2]);
        
        //DEBUG Output
        //ROS_INFO_STREAM(goal.x << " " << goal.y << " " << goal.z);
        
        ac.sendGoal(goal);
        
        /*
        if (client_set_target_Pt.call(srv))
        {
            ROS_INFO("Ok: %ld", (long int)srv.response.ok);
        }
        else
        {
            ROS_ERROR("Failed to call service moveToTCP");
        }
        */
        
    }   

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_to_robot_conversion");
  camera_to_robot cTr_convertor;

  ros::spin();
  return 0;
}