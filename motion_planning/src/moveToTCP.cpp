#include "ros/ros.h"
#include "motion_planning/moveToQ.h"

#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>
#include <caros_universalrobot/UrServiceServoQ.h>

#include "std_msgs/String.h"
#include <sstream>
#include <string>


#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/proximity/CollisionDetector.hpp>

// Based on the universal_robot_test.h from caros.
class planner
{
public:

    planner(ros::NodeHandle nh) : nodehandle_("~"), sdsip_(nodehandle_, "caros_universalrobot")
    {
        //nodehandle_ = nh;
        //sdsip_(nh);

       /* // Initialize things from caros/robwork
        workcell_ = caros::getWorkCell();
        if (workcell_ == NULL)
        {
            ROS_ERROR("No workcell was loaded - exiting...");
            throw std::runtime_error("Not able to obtain a workcell.");
        }

        std::string device_name = "UR1"; // TODO change this to find robot name on parameter server


        ROS_DEBUG_STREAM("Looking for the device '" << device_name << "' in the workcell.");
        device_ = workcell_->findDevice(device_name);
        if (device_ == NULL)
        {
          ROS_FATAL_STREAM("Unable to find device " << device_name << " in the loaded workcell");
          throw std::runtime_error("Not able to find the device within the workcell.");
        }
*/
    }

    bool move_to_q(motion_planning::moveToQ::Request  &req,
                   motion_planning::moveToQ::Response &res)
    {
        ROS_INFO("Moving....");
        bool return_stat = true;
        //req.Q[0] = 1;
        res.ok = 1;
        rw::math::Q goal(6,req.Q[0],req.Q[1],req.Q[2],req.Q[3],req.Q[4],req.Q[5]);
        
	return_stat = sdsip_.moveServoQ(goal);
        ROS_INFO("Movement done");// + std::to_string(req.Q[1]));

        if (!return_stat)
        {
            return_stat = false;
            ROS_ERROR_STREAM("The serial device didn't acknowledge the moveServoQ command.");
        }
        return return_stat;
    }

/*
    bool move_to_q2(motion_planning::moveToQ::Request  &req,
                   motion_planning::moveToQ::Response &res)
    {
        ROS_INFO("Moving....");
        ros::ServiceClient client = n.serviceClient<caros_universalrobot::servo_q>("servo_q");
        caros_universalrobot::servo_q srv;
        srv.q

        bool return_stat = true;
        req.Q[0] = 1;
        res.ok = 1;
        rw::math::Q goal(6,req.Q[0],req.Q[1],req.Q[2],req.Q[3],req.Q[4],req.Q[5]);
        return_stat = sdsip_.moveServoQ(goal);
        ROS_INFO("Movement done");// + std::to_string(req.Q[1]));

        if (!return_stat)
        {
            return_stat = false;
            ROS_ERROR_STREAM("The serial device didn't acknowledge the moveServoQ command.");
        }
        return return_stat;
    }
*/


protected:
    ros::NodeHandle nodehandle_;
    caros::SerialDeviceSIProxy sdsip_;

    rw::models::WorkCell::Ptr workcell_;
    rw::models::Device::Ptr device_;
    rw::pathplanning::QToQPlanner::Ptr planner_;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planning");
    ros::NodeHandle n;

    planner planner1(n);

    ros::ServiceServer service = n.advertiseService("moveToQ", &planner::move_to_q, &planner1);
    ROS_INFO("Nice motionplanning ready!");
    ros::spin();

    return 0;
}

