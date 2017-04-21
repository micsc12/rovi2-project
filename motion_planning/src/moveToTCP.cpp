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
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/kinematics/State.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>


#include <rw/loaders/WorkCellLoader.hpp>

// Based on the universal_robot_test.h from caros.
class planner
{
public:

    planner() : nodehandle_("~"), sdsip_(nodehandle_, "caros_universalrobot")
    {
        ROS_INFO("This node needs a workcell located at workcell/WC3_scene.wc.xml");

        wc = rw::loaders::WorkCellLoader::Factory::load("workcell/WC3_Scene.wc.xml");
        device = wc->findDevice("UR1");

        currentState = wc->getDefaultState();

        // Using same collisionchecker as RWstudio, to make it possible to validate results
        colDetect = new rw::proximity::CollisionDetector(wc,rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());


    }

    bool move_to_q(motion_planning::moveToQ::Request  &req,
                   motion_planning::moveToQ::Response &res)
    {
        ROS_INFO("Moving....");

        bool return_stat = true;
        res.ok = 1;
        rw::math::Q goal(6,req.Q[0],req.Q[1],req.Q[2],req.Q[3],req.Q[4],req.Q[5]);
        device->setQ(goal,currentState);

        if (!colDetect->inCollision(currentState))
        {
            return_stat = sdsip_.moveServoQ(goal);
            ROS_INFO("Movement done");// + std::to_string(req.Q[1]));

            if (!return_stat)
            {
                return_stat = false;
                ROS_ERROR_STREAM("The serial device didn't acknowledge the moveServoQ command.");
            }
        }
        else
        {
            ROS_INFO("Goal is in collision!");
            return_stat = false;
        }
        return return_stat;

        // Get current Q from sdsip_.getQ()
    }

protected:
    ros::NodeHandle nodehandle_;
    caros::SerialDeviceSIProxy sdsip_;

    rw::models::WorkCell::Ptr wc;
    rw::models::Device::Ptr device;
    rw::kinematics::State  currentState;

    rw::proximity::CollisionStrategy::Ptr colStrat;
    rw::proximity::CollisionDetector::Ptr colDetect;
    rw::pathplanning::QToQPlanner::Ptr planner_;


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planning");
    ros::NodeHandle n;

    planner planner1;

    ros::ServiceServer service = n.advertiseService("moveToQ", &planner::move_to_q, &planner1);
    ROS_INFO("Nice motionplanning ready!");
    ros::spin();

    return 0;
}

