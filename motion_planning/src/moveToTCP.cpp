#include "ros/ros.h"
#include "motion_planning/moveToQ.h"
#include "motion_planning/moveToTCP.h"


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
#include <rw/models/SerialDevice.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rw/invkin/ClosedFormIKSolverUR.hpp>

#include <rw/loaders/WorkCellLoader.hpp>

// Libs for Pathplanner
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>

#define MAXTIME 20.

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

        //rw::kinematics::State currentStatePtr = &currentState;
        // Using same collisionchecker as RWstudio, to make it possible to validate results
        colDetect = new rw::proximity::CollisionDetector(wc,rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

        // Using inverse kinematic from rw:
        rw::models::SerialDevice::Ptr sDevice = rw::common::ownedPtr( new rw::models::SerialDevice(device->getBase(), device->getEnd(), device->getName(), currentState));
        urIK = new rw::invkin::ClosedFormIKSolverUR(sDevice,currentState);
        // Check joint limit here?
        
        // Constraint planner needed for RRT
        plannerConstraint = rw::pathplanning::PlannerConstraint::make(colDetect, device, currentState);

        sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device), plannerConstraint.getQConstraintPtr());
        
        metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();

        // RRT connet planner
        plannerRRT = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(plannerConstraint, sampler, metric, extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);


    }
    /*//Maybe not needed after all
    rw::math::Q convert_to_q(rw::math::Transform3D<double> TCP)
    {
       rw::math::Q return_val((size_t) 6,(double)0.0);

       return return_val;
    }
    //*/

    // This functions uses the closedformIKsolver to find the configuration we want.
    rw::math::Q convert_to_q(double x, double y, double z)
    {
        rw::math::Vector3D<double> translation(x,y,z);
        rw::math::Rotation3D<double> current_rotation;

        // getQ from device, setQ in currentState, get rotation from the device.
        rw::math::Q currentQ = sdsip_.getQ();

        device->setQ(currentQ,currentState);

        current_rotation = device->baseTend(currentState).R();

        // base to end transform for new position, same orientation:
        // this should be done more intelligently, (3 equations with 6 unknows, select nearest)
        rw::math::Transform3D<double> transformation(translation,current_rotation);

        std::vector<rw::math::Q> solutions;
        solutions = urIK->solve(transformation, currentState);


        // Now it just returns the first solution, not the best!!!
        return solutions[0];
    }

    //Moves the device to X,y,z but keeping its position.
    bool move_to_tcp(motion_planning::moveToTCP::Request  &req,
                   motion_planning::moveToTCP::Response &res)
    {
        ROS_INFO("Moving....");

        bool return_stat;
        res.ok = 1;
        rw::math::Q goal;
        goal = convert_to_q(req.x,req.y,req.z);

        return_stat = move_device_to_q(goal);

        return return_stat;
    }

    //Moves the device to Q
    bool move_to_q(motion_planning::moveToQ::Request  &req,
                   motion_planning::moveToQ::Response &res)
    {
        ROS_INFO("Moving....");

        bool return_stat;
        res.ok = 1;
        
        rw::math::Q goal(6,req.Q[0],req.Q[1],req.Q[2],req.Q[3],req.Q[4],req.Q[5]);        
        
        return_stat = move_device_to_q(goal);

        return return_stat;

        // Get current Q from sdsip_.getQ()
    }

    // Tells the device to servo to Q
    bool move_device_to_q(rw::math::Q configuration)
    {
        bool return_stat = true;
        //device->setQ(configuration,currentState);
        
        
        // getQ from device, setQ in currentState, get rotation from the device.
        rw::math::Q currentQ = sdsip_.getQ();
        device->setQ(currentQ,currentState);
        
        ROS_INFO_STREAM("Moving from: " << currentQ);
        
        rw::trajectory::QPath path;
        
        // Find Path
        bool path_found = plannerRRT->query(currentQ,configuration,path,MAXTIME);
        
        if (path_found)
        {
            ROS_INFO_STREAM( "Path length: " << path.size() );
            for (int i = 0; i < path.size(); ++i)
            {
                ROS_INFO_STREAM(path[i]);
            }
        }
        else
            ROS_INFO("Path was NOT found.");
        
        

        if (!colDetect->inCollision(currentState))
        {
            return_stat = sdsip_.moveServoQ(configuration);
            ROS_INFO("Movement done");

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
    }


protected:
    ros::NodeHandle nodehandle_;
    caros::SerialDeviceSIProxy sdsip_;

    rw::models::WorkCell::Ptr wc;
    rw::models::Device::Ptr device;
    rw::kinematics::State  currentState;

    rw::proximity::CollisionStrategy::Ptr colStrat;
    rw::proximity::CollisionDetector::Ptr colDetect;

    rw::invkin::ClosedFormIKSolverUR::Ptr urIK;

    rw::pathplanning::QToQPlanner::Ptr planner_;
    
    // costraint planner needed for pathplanner
    rw::pathplanning::PlannerConstraint plannerConstraint;

    rw::pathplanning::QSampler::Ptr sampler;
    
    rw::math::QMetric::Ptr metric;
    
    // set pathplanner step size epsilon - value in radians
        double extend = 0.001;    
    // RRT connet planner
    rw::pathplanning::QToQPlanner::Ptr plannerRRT;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planning");
    ros::NodeHandle n;

    planner planner1;

    ros::ServiceServer service = n.advertiseService("moveToQ", &planner::move_to_q, &planner1);
    ros::ServiceServer service2 = n.advertiseService("moveToTCP", &planner::move_to_tcp, &planner1);
    ROS_INFO("Nice motionplanning ready!");
    ros::spin();

    return 0;
}

