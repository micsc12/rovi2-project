#include "ros/ros.h"
#include "motion_planning/moveToQ.h"
#include "motion_planning/moveToTCP.h"
#include "motion_planning/moveToHome.h"

#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>
#include <caros_universalrobot/UrServiceServoQ.h>

#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <cmath>


#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
//#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/proximity/CollisionDetector.hpp>
//#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/SerialDevice.hpp>

#include <rw/math/Jacobian.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rw/invkin/ClosedFormIKSolverUR.hpp>

#include <rw/loaders/WorkCellLoader.hpp>


// Libs for Pathplanner
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>


using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;

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

        sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device),
                                                              plannerConstraint.getQConstraintPtr());

        // Using standard metric
        metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();

        // RRT connet planner
        //plannerRRT = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(plannerConstraint, sampler, metric, extend, rwlibs::pathplanners::RRTPlanner::RRTConnect);

        // Use standard params:
        plannerRRT = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(plannerConstraint, device,rwlibs::pathplanners::RRTPlanner::RRTConnect);



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
        rw::math::Rotation3D<double> current_rotation,roll,pitch,yaw;

        // getQ from device, setQ in currentState, get rotation from the device.
        rw::math::Q currentQ = sdsip_.getQ();

        device->setQ(currentQ,currentState);




        // base to end transform for new position, same orientation:
        // this should be done more intelligently, (3 equations with 6 unknows, select nearest)


        std::vector<rw::math::Q> solutions,new_solutions;
        // Get solutions for all possibble orientations:
        for (int x = 0; x < 200; x+=20)
        {
            yaw = getXrotation(x);
            for (int y = 0; y < 200; y+=20)
            {
                pitch = getYrotation(y);
                for (int z = 0; z < 200; z+=20)
                {
                    roll = getZrotation(z);

                    current_rotation = roll*pitch*yaw;
                    rw::math::Transform3D<double> transformation(translation,current_rotation);

                    new_solutions = urIK->solve(transformation, currentState);
                    solutions.insert(solutions.end(),new_solutions.begin(),new_solutions.end());
                }
            }
        }

        // If no solutions exist, the service failed:
        if(solutions.size() == 0)
        {
            rw::math::Q ret(6,0.0,0.0,0.0,0.0,0.0,0.0);
            return ret;
        }

        // Determine the best of the found solutions:
        int bestsolution = 0;
        double max_dist = 10000;
        rw::math::Q difference;

        ROS_INFO_STREAM(solutions.size() << " solutions found. Selecting the best one!");


        for (int i = 0; i < solutions.size(); i++)
        {
            //ROS_INFO_STREAM("checking solutions" << solutions[i][5]);
            difference = currentQ - solutions[i];


            if (difference.norm2() < max_dist)
            {
                bestsolution = i;
                max_dist = difference.norm2();
            }
        }
        //solutions[bestsolution][5] = 0;


        // Now it just returns the first solution, not the best!!!
        // If no solution exists, the node crashes at the moment. This needs to be fixed!
        return solutions[bestsolution];
    }


    // This functions uses the closedformIKsolver to find the configuration we want.
    /*rw::math::Q convert_to_q2(double x, double y, double z)
    {
        rw::math::Vector3D<double> translation(x,y,z);
        rw::math::Rotation3D<double> current_rotation;

        // getQ from device, setQ in currentState, get rotation from the device.
        rw::math::Q currentQ = sdsip_.getQ();

        device->setQ(currentQ,currentState);

        rw::math::Jacobian full_jacobian = device->baseJend(currentState);

        rw::math::Jacobian jacobian(3,6);
        jacobian.e() = full_jacobian.e().block(0,0,3,6);


        rw::math::Q solution(6);
        solution.e() = jacobian.e().transpose()*(jacobian.e()*jacobian.e().transpose()).inverse()*translation.e();



        // base to end transform for new position, same orientation:
        // this should be done more intelligently, (3 equations with 6 unknows, select nearest)
        rw::math::Transform3D<double> transformation(translation,current_rotation);

        std::vector<rw::math::Q> solutions;
        solutions = urIK->solve(transformation, currentState);

        // Determine the best of the found solutions:
        int bestsolution = 0;
        double max_dist = 10000;
        rw::math::Q difference;


        for (int i = 0; i < solutions.size(); i++)
        {
            ROS_INFO_STREAM("checking solutions" << solutions[i][5]);
            difference = currentQ - solutions[i];

            difference[5] = 0;
            if (difference.norm2() < max_dist)
            {
                bestsolution = i;
                max_dist = difference.norm2();
            }
        }
        solutions[bestsolution][5] = 0;


        // Now it just returns the first solution, not the best!!!
        // If no solution exists, the node crashes at the moment. This needs to be fixed!
        return solution;
    }
    //*/


    rw::math::Rotation3D<double> getXrotation(double angle)
    {
        rw::math::Rotation3D<double> ret_val;
        ret_val(0,0) = 1;
        ret_val(0,1) = 0;
        ret_val(0,2) = 0;

        ret_val(1,0) = 0;
        ret_val(1,1) = cos(angle/180 * M_PI);
        ret_val(1,2) = -sin(angle/180 * M_PI);

        ret_val(2,0) = 0;
        ret_val(2,1) = sin(angle/180 * M_PI);
        ret_val(2,2) = cos(angle/180 * M_PI);
    }
    rw::math::Rotation3D<double> getYrotation(double angle)
    {
        rw::math::Rotation3D<double> ret_val;
        ret_val(0,0) = cos(angle/180 * M_PI);
        ret_val(0,1) = 0;
        ret_val(0,2) = sin(angle/180 * M_PI);;

        ret_val(1,0) = 0;
        ret_val(1,1) = 1;
        ret_val(1,2) = 0;

        ret_val(2,0) = -sin(angle/180 * M_PI);
        ret_val(2,1) = 0;
        ret_val(2,2) = cos(angle/180 * M_PI);
    }
    rw::math::Rotation3D<double> getZrotation(double angle)
    {
        rw::math::Rotation3D<double> ret_val;
        ret_val(0,0) = cos(angle/180 * M_PI);
        ret_val(0,1) = -sin(angle/180 * M_PI);
        ret_val(0,2) = 0;

        ret_val(1,0) = sin(angle/180 * M_PI);
        ret_val(1,1) = cos(angle/180 * M_PI);
        ret_val(1,2) = 0;

        ret_val(2,0) = 0;
        ret_val(2,1) = 0;
        ret_val(2,2) = 1;
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
    bool move_to_home(motion_planning::moveToHome::Request  &req,
                      motion_planning::moveToHome::Response &res) //motion_planning::moveToHome::Request  &req,
    {


        bool return_stat;
        res.ok = 1;
        rw::math::Q goal(6);
        goal[0] = 0.0;
        goal[1] = -1.57;
        goal[2] = 0.0;
        goal[3] = -1.57;
        goal[4] = 0.0;
        goal[5] = 0.0;


        return_stat = move_device_to_q(goal);

        return return_stat;


    }

    // Tells the device to servo to Q
    bool move_device_to_q(rw::math::Q configuration)
    {
        bool return_stat = true;

        // getQ from device, setQ in currentState,
        rw::math::Q currentQ = sdsip_.getQ();
        device->setQ(currentQ,currentState);




        ROS_INFO_STREAM("Moving from: " << currentQ);

        rw::trajectory::QPath path;

        // Find Path
        bool path_found = plannerRRT->query(currentQ,configuration,path);//,MAXTIME);

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


        for (int i = 0; i < path.size(); ++i)
        {
            return_stat = sdsip_.moveServoQ(path[i]);
            ROS_INFO("Movement done");
            ros::Duration(5).sleep();


        }

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
    ros::ServiceServer service3 = n.advertiseService("moveToHome", &planner::move_to_home, &planner1);
    ROS_INFO("Nice motionplanning ready!");
    ros::spin();

    return 0;
}

