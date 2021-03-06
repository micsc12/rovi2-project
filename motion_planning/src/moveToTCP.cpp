#include "ros/ros.h"
#include "motion_planning/moveToQ.h"
#include "motion_planning/moveToTCP.h"
#include "motion_planning/moveToHome.h"

#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>
#include <caros_universalrobot/UrServiceServoQ.h>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <string>
#include <cmath>
#include <fstream>      // std::ofstream


#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
//#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/proximity/CollisionDetector.hpp>
//#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/SerialDevice.hpp>

// ---------------------------------
#include <rw/math/Transform3D.hpp>
//#include <rws/RobWorkStudio.hpp>
#include <rw/kinematics/MovableFrame.hpp>
// ---------------------------------

#include <rw/math/Jacobian.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>

#include <rw/loaders/WorkCellLoader.hpp>


// Libs for Pathplanner
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>

#include <math.h>     /* fabs */


using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;

// Based on the universal_robot_test.h from caros.
class planner
{
public:

    planner() : nodehandle_("~"), sdsip_(nodehandle_, "caros_universalrobot")
    {
        ROS_INFO("This node needs a workcell located at workcell/WC3_scene.wc.xml");
        ROS_INFO("My good sir, please make sure that you are running this node from catkin workspace");

        wc = rw::loaders::WorkCellLoader::Factory::load("/home/petr/catkin_ws/workcell/WC3_Scene.wc.xml");
        if (wc == NULL) 
        {
            ROS_INFO(" Workcell not found.");
        }
        
        device = wc->findDevice("UR1");
        if (device == NULL) 
        {
            ROS_INFO(" Device not found.");
        }

        currentState = wc->getDefaultState();
        // find frame of TCP
        TCP_Frame = wc->findFrame("UR1.TCP");
        if (TCP_Frame == NULL) 
        {
            ROS_INFO(" TCP Frame not found.");
        }
        
        // find frame of the marker
        Marker_Frame = wc->findFrame("WSG50.MarkerFrame");
        if (Marker_Frame == NULL) 
        {
            ROS_INFO(" Marker Frame not found.");
        }
        
        Ball_Frame = wc->findFrame("BallFrame");
        if (Ball_Frame == NULL) 
        {
            ROS_INFO(" Ball Frame not found.");
        }
        
        grasp = nodehandle_.advertise<std_msgs::Bool>("/motion_planning/grasp", 1);


        // Using same collisionchecker as RWstudio, to make it possible to validate results
        colDetect = new rw::proximity::CollisionDetector(wc,rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

        // Using inverse kinematic from rw:
        rw::models::SerialDevice::Ptr sDevice = rw::common::ownedPtr( new rw::models::SerialDevice(device->getBase(), device->getEnd(), device->getName(), currentState));

        // Using jacobian IK solver, because it minimizes change in Q
        urIK = new rw::invkin::JacobianIKSolver(sDevice, Ball_Frame, currentState);

        // Enable clamping (so IK only returns solutions within joint limits)
        urIK->setCheckJointLimits(true);



        // Constraint planner needed for RRT
        plannerConstraint = rw::pathplanning::PlannerConstraint::make(colDetect, device, currentState);

        // Use standard params:
        plannerRRT = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(plannerConstraint, device,rwlibs::pathplanners::RRTPlanner::RRTConnect);

        ofs.open("Calibration_camera_robot.txt", std::ofstream::out);
        ofsQ.open("Calibration_camera_robot_Q.txt", std::ofstream::out);

    }
    
    // This function is used for moving robot around the cell while keeping constant orientation of TCP pointing towards camera 
    rw::math::Q convert_to_q_calibration(double x, double y, double z)
    {
        rw::math::Vector3D<double> translation(x,y,z);
        rw::math::RPY<double> rotation(-0.406, 0.038, -1.036);
        
        rw::math::Transform3D<double> transformation(translation,rotation);

        // getQ from device, setQ in currentState, get rotation from the device.
        rw::math::Q currentQ = sdsip_.getQ();

        device->setQ(currentQ,currentState);
        
        std::vector<rw::math::Q> solutions = urIK->solve(transformation, currentState);
        
        
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

        ROS_INFO_STREAM(solutions.size() << " solutions found for IK. Selecting the best one!");

        // Check norm2 length of solutions in q-space.
        for (int i = 0; i < solutions.size(); i++)
        {

            difference = currentQ - solutions[i];
            if (difference.norm2() < max_dist)
            {
                bestsolution = i;
                max_dist = difference.norm2();
            }
        }

        // Return the best invkin solution:
        return solutions[bestsolution];
    }
    
    // This functions uses the closedformIKsolver to find the configuration we want.
    rw::math::Q convert_to_q(double x, double y, double z)
    {
        rw::math::Vector3D<double> translation(x,y,z);
        rw::math::RPY<double> rotation(3.14, 0, -0.8);   //rotation(0, -1.5, 1.5);

        // getQ from device, setQ in currentState, get rotation from the device.
        rw::math::Q currentQ = sdsip_.getQ();

        device->setQ(currentQ,currentState);        


        std::vector<rw::math::Q> solutions; //,new_solutions;
            
        rw::math::Transform3D<double> transformation(translation,rotation);
        solutions = urIK->solve(transformation, currentState);

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

        ROS_INFO_STREAM(solutions.size() << " solutions found for IK. Selecting the best one!");


        // Check norm2 length of solutions in q-space.
        for (int i = 0; i < solutions.size(); i++)
        {

            difference = currentQ - solutions[i];
            if (difference.norm2() < max_dist)
            {
                bestsolution = i;
                max_dist = difference.norm2();
            }
        }

        // Return the best invkin solution:
        return solutions[bestsolution];
    }


    //Moves the device to X,y,z but keeping its orientation.
    bool move_to_tcp(motion_planning::moveToTCP::Request  &req,
                     motion_planning::moveToTCP::Response &res)
    {
        //target position = last detection of red ball position
        rw::math::Vector3D<double> targetPos(req.x, req.y, req.z);
        
        //update current state of device in model
        rw::math::Q currentQ = get_current_Q();
        device->setQ(currentQ,currentState);
        // find current tranformation for the Ball_Frame
        auto baseTball_frame = device->baseTframe(Ball_Frame, currentState);
        // get only current position
        rw::math::Vector3D<double> currPos = baseTball_frame.P();
        
        // compute 2 norm of error from target position
        auto err = (currPos - targetPos);
        double err2norm = err.norm2();
        
        ROS_INFO_STREAM("2-norm of position error: " << err2norm);
        
        std_msgs::Bool msg_grasp;
        msg_grasp.data = false;
        // if the error is low enough GRASP red ball
        if (err2norm < 0.04)
        {
            //ROS_INFO_STREAM("GRASP TRUE");
            msg_grasp.data = true;
        }
        
        // publish do_grasp on topic
        grasp.publish(msg_grasp);
        
        
        ROS_INFO_STREAM("Calling Inverse kinematics ..." << std::endl);

        bool return_stat;
        res.ok = 1;
        rw::math::Q goal;
        //Debug
        ROS_INFO_STREAM(req.x << " " << req.y << " " << req.z);

        
        goal = convert_to_q(req.x,req.y,req.z);
        
        return_stat = move_device_to_q(goal);

        return return_stat;
    }
    
    //Moves the device to X,y,z but keeping its orientation towards the camera.
    bool move_to_tcp_calibration(motion_planning::moveToTCP::Request  &req,
                     motion_planning::moveToTCP::Response &res)
    {
        
        ROS_INFO("Calling Path planner ...");

        bool return_stat;
        res.ok = 1;
        rw::math::Q goal;
        
        // for calibration keep TCP pointing to the camera
        goal = convert_to_q_calibration(req.x,req.y,req.z);
        
        return_stat = move_device_to_q(goal, true);

        return return_stat;
    }

    //Moves the device to Q
    bool move_to_q(motion_planning::moveToQ::Request  &req,
                   motion_planning::moveToQ::Response &res)
    {
        ROS_INFO("Calling Path planner ...");

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
    bool move_device_to_q(rw::math::Q configuration, bool calibration = false)
    {
        bool return_stat = true;

        // Get current Q from device
        rw::math::Q currentQ = get_current_Q();
        device->setQ(currentQ,currentState);

        rw::trajectory::QPath path;

        // Find Path
        bool path_found = plannerRRT->query(currentQ,configuration,path);//,MAXTIME);

        // If a path exists:
        if (path_found)
        {
            ROS_INFO_STREAM("Path length: " << path.size() );
            
            ROS_INFO_STREAM("Moving from: " << currentQ);
            
            rw::common::Timer t;
            for (int i = 1; i < path.size(); ++i)
            {
                ROS_INFO_STREAM("Moving to configuration " << i <<" : " << path[i] << " ...");


                if (calibration)
                {
                return_stat = sdsip_.moveServoQ(path[i]);
                
                // Make sure, that new messages from robot are current
                wait_for_current_messages();
                
                // for some reason isMovin() doesn't work to wait until movement is finished
                //t.resetAndResume();
                wait_to_finish_move(path[i]);
                // Print out last Q error from path, before changing targetQ configuration
                //ROS_INFO_STREAM("Final error from path[" << i << "] configuration: " << error_from_target(path[i]) );
                //t.pause();
                }
                
                else
                {
                    return_stat = sdsip_.moveServoQ(path[i]); // Petrs
                }

                
            }
            
            ROS_INFO("Movement done");
            if (calibration)
            {
                ROS_INFO("CALIBRATION: Waiting for robot to COMPLETELY finish move ...");
                // for camera calibration -> Wait until movement is COMPLETELY finished - 10s to be on safe side
                ros::Duration(10).sleep();
                ros::spinOnce();
                
                // Get current Q configuration
                currentQ = get_current_Q();
                device->setQ(currentQ, currentState);
                ROS_INFO_STREAM("Q after move: " << currentQ);
                // save it to file
                ofsQ << currentQ << std::endl;
                
                // Get transformation from base to TCP frame 
                auto baseTtcp = device->baseTframe(TCP_Frame, currentState);
                ROS_INFO_STREAM("TCP position in Robot base frame[x, y, z]: " << baseTtcp.P());
                
                // Compute transform from base to marker on the TCP
                rw::math::Transform3D<double> baseTmarker = device->baseTframe(Marker_Frame, currentState);
                // Get marker 3D position
                rw::math::Vector3D<double> markerP = baseTmarker.P();
                ROS_INFO_STREAM("Marker position [x, y, z]: " << markerP);
                
                ofs << markerP << std::endl;
            }
            
        }
        else
        {
            ROS_INFO("Path was NOT found.");
            
            rw::kinematics::State goalState = wc->getDefaultState(); 

            //TODO: THIS SEEMS WRONG -> IT does not check the goal Q, but the start Q
            device->setQ(currentQ,goalState);
            if (colDetect->inCollision(goalState))
                ROS_INFO("Goal is in collision!");
                
            return_stat = false;
        }

        return return_stat;
    }


protected:
    ros::NodeHandle nodehandle_;
    caros::SerialDeviceSIProxy sdsip_;
    ros::Publisher grasp;

    rw::models::WorkCell::Ptr wc;
    rw::models::Device::Ptr device;
    rw::kinematics::State  currentState;
    rw::kinematics::Frame* TCP_Frame;
    rw::kinematics::Frame* Marker_Frame;
    rw::kinematics::Frame* Ball_Frame;

    rw::proximity::CollisionStrategy::Ptr colStrat;
    rw::proximity::CollisionDetector::Ptr colDetect;

    //rw::invkin::ClosedFormIKSolverUR::Ptr urIK;
    rw::invkin::JacobianIKSolver::Ptr urIK;


    // costraint planner needed for pathplanner
    rw::pathplanning::PlannerConstraint plannerConstraint;

    rw::pathplanning::QSampler::Ptr sampler;

    rw::math::QMetric::Ptr metric;

    // set pathplanner step size epsilon - value in radians
    double extend = 0.1;
    // RRT connet planner
    rw::pathplanning::QToQPlanner::Ptr plannerRRT;
    
    // Variables for storing last 2 maximum joint velocities 
    double previous_max_Abs = 0;
    double max_Abs_speed = 0;
    
    std::ofstream ofs;
    std::ofstream ofsQ;
    
    bool wait_for_current_messages()
    {
        // Make sure, that new messages from robot are current
        ros::Time current_timestamp = ros::Time::now();
        ros::Time obtained_timestamp = sdsip_.getTimeStamp();
        
        // Wait here, until timestamp received from robot is newer
        // than current local timestamp
        while (current_timestamp > obtained_timestamp)
        {
            ros::Duration(0.1).sleep();  // In seconds
            ros::spinOnce();
            obtained_timestamp = sdsip_.getTimeStamp();
            //ROS_INFO("Waiting for current message...");
        }
        
        return true;
        
        //ToDo
        // false response when unable to get current message
    }
    
    rw::math::Q get_current_Q()
    {
        wait_for_current_messages();
        // getQ from device
        return sdsip_.getQ();
    }
    
    rw::math::Q get_current_Q_vel()
    {
        wait_for_current_messages();
        rw::math::Q currentQ_vel = sdsip_.getQd();
        //ROS_INFO_STREAM("Current joint velocity: " << currentQ_vel);
        
        return currentQ_vel;
    }
    
    rw::math::Q error_from_target(const rw::math::Q &targetQ)
    {        
        rw::math::Q currentQ = get_current_Q();
        //ROS_INFO_STREAM("Current joint velocity: " << currentQ_vel);
        
        rw::math::Q error_Q = targetQ - currentQ;
        
        // Get ABS of errors
        rw::math::Q abs_error_Q = error_Q;  //dummy init of Q vector needed for correct size
        for (int i = 0; i < error_Q.size(); ++i)
        {
            abs_error_Q[i] = fabs(error_Q[i]);
        }
        
        return abs_error_Q;
    }
    
    bool approaching_goal_configuration()
    {
        previous_max_Abs = max_Abs_speed;
        max_Abs_speed = 0;
        
        rw::math::Q currentQ_vel = get_current_Q_vel();
        //ROS_INFO_STREAM("Current joint velocity: " << currentQ_vel);
        
        for (int i = 0; i < currentQ_vel.size(); ++i)
        {
            if (max_Abs_speed < fabs(currentQ_vel[i]) )
            {
                max_Abs_speed = fabs(currentQ_vel[i]);
            }
        }
        
        //ROS_INFO_STREAM(max_Abs_speed << "  " << currentQ_vel[0] << "   " << abs(currentQ_vel[0]));
        
        // check if the maximum speed is low enough and that we are slowing down (max_Abs_speed < previous_max_Abs)
        if ( (max_Abs_speed <= previous_max_Abs) && (max_Abs_speed < 0.5) )
            return true;
        else
            return false;
    }
    
    void wait_to_finish_move(const rw::math::Q &targetQ)
    {
        while(!approaching_goal_configuration(targetQ)) 
        {
            // Make sure, that new messages from robot are current
            wait_for_current_messages();  
            //get_current_Q_vel();
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }
    
    bool approaching_goal_configuration(const rw::math::Q &targetQ)
    {       
        rw::math::Q error_Q = error_from_target(targetQ);
        //ROS_INFO_STREAM("Error from target Q: " << error_Q);
        
        // check each joint for error from target
        for (int i = 0; i < error_Q.size(); ++i)
        {
            double max_error_allowed = 0.1;
            if (error_Q[i] > max_error_allowed )
            {
                // Any of joints breaches max err allowed -> we are not finished yet
                return false;
            }
        }
        
        // close to targetQ -> call speed check functions
        // function overloaded by no arguments
        return approaching_goal_configuration();
        
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planning");
    ros::NodeHandle n;

    planner planner1;

    ros::ServiceServer service = n.advertiseService("moveToQ", &planner::move_to_q, &planner1);
    ros::ServiceServer service2 = n.advertiseService("moveToTCP", &planner::move_to_tcp, &planner1);
    ros::ServiceServer service3 = n.advertiseService("moveToHome", &planner::move_to_home, &planner1);
    ros::ServiceServer service4 = n.advertiseService("moveToTCPcalibration", &planner::move_to_tcp_calibration, &planner1);
    
    ROS_INFO("Nice motionplanning ready!");
    ros::spin();

    return 0;
}

