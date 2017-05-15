/*
 * ROS includes
 */ 
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "action_motion_planning/Target_PositionAction.h"

#include "motion_planning/moveToQ.h"
#include "motion_planning/moveToTCP.h"
#include "motion_planning/moveToHome.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

/*
 * CAROS includes
 */
#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>
#include <caros_universalrobot/UrServiceServoQ.h>

/*
 * ROBWORK includes
 */
#include <rw/rw.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/math/Jacobian.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

// Libs for Pathplanner
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>

/*
 * GENERAL CPP includes
 */
#include <sstream>
#include <string>
#include <cmath>
#include <fstream>      // std::ofstream
#include <math.h>     /* fabs */

using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;


// Based on the universal_robot_test.h from caros.
class planner
{
public:

    planner() : nodehandle_("~"), sdsip_(nodehandle_, "caros_universalrobot"), as_(nodehandle_, "/action_motion_planner", boost::bind(&planner::move_to_tcp, this, _1), false),
        action_name_("/action_motion_planner")
    {
        
    
        as_.start();
            
        ROS_INFO("This node needs a workcell located at workcell/WC3_scene.wc.xml");
        ROS_INFO("Check that you are running this node from catkin workspace Idiot !!!");

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
        
        //Base_Frame = wc->findFrame("Base");

        //rw::kinematics::State currentStatePtr = &currentState;
        // Using same collisionchecker as RWstudio, to make it possible to validate results
        colDetect = new rw::proximity::CollisionDetector(wc,rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

        // Using inverse kinematic from rw:
        rw::models::SerialDevice::Ptr sDevice = rw::common::ownedPtr( new rw::models::SerialDevice(device->getBase(), device->getEnd(), device->getName(), currentState));
        //urIK = new rw::invkin::ClosedFormIKSolverUR(sDevice,currentState);
        urIK = new rw::invkin::JacobianIKSolver(sDevice, Ball_Frame, currentState);
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

        ofs.open("Calibration_camera_robot.txt", std::ofstream::out);
        ofsQ.open("Calibration_camera_robot_Q.txt", std::ofstream::out);

    }
    
    //Moves the device to X,y,z but keeping its orientation.
    bool move_to_tcp_manual(motion_planning::moveToTCP::Request  &req,
                     motion_planning::moveToTCP::Response &res)
    {
        //target position = last detection of red ball position
        rw::math::Vector3D<double> targetPos(req.x, req.y, req.z);
        
        //update current state of device in model
        rw::math::Q currentQ;
        get_current_Q(currentQ);
        device->setQ(currentQ,currentState);
        
        ROS_INFO_STREAM("Calling Inverse kinematics ..." << std::endl);

        bool return_stat;
        res.ok = 1;
        rw::math::Q goal;
        //Debug
        ROS_INFO_STREAM(req.x << " " << req.y << " " << req.z);

        
        goal = convert_to_q(req.x,req.y,req.z, currentState, currentQ);
        
        return_stat = move_device_to_q(goal, currentQ, currentState);
        
        // READ OUT POSITION
        ROS_INFO("CALIBRATION: Waiting for robot to COMPLETELY finish move ...");
        // for camera calibration -> Wait until movement is COMPLETELY finished - 10s to be on safe side
        ros::Duration(5).sleep();
        ros::spinOnce();
        
        // Get current Q configuration
        get_current_Q(currentQ);
        device->setQ(currentQ, currentState);
        ROS_INFO_STREAM("Q after move: " << currentQ);
        // save it to file
        ofsQ << currentQ << std::endl;
        
        // Get transformation from base to TCP frame 
        auto baseTtcp = device->baseTframe(TCP_Frame, currentState);
        ROS_INFO_STREAM("TCP position in Robot base frame[x, y, z]: " << baseTtcp.P());
        
        // Compute transform from base to marker on the TCP
        rw::math::Transform3D<double> baseTmarker = device->baseTframe(Ball_Frame, currentState);
        // Get marker 3D position
        rw::math::Vector3D<double> markerP = baseTmarker.P();
        ROS_INFO_STREAM("Marker position [x, y, z]: " << markerP);
        
        ofs << markerP << std::endl;

        return return_stat;
    }
    
    //Moves the device to X,y,z but keeping its orientation.
    bool move_to_tcp(const action_motion_planning::Target_PositionGoalConstPtr &action_goal)
    {
        bool success;
        //update current state of device in model
        rw::math::Q currentQ;
        success = get_current_Q(currentQ);
        
        // static variable should be initialized only once
        static rw::math::Vector3D<double> previousTarget = device->baseTframe(Ball_Frame, currentState).P();
        
        // create RW Vector3D for new target position = last detection of red ball position
        rw::math::Vector3D<double> targetPos(action_goal->x, action_goal->y, action_goal->z);
        
        /*
        rw::math::Vector3D<double> aboveBase = (targetPos - rw::math::Vector3D<double>(0,0,0.3));
        double aboveBase_norm2 = aboveBase.norm2();
        
        
        // ----------------------------------------------------
        // compute new target orientation
        // z axis is the vector from base to new target pos. divide by norm2 to get unit vector
        rw::math::Vector3D<double> z_ax(aboveBase/aboveBase_norm2);
        // y axis is projection of change in target position onto plane which is normal to the new z axis
        rw::math::Vector3D<double> change_in_target = targetPos - previousTarget;
        // do the projection onto the plane
        rw::math::Vector3D<double> projection = change_in_target - rw::math::dot(change_in_target, z_ax) * z_ax;
        // finally make sure that the vector is unit size
        rw::math::Vector3D<double> y_ax = projection/projection.norm2();
        // Lastly find 3rd coordinate axis, which is orthogonal to the previous two by 
        // taking their cross product
        rw::math::Vector3D<double> cross = rw::math::cross(z_ax, y_ax);
        // Make sure to have unit vector
        rw::math::Vector3D<double> x_ax = cross/cross.norm2();
        
        // create desired target ROtation matrix
        rw::math::Rotation3D<double> targetRot(x_ax, y_ax, z_ax);
        // desired target transformation
        rw::math::Transform3D<double> targetT(targetPos, targetRot);
        
        ROS_INFO_STREAM(rw::math::RPY<double>(targetT.R()));
        
        // save current target position for future calculations
        previousTarget = targetPos;
        // ----------------------------------------------------       
        */
        
        if (success)
        {
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
            
            rw::math::Q goal;
            //Debug
            //ROS_INFO_STREAM(action_goal->x << " " << action_goal->y << " " << action_goal->z);

            
            goal = convert_to_q(action_goal->x,action_goal->y,action_goal->z, currentState, currentQ);
            //goal = convert_to_q(targetT, currentState, currentQ);
            
            return_stat = move_device_to_q(goal, currentQ, currentState);

            if (return_stat)
            {
                result_.ok = 1;
                as_.setSucceeded(result_);
                ROS_INFO("move_to_tcp FINISHED");
            }
            else
            {
                ROS_INFO("move_to_tcp INTERRUPTED DURING move");
            }    
            return return_stat;
        }
        else
        {
            ROS_INFO("move_to_tcp INTERRUPTED BEFORE start of move");
            return false;
        }
    }
    
    rw::math::Q convert_to_q(rw::math::Transform3D<double>& targetT, \
                            const rw::kinematics::State& state, const rw::math::Q& currentQ)
    {
        std::vector<rw::math::Q> solutions;
        // Find IK solution using JacobianIKSolver (ClosedFormIKSolverUR does NOT work well)
        solutions = urIK->solve(targetT, state);

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
        
        // Now it just returns the first solution, not the best!!!
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! If no solution exists, the node crashes at the moment. This needs to be fixed! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        return solutions[bestsolution];
    }
    
    // This functions uses the JacobianIKsolver to find the configuration we want.
    rw::math::Q convert_to_q(double x, double y, double z, \
                            const rw::kinematics::State& state, const rw::math::Q& currentQ)
    {
        rw::math::Vector3D<double> translation(x,y,z);

        rw::math::RPY<double> rotation(-0.406, 0.038, -1.036); //(3.14, 0, -0.8);  

        /*
        // getQ from device, setQ in currentState, get rotation from the device.
        rw::math::Q currentQ;
        get_current_Q(currentQ); //sdsip_.getQ();

        device->setQ(currentQ,currentState);        
        */
        
        rw::math::Transform3D<double> transformation(translation,rotation);
        
        std::vector<rw::math::Q> solutions;
        
        // Find IK solution using JacobianIKSolver (ClosedFormIKSolverUR does NOT work well)
        solutions = urIK->solve(transformation, state);

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
        
        // Now it just returns the first solution, not the best!!!
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! If no solution exists, the node crashes at the moment. This needs to be fixed! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        return solutions[bestsolution];
    }
    
    //Moves the device to Q
    bool move_to_q(motion_planning::moveToQ::Request  &req,
                   motion_planning::moveToQ::Response &res)
    {
        //update current state of device in model
        rw::math::Q currentQ;
        get_current_Q(currentQ);
        device->setQ(currentQ,currentState);
        
        ROS_INFO("Calling Path planner ...");

        bool return_stat;
        res.ok = 1;
        rw::math::Q goal(6,req.Q[0],req.Q[1],req.Q[2],req.Q[3],req.Q[4],req.Q[5]);

        return_stat = move_device_to_q(goal, currentQ, currentState);

        return return_stat;

        // Get current Q from sdsip_.getQ()
    }
    
    // Tells the device to servo to Q
    bool move_device_to_q(const rw::math::Q& targetQ, const rw::math::Q& currentQ,\
                          const rw::kinematics::State& state)
    {
        bool return_stat = true;

        /*
        rw::math::Q currentQ;
        get_current_Q(currentQ);
        
        device->setQ(currentQ,currentState);
        */

        rw::trajectory::QPath path;

        // Find Path
        bool path_found = plannerRRT->query(currentQ,targetQ,path);//,MAXTIME);

        if (path_found)
        {
            ROS_INFO_STREAM("Path length: " << path.size() );
            
            ROS_INFO_STREAM("Moving from: " << currentQ);
            
            rw::common::Timer t;
            bool success = true;
            for (int i = 1; i < path.size(); ++i)
            {
                ROS_INFO_STREAM("Moving to configuration " << i <<" : " << path[i] << " ...");
                // Testing out moveptp


                //if (calibration)
                {
                return_stat = sdsip_.moveServoQ(path[i]);
                //ROS_INFO_STREAM(return_stat);
                // Make sure, that new messages from robot are current
                //wait_for_current_messages();
                
                // for some reason isMovin() doesn't work to wait until movement is finished
                //t.resetAndResume();
                success = wait_to_finish_move(path[i]);
                
                if (!success) // || !return_stat)
                {
                    //success = false;
                    break; 
                }
                // Print out last Q error from path, before changing targetQ configuration
                //ROS_INFO_STREAM("Final error from path[" << i << "] targetQ: " << error_from_target(path[i]) );
                //t.pause();
                }
                /*
                else
                {
                    //return_stat = sdsip_.movePtp(path[i]); // MIchaels
                    return_stat = sdsip_.moveServoQ(path[i]); // Petrs
                }
                */
                
            }
            
            // set the action state to SUCCESS
            if (success)
            {
                //result_.ok = 1;
                //as_.setSucceeded(result_);
                return_stat = true;
                //ROS_INFO("Movement done");
            }
            else
            {
                //result_.ok = 0;
                //as_.setAborted(result_);
                return_stat = false;
                ROS_INFO("Movement INTERRUPTED DURING path following");
            }                        
        }
        else
        {
            ROS_INFO("Path was NOT found.");
            
            rw::kinematics::State goalState = wc->getDefaultState(); 
            device->setQ(targetQ,goalState);
            if (colDetect->inCollision(goalState))
                ROS_INFO("Goal is in collision!");
                
            result_.ok = 0;
            as_.setAborted(result_);
            
            return_stat = false;
        }

        return return_stat;
    }
    
protected:
    ros::NodeHandle nodehandle_;
    caros::SerialDeviceSIProxy sdsip_;
    
    actionlib::SimpleActionServer<action_motion_planning::Target_PositionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    action_motion_planning::Target_PositionFeedback feedback_;
    action_motion_planning::Target_PositionResult result_;
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
            ros::Duration(0.05).sleep();  // In seconds
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                return false;
                //success = false;
                //break;
            }
            ros::spinOnce();
            obtained_timestamp = sdsip_.getTimeStamp();
            //ROS_INFO("Waiting for current message...");
        }
        
        return true;
        
        //ToDo
        // false response when unable to get current message
    }
    
    bool get_current_Q(rw::math::Q &currentQ)
    {
        bool success = wait_for_current_messages();
        // getQ from device
        currentQ = sdsip_.getQ();
        
        return success;
    }
    /*
    rw::math::Q get_current_Q_vel()
    {
        wait_for_current_messages();
        rw::math::Q currentQ_vel = sdsip_.getQd();
        //ROS_INFO_STREAM("Current joint velocity: " << currentQ_vel);
        
        return currentQ_vel;
    }*/
    
    rw::math::Q error_from_target(const rw::math::Q &targetQ, bool &success)
    {        
        rw::math::Q currentQ;
        success = get_current_Q(currentQ);
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
    
    /*
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
    */
    
    bool wait_to_finish_move(const rw::math::Q &targetQ)
    {
        bool success = true;
        while(!approaching_goal_configuration(targetQ, success)) 
        {
            // Make sure, that new messages from robot are current
            //success = wait_for_current_messages();
            
            if (success == false)
                return false;
            
            //get_current_Q_vel();
            ros::spinOnce();
            //ros::Duration(0.1).sleep();
        }
        return success;
    }
    
    bool approaching_goal_configuration(const rw::math::Q &targetQ, bool &success)
    {       
        rw::math::Q error_Q = error_from_target(targetQ, success);
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
        return true; //approaching_goal_configuration();
        
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_motion_planner");
    ros::NodeHandle n;

    planner planner1;

    
    ros::ServiceServer service = n.advertiseService("moveToQ", &planner::move_to_q, &planner1);
    ros::ServiceServer service2 = n.advertiseService("moveToTCP_manual", &planner::move_to_tcp_manual, &planner1);
    /*
    ros::ServiceServer service3 = n.advertiseService("moveToHome", &planner::move_to_home, &planner1);
    ros::ServiceServer service4 = n.advertiseService("moveToTCPcalibration", &planner::move_to_tcp_calibration, &planner1);
    */
    ROS_INFO("Improved ACTION motionplanning ready!");
    ros::spin();

    return 0;
}
    