#include <iostream>
// for converting numbers to strings
#include <sstream>
// for reading and writing from and to a file
#include <fstream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

// includes for checking and creating directory
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <rwlibs/algorithms/PointPairsRegistration.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

typedef std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > PointPair;

//function prototypes
void marker_positions_from_Q_values(std::string input_file, std::string output_file);
void ball_positions_from_Q_values(std::string input_file, std::string output_file_base, std::string output_file_cam,\
                                  rw::math::Transform3D<double>& cameraTbase);
double totalSquaredError(rw::math::Transform3D<double> baseTcamera,\
                         std::vector<PointPair> pointPairVector);
void find_baseTcamera(std::string input_file, std::string output_file, rw::math::Transform3D<double> &baseTcamera_out);



void marker_positions_from_Q_values(std::string input_file, std::string output_file)
{
    /*
     * computes forward kinematics based on Q values in the input file loaded to <infile>
     * saves positional part of transformation to the file <outfile>
     */
    const string wcFile = "/home/petr/catkin_ws/workcell/WC3_Scene.wc.xml";
    const string deviceName = "UR1";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
    }

    rw::kinematics::Frame* marker_Frame = wc->findFrame("WSG50.MarkerFrame");
    if (marker_Frame == NULL) {
        cerr << "marker Frame not found!" << endl;
    }

    std::ifstream infile (input_file);
    std::ofstream outfile (output_file, std::ofstream::out);

    State state = wc->getDefaultState();

    // Load joint configurations for file and compute position of the marker for all of them
    double q0, q1, q2, q3, q4, q5;
    while (infile >> q0 >> q1 >> q2 >> q3 >> q4 >> q5)
    {
        rw::math::Q config(6, q0, q1, q2, q3, q4, q5);
        device->setQ(config, state);
        rw::math::Transform3D<double> markerT = device->baseTframe(marker_Frame, state);
        outfile << markerT.P()[0] << " " << markerT.P()[1] << " " << markerT.P()[2] << std::endl;
    }

    infile.close();
    outfile.close();

}

void ball_positions_from_Q_values(std::string input_file, std::string output_file_base, std::string output_file_cam,\
                                  rw::math::Transform3D<double>& cameraTbase)
{
    /*
     * computes forward kinematics based on Q values in the input file loaded to <infile>
     * saves positional part of transformation to the file <outfile>
     */
    const string wcFile = "/home/petr/catkin_ws/workcell/WC3_Scene.wc.xml";
    const string deviceName = "UR1";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
    }

    rw::kinematics::Frame* ball_Frame = wc->findFrame("BallFrame");
    if (ball_Frame == NULL) {
        cerr << "Ball Frame not found!" << endl;
    }

    std::ifstream infile (input_file);
    std::ofstream outfile_base (output_file_base, std::ofstream::out);
    std::ofstream outfile_cam (output_file_cam, std::ofstream::out);

    State state = wc->getDefaultState();

    // Load joint configurations for file and compute position of the marker for all of them
    double q0, q1, q2, q3, q4, q5;
    while (infile >> q0 >> q1 >> q2 >> q3 >> q4 >> q5)
    {
        q0 = q0*Deg2Rad; q1 = q1*Deg2Rad; q2 = q2*Deg2Rad; q3 = q3*Deg2Rad; q4 = q4*Deg2Rad; q5 = q5*Deg2Rad;
        rw::math::Q config(6, q0, q1, q2, q3, q4, q5);
        device->setQ(config, state);
        rw::math::Transform3D<double> ballT_base = device->baseTframe(ball_Frame, state);
        rw::math::Transform3D<double> ballT_cam = cameraTbase * ballT_base;
        outfile_base << ballT_base.P()[0] << " " << ballT_base.P()[1] << " " << ballT_base.P()[2] << std::endl;
        outfile_cam << ballT_cam.P()[0] << " " << ballT_cam.P()[1] << " " << ballT_cam.P()[2] << std::endl;
    }

    infile.close();
    outfile_base.close();

}

void find_baseTcamera(std::string p_base_file, std::string p_cam_file, rw::math::Transform3D<double> &baseTcamera_out)
{
    // file streams for loading marker positional coordinate in robot's base frame
    // and in camera's frame
    std::ifstream ifs_p_base (p_base_file);
    std::ifstream ifs_p_cam (p_cam_file);

    double x,y,z;
    rw::math::Vector3D<double> p_base;
    rw::math::Vector3D<double> p_cam;
    PointPair pointPair;
    std::vector<PointPair> pointPairVector;

    while(ifs_p_base >> x >> y >> z) {
        p_base = rw::math::Vector3D<double>(x, y, z);
        ifs_p_cam >> x >> y >> z;
        p_cam = rw::math::Vector3D<double>(x, y, z);
        pointPair = std::pair<rw::math::Vector3D<double>, rw::math::Vector3D<double> >(p_base, p_cam);

        pointPairVector.push_back(pointPair);
    }

    //rw::math::Transform3D<double> cameraTbase = rwlibs::algorithms::\
    PointPairsRegistration::pointPairRegistrationQuaternion(pointPairVector);
    rw::math::Transform3D<double> cameraTbase = rwlibs::algorithms::\
    PointPairsRegistration::pointPairRegistrationSVD(pointPairVector);      //pointPairRegistrationSVD results in much smaller total square error

    baseTcamera_out = inverse(cameraTbase);
    std::cout << "Transformation matrix from Base frame to Camera frame (transforms from Camera frame to Base frame): "\
              << std::endl << baseTcamera_out << std ::endl << std::endl;

    std::cout << "Transformation matrix from Camera frame to Base frame (transforms from Base frame to Camera frame): "\
              << std::endl << cameraTbase << std ::endl << std::endl;

    rw::math::RPY<double> rpy(baseTcamera_out.R());

    // tranform angles from radians to degrees as is used in RobWork
    double rollDeg = Rad2Deg * rpy[0];
    double pitchDeg = Rad2Deg * rpy[1];
    double yawDeg = Rad2Deg * rpy[2];
    double xC = baseTcamera_out.P()[0];
    double yC = baseTcamera_out.P()[1];
    double zC = baseTcamera_out.P()[2];

    std::cout << "Position of camera in UR1 base frame: [XYZ] " << xC << " " << yC << " " << zC << std::endl;
    std::cout << "Orientation of camera in UR1 base frame: [RPY] " << rollDeg << " " << pitchDeg << " " << yawDeg << std::endl;
    ifs_p_base.close();
    ifs_p_cam.close();

    double totalSquareErr = totalSquaredError(baseTcamera_out, pointPairVector);
    std::cout << "Total Square Error: " << totalSquareErr << std::endl;
}

double totalSquaredError(rw::math::Transform3D<double> baseTcamera,\
                         std::vector<PointPair> pointPairVector)
{
    /*
     * computes sum of squared errors of difference between points in robot base frame
     * and transformed point from camera frame into robot's base frame
     * transformation is done by matrix found using <pointPairRegistration> function
     */
    double totalSquareError = 0;
    rw::math::Vector3D<double> error;
    for (size_t i = 0; i < pointPairVector.size(); ++i)
    {
        //std::cout << pointPairVector[i].first << std::endl;
        //std::cout << baseTcamera*pointPairVector[i].second << std::endl;

        error = (pointPairVector[i].first - baseTcamera*pointPairVector[i].second);

        totalSquareError += std::pow(error.norm2(), 2);
        //std::cout << error << std::endl << std::endl;
    }

    return totalSquareError;
}

int main(int argc, char** argv) {
    //marker_positions_from_Q_values();

    /*
    rw::math::Vector3D<double> basePcamera(0.601006, 0.60059, 1.35784);
    rw::math::Rotation3D<double> baseRcamera(-0.932767, 0.165071, -0.320464,\
                                              0.359333, 0.496624, -0.790091,\
                                             0.0287291, -0.852125, -0.52255);

    rw::math::Transform3D<double> baseTcamera(basePcamera, baseRcamera);
    */
/*
    std::string p_base = "../cam_robot_marker_P.txt";
    std::string p_cam = "../marker_P_in_Camera_Frame.txt";
    rw::math::Transform3D<double> baseTcamera;
    find_baseTcamera(p_base, p_cam, baseTcamera);
*/

    std::string inFile = "../robot_3d.txt";
    std::string outFile_BaseFrame = "../Ball_3d_Pos_Base.txt";
    std::string outFile_CamFrame = "../Ball_3d_Pos_Cam.txt";

    rw::math::Vector3D<double> cameraPbase(0.305777, 0.759573, 1.37666);
    rw::math::Rotation3D<double> cameraRbase(-0.932767, 0.359333, 0.0287291,\
                                              0.165071, 0.496624, -0.852125,\
                                             -0.320464, -0.790091, -0.52255);

    rw::math::Transform3D<double> cameraTbase(cameraPbase, cameraRbase);
    ball_positions_from_Q_values(inFile, outFile_BaseFrame, outFile_CamFrame, cameraTbase);

}
