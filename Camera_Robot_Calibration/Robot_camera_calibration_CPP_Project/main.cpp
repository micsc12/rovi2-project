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


/*
void writeToStatFile(std::ofstream& fout, double epsilon, unsigned long steps, Timer t)
{
    if (t.getTime() >= MAXTIME)
    {
        fout<< "NaN ,\t" << "NaN" << endl;
    }
    else
    {
        fout << epsilon << " ,\t" << steps << " ,\t" << t.getTime() << std::endl;
    }

}
 */

void marker_positions_from_Q_values()
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

    std::ifstream infile ("../Calibration_Cam_Q_Final");
    std::ofstream outfile ("../cam_robot_marker_P", std::ofstream::out);

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


    // file streams for loading marker positional coordinate in robot's base frame
    // and in camera's frame
    std::ifstream ifs_p_base ("../cam_robot_marker_P");
    std::ifstream ifs_p_cam ("../marker_P_in_Camera_Frame.txt");

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

    rw::math::Transform3D<double> baseTcamera = inverse(cameraTbase);
    std::cout << "Transformation matrix from Base frame to Camera frame (transforms from Camera frame to Base frame): "\
              << std::endl << baseTcamera << std ::endl;

    rw::math::RPY<double> rpy(baseTcamera.R());

    // tranform angles from radians to degrees as is used in RobWork
    double rollDeg = Rad2Deg * rpy[0];
    double pitchDeg = Rad2Deg * rpy[1];
    double yawDeg = Rad2Deg * rpy[2];
    double xC = baseTcamera.P()[0];
    double yC = baseTcamera.P()[1];
    double zC = baseTcamera.P()[2];

    std::cout << "Position of camera in UR1 base frame: [XYZ] " << xC << " " << yC << " " << zC << std::endl;
    std::cout << "Orientation of camera in UR1 base frame: [RPY] " << rollDeg << " " << pitchDeg << " " << yawDeg << std::endl;
    ifs_p_base.close();
    ifs_p_cam.close();

    double totalSquareErr = totalSquaredError(baseTcamera, pointPairVector);
    std::cout << "Total Square Error: " << totalSquareErr << std::endl;
}
