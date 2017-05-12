#ifndef PATH_OPTIMIZER_HPP
#define PATH_OPTIMIZER_HPP

#include <rw/trajectory/Path.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/proximity/CollisionDetector.hpp>


#include <cmath>
#include <cstdlib>
#include <iostream>
#include <ctime>

using namespace rw::trajectory;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;


class Pathoptimizerr
{
public:
    Pathoptimizerr(QPath);

    // Path optimization:
    void prunePath();
    void shortcutPath();

    // Helper functions:
    void discretizePath(double stepsize); // Discretizes path with steps of size stepsize (norm2)

    bool checkPathSegment(Q start, Q goal,double);



    double lenghtOfPathQSpace(QPath); // Returns the length of the internal path

    double distance(Q,Q);





    QPath internal_path;
    QPath discretized_path;
    double dist_initial;
    double dist_afterpruning;
    WorkCell::Ptr wc;
    Device::Ptr device;
    State currentState;
    int stepsneeded;
    Q step2;
    rw::proximity::CollisionDetector::Ptr colDetect;


};

#endif // PATH_OPTIMIZER_HPP
