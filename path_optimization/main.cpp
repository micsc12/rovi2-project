#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>
#include <rw/kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/Frame.hpp>

#include "path_optimizer.hpp"

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




int main(int argc, char** argv) {

    QPath testpath;
    Q* next_q;
    next_q = new Q(6, 0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0);
    testpath.push_back(*next_q);
    next_q = new Q(6, 0.491173, -0.944223, -0.361745, -1.19975, 0.4972, 0.508311);
    testpath.push_back(*next_q);
    next_q = new Q(6, 1.11775, -0.611779, -0.0171425, -1.10229, 0.461199, 0.270917);
    testpath.push_back(*next_q);
    next_q = new Q(6, 0.533164, -0.936963, -0.44937, -0.475713, 0.769791, 0.594182);
    testpath.push_back(*next_q);
    next_q = new Q(6, 0.399032, -0.853716, -0.496522, -0.410834, 0.985504, 1.22076);
    testpath.push_back(*next_q);
    next_q = new Q(6, 0.710367, -0.948793, 0.0996917, 0.21574, 0.420631, 0.663132);
    testpath.push_back(*next_q);
    next_q = new Q(6, 1.33694, -1.55638, 0.0720986, -0.219145, 0.349089, 0.648826);
    testpath.push_back(*next_q);
    next_q = new Q(6, 1.76033, -0.929811, 0.6724, -0.753769, 0.386632, 0.450758);
    testpath.push_back(*next_q);
    next_q = new Q(6, 1.58669, -1.19312, 1.29897, -0.905292, 0.290008, 0.277235);
    testpath.push_back(*next_q);
    next_q = new Q(6, 1.41305, -1.45644, 1.92555, -1.05681, 0.193384, 0.103712);
    testpath.push_back(*next_q);
    next_q = new Q(6, 1.40639, -1.46655, 1.9496, -1.06263, 0.189674, 0.0970494);
    testpath.push_back(*next_q);
    next_q = new Q(6, 1.99545, -0.839973, 1.6916, -0.577307, -0.0798846, 0.136506);
    testpath.push_back(*next_q);
    next_q = new Q(6, 2.03724, -0.520004, 1.06502, -1.10344, 0.411187, 0.321213);
    testpath.push_back(*next_q);
    next_q = new Q(6, 1.94613, -0.17226, 0.438452, -0.973427, 0.413602, -0.042939);
    testpath.push_back(*next_q);
    next_q = new Q(6, 1.9, 0.0, 0.0, -1.6, 0.0, 0.0);
    testpath.push_back(*next_q);

    Pathoptimizerr optimizer(testpath);


    cout << "Path length:" << optimizer.internal_path.size() <<  endl;
    cout << optimizer.lenghtOfPathQSpace() << endl;
    optimizer.prunePath();
    cout << "path pruning done.." << endl;
    cout << "Path length:" << optimizer.internal_path.size() <<  endl;
    cout << optimizer.lenghtOfPathQSpace() << endl;

    for (int i = 0; i < 4; i++)
    {
        cout << "rosservice call moveToQ -- '[" << optimizer.internal_path[i](0) << ", " << optimizer.internal_path[i](1) << ", "<< optimizer.internal_path[i](2) << ", "<< optimizer.internal_path[i](3) << ", "<< optimizer.internal_path[i](4) << ", "<< optimizer.internal_path[i](5) << "]'" << endl;
    }
    return 0;
}
