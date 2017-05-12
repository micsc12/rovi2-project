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
#include <chrono>
#include <vector>

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

    vector<QPath> testpath;
    QPath current_path;
    Q* next_q;
    // Path 0:
    {
    next_q = new Q(6, 0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.491173, -0.944223, -0.361745, -1.19975, 0.4972, 0.508311);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.11775, -0.611779, -0.0171425, -1.10229, 0.461199, 0.270917);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.533164, -0.936963, -0.44937, -0.475713, 0.769791, 0.594182);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.399032, -0.853716, -0.496522, -0.410834, 0.985504, 1.22076);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.710367, -0.948793, 0.0996917, 0.21574, 0.420631, 0.663132);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.33694, -1.55638, 0.0720986, -0.219145, 0.349089, 0.648826);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.76033, -0.929811, 0.6724, -0.753769, 0.386632, 0.450758);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.58669, -1.19312, 1.29897, -0.905292, 0.290008, 0.277235);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.41305, -1.45644, 1.92555, -1.05681, 0.193384, 0.103712);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.40639, -1.46655, 1.9496, -1.06263, 0.189674, 0.0970494);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.99545, -0.839973, 1.6916, -0.577307, -0.0798846, 0.136506);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.03724, -0.520004, 1.06502, -1.10344, 0.411187, 0.321213);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.94613, -0.17226, 0.438452, -0.973427, 0.413602, -0.042939);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0.0, 0.0, -1.6, 0.0, 0.0);
    current_path.push_back(*next_q);
    testpath.push_back(current_path);
    current_path.clear();
    }

    // Path 1:
    {
        next_q = new Q(6, 4.43981e-16, -1.57, 4.97432e-16, -1.57, -2.21936e-16, -4.43981e-17);
        current_path.push_back(*next_q);
        next_q = new Q(6, 0.491226, -0.943427, -0.361784, -1.19898, 0.497253, 0.508366);
        current_path.push_back(*next_q);
        next_q = new Q(6, 1.1178, -0.611314, -0.0171521, -1.10185, 0.461228, 0.270939);
        current_path.push_back(*next_q);
        next_q = new Q(6, 0.533177, -0.936557, -0.449405, -0.475272, 0.769836, 0.594222);
        current_path.push_back(*next_q);
        next_q = new Q(6, 0.399041, -0.853392, -0.49655, -0.410481, 0.985543, 1.2208);
        current_path.push_back(*next_q);
        next_q = new Q(6, 0.710392, -0.948506, 0.0996998, 0.216092, 0.420634, 0.663136);
        current_path.push_back(*next_q);
        next_q = new Q(6, 1.33697, -1.55614, 0.0721055, -0.21884, 0.349091, 0.64883);
        current_path.push_back(*next_q);
        next_q = new Q(6, 1.76044, -0.929564, 0.672546, -0.753776, 0.386642, 0.450713);
        current_path.push_back(*next_q);
        next_q = new Q(6, 1.58673, -1.19303, 1.29912, -0.905312, 0.290002, 0.277192);
        current_path.push_back(*next_q);
        next_q = new Q(6, 1.41302, -1.45649, 1.92569, -1.05685, 0.193362, 0.103671);
        current_path.push_back(*next_q);
        next_q = new Q(6, 1.40639, -1.46655, 1.9496, -1.06263, 0.189674, 0.0970494);
        current_path.push_back(*next_q);
        next_q = new Q(6, 1.99545, -0.839973, 1.6916, -0.577307, -0.0798846, 0.136506);
        current_path.push_back(*next_q);
        next_q = new Q(6, 2.03724, -0.520004, 1.06502, -1.10344, 0.411187, 0.321213);
        current_path.push_back(*next_q);
        next_q = new Q(6, 1.94613, -0.17226, 0.438452, -0.973427, 0.413602, -0.042939);

    testpath.push_back(current_path);
    current_path.clear();
    }
    // Path 2:
        {
    next_q = new Q(6, -1.77267e-17, -1.57, 7.99599e-17, -1.57, 8.89046e-18, -5.77338e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.430157, -1.36422, 0.626573, -1.90474, -0.352286, 0.583833);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.05673, -0.912545, 0.479401, -1.67097, -0.466423, 0.303232);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.59189, -0.722286, 0.334934, -1.21469, -0.758764, -0.323341);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.74471, -1.08312, -0.25399, -1.08671, -1.01197, -0.949914);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.89752, -1.44395, -0.842914, -0.958726, -1.26519, -1.57649);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.91701, -1.48997, -0.918025, -0.942403, -1.29748, -1.6564);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.72076, -0.890949, -0.291452, -1.17032, -0.730544, -1.67655);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.34733, -0.453708, 0.130999, -0.570589, -1.17144, -1.20784);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.17068, -0.133204, 0.605486, -0.716398, -1.24303, -0.581263);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.93589, -0.278778, 0.272899, -1.34297, -0.626573, -0.0904876);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);
        testpath.push_back(current_path);
        current_path.clear();
        }
    // Path 3:
        {
    next_q = new Q(6, 1.0659, -1.37605, 1.79766, 0.0470952, 1.26802, -0.0697113);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.678256, -1.90131, 1.96775, 0.673668, 1.44943, 0.0115225);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.380621, -2.19586, 2.38012, 0.374208, 1.36453, 0.638096);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.00719, -1.6065, 2.06197, -0.22317, 1.25896, 1.18697);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.63049, -2.19338, 2.09976, -0.849743, 0.816931, 1.1351);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.14253, -2.59756, 2.61336, -1.47632, 0.471877, 1.52658);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.10885, -2.28998, 2.37962, -1.24418, 0.376554, 1.50098);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.04023, -1.66341, 1.90348, -0.771287, 0.18237, 1.44882);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.97161, -1.03684, 1.42734, -0.298396, -0.0118134, 1.39667);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.903, -0.410264, 0.951199, 0.174496, -0.205997, 1.34451);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.96124, -0.300717, 0.324626, -0.410707, 0.194828, 0.867196);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.99158, -0.160698, 0.256136, -0.973427, -0.431745, 0.245003);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);
        testpath.push_back(current_path);
        current_path.clear();
        }
    // Path 4:
        {
    next_q = new Q(6, 3.0195e-16, -1.57, -9.05959e-16, -1.57, 1.42139e-16, 3.55076e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.626573, -1.16016, 0.150518, -1.68771, -0.189105, -0.0370379);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.25315, -0.750317, 0.301036, -1.80543, -0.378209, -0.0740759);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.664699, -0.123744, 0.874553, -2.06912, -0.0688695, -0.316003);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.15532, -0.00987592, 1.31542, -2.2275, -0.300405, 0.310571);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.64594, 0.103992, 1.75629, -2.38589, -0.531941, 0.937144);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.86412, 0.154627, 1.95234, -2.45632, -0.634902, 1.21577);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.25824, -0.278709, 1.6356, -2.40471, -0.533296, 0.589201);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.0914, -0.324418, 1.12529, -2.12577, -0.183951, -0.0373725);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.07607, -0.212043, 0.498722, -1.96003, -0.626573, -0.12272);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);
        testpath.push_back(current_path);
        current_path.clear();
        }
    // Path 5:
        {
    next_q = new Q(6, 0.680882, -0.872039, -0.394381, -1.05851, 0.710397, 1.01184);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.03229, -0.633442, 0.207624, -0.873486, 0.738611, 1.63841);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.0698, -0.294662, 0.757649, -0.246913, 0.32693, 1.16655);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.982239, 0.228044, 1.38422, -0.0680853, 0.773973, 0.769772);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.19085, -0.392712, 1.79464, 0.558488, 1.36256, 0.248467);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.81743, -0.954767, 2.3964, -0.0425808, 1.79433, 0.40415);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.13505, -0.759379, 1.76983, -0.518127, 1.45904, 0.544302);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.22373, -0.70483, 1.5949, -0.65089, 1.36544, 0.58343);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.97544, -0.2903, 0.968328, -1.26042, 0.917457, 1.10611);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.87705, -0.348194, 0.64986, -1.18968, 0.290884, 0.567847);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.89844, -0.130586, 0.0232869, -1.60121, -0.248478, 0.626573);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);
        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 6:
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.21169, -0.949462, 0.581163, -2.05738, 0.626573, 0.132929);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.587176, -0.322888, 0.97631, -1.68232, 0.431079, -0.464559);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.916702, 0.193306, 1.60288, -1.3342, -0.0273809, -1.0012);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.402, -0.399346, 2.07764, -0.707622, -0.575904, -0.548691);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.02857, -0.468702, 1.9637, -1.28834, -0.849485, -0.407888);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.49693, -0.520545, 1.87852, -1.72243, -1.05399, -0.302638);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.09446, -0.313845, 1.25195, -1.74728, -0.482686, -0.252587);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.93645, -0.31181, 0.625377, -1.80776, -0.491258, -0.626573);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }
    // Path 7
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.0367644, -1.23706, 0.624004, -0.943427, 0.265557, 0.448614);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.355526, -1.11841, 0.916032, -0.316854, 0.519302, 0.498221);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.351936, -1.55945, 0.515405, 0.289027, -0.107271, 0.51954);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.541555, -1.63951, 0.995943, 0.9156, -0.000276165, 0.880518);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.16813, -1.61353, 1.59343, 0.520068, 0.0496865, 0.322573);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.7947, -1.43568, 1.17763, 0.831165, 0.396152, 0.708567);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.30315, -0.809109, 1.43283, 0.229151, 0.766717, 1.32624);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.29337, -0.800635, 1.4136, 0.208708, 0.730917, 1.29169);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.12217, -0.652319, 1.07687, -0.149101, 0.104344, 0.687015);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.95098, -0.504002, 0.740141, -0.50691, -0.522229, 0.0823429);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9915, -0.284524, 0.517888, -1.13348, -0.304169, 0.626573);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 8
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.346362, -0.943427, 0.447093, -1.92731, 0.0447056, -0.214728);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.529508, -1.1866, -0.0109177, -1.30074, 0.667506, -0.36046);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.54891, -1.17153, -0.637491, -0.97768, 0.574928, -0.946136);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.17548, -1.45101, -0.0624966, -0.42313, 0.670476, -1.15744);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.80206, -1.12426, 0.0538863, -0.0559883, 0.111113, -0.76243);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.01792, -0.706284, 0.395056, -0.682562, -0.382876, -0.429233);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.45439, -0.857741, 1.02163, -0.846292, -0.804121, -0.0159067);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.89086, -1.0092, 1.6482, -1.01002, -1.22537, 0.39742);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.99391, -1.04496, 1.79613, -1.04868, -1.32482, 0.495004);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.8139, -0.418383, 1.7481, -0.80508, -1.6509, 0.174309);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.37651, -0.258897, 1.12153, -0.868179, -1.61296, 0.453948);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.29772, -0.164489, 0.837294, -1.49475, -1.02321, 0.00267025);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.89898, -0.353816, 0.626573, -0.988846, -0.397822, -0.623903);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 9
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.0732827, -1.45495, 0.558063, -0.943427, 0.390653, 0.0689399);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.221514, -1.79092, 0.881307, -0.316854, -0.150774, -0.283508);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.631976, -1.2803, 1.50788, -0.202035, -0.314408, -0.903754);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.502488, -0.653732, 1.01151, 0.36189, -0.563947, -1.21057);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.00878397, -0.0271586, 1.53451, 0.558218, -1.00803, -1.48714);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.632696, -0.419851, 1.4215, 1.10761, -0.381461, -1.24071);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.25927, -0.332087, 1.35713, 0.649784, -0.300696, -1.40643);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.56915, -0.312896, 1.58742, 0.0232105, -0.500957, -1.72755);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.84588, -0.295759, 1.79307, -0.536316, -0.679789, -2.0143);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.02503, -0.224826, 1.27847, 0.090257, -0.20199, -1.60896);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.07664, -0.220019, 0.651895, 0.27972, 0.333841, -1.2108);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.8992, -0.191899, 0.2354, -0.346854, -0.0292394, -0.819255);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.01309, 0.195425, -0.0937489, -0.973427, 0.277692, -0.433743);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);


        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 10
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.626573, -1.28807, -0.374942, -1.7732, 0.0471572, 0.119792);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.362337, -0.661498, 0.125059, -1.98852, 0.122806, 0.180242);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.140447, -0.707755, 0.751632, -2.34035, 0.132232, 0.0354514);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.324152, -1.33433, 1.35954, -2.84456, -0.413033, -0.492056);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.51223, -1.87668, 1.98611, -2.83742, -0.984426, -0.734473);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.06298, -1.6442, 1.91901, -2.21979, -1.59824, -0.1079);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.5315, -1.42061, 2.04684, -1.688, -0.971671, -0.594403);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.00001, -1.19702, 2.17468, -1.15621, -0.345098, -1.08091);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.29716, -1.05521, 2.25575, -0.818933, 0.0522968, -1.38946);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.44939, -0.560924, 1.62918, -0.629996, -0.0660756, -1.08828);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.42906, -0.32711, 1.05308, -1.09144, -0.692649, -1.53435);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.91352, -0.176463, 0.473022, -1.0301, -0.934557, -0.907781);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.04651, 0.0947638, 0.0350066, -0.488529, -0.893724, -0.281208);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.86738, -0.368926, 0.550141, -0.973427, -0.267151, 0.276295);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 11
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.626573, -1.28807, -0.374942, -1.7732, 0.0471572, 0.119792);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.362337, -0.661498, 0.125059, -1.98852, 0.122806, 0.180242);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.140447, -0.707755, 0.751632, -2.34035, 0.132232, 0.0354514);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.324152, -1.33433, 1.35954, -2.84456, -0.413033, -0.492056);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.51223, -1.87668, 1.98611, -2.83742, -0.984426, -0.734473);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.06298, -1.6442, 1.91901, -2.21979, -1.59824, -0.1079);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.5315, -1.42061, 2.04684, -1.688, -0.971671, -0.594403);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.00001, -1.19702, 2.17468, -1.15621, -0.345098, -1.08091);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.29716, -1.05521, 2.25575, -0.818933, 0.0522968, -1.38946);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.44939, -0.560924, 1.62918, -0.629996, -0.0660756, -1.08828);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.42906, -0.32711, 1.05308, -1.09144, -0.692649, -1.53435);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.91352, -0.176463, 0.473022, -1.0301, -0.934557, -0.907781);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.04651, 0.0947638, 0.0350066, -0.488529, -0.893724, -0.281208);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.86738, -0.368926, 0.550141, -0.973427, -0.267151, 0.276295);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);
        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 12
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.300472, -1.87923, 0.626573, -1.30956, 0.114915, 0.403625);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.207472, -1.42181, 1.25315, -1.03395, 0.355453, 0.721697);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.0717828, -1.19338, 1.87972, -0.537365, -0.244973, 0.377593);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.633793, -1.67909, 2.50629, 0.0228367, -0.285952, 0.736368);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.24766, -1.2929, 2.04219, -0.213865, -0.912525, 1.26886);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.86153, -0.906701, 1.57808, -0.450568, -1.5391, 1.80134);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.98381, -0.829769, 1.48563, -0.49772, -1.66391, 1.90742);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.08732, -0.228572, 0.90891, -1.11321, -1.77703, 1.28084);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.12085, -0.302302, 1.12089, -1.58084, -1.83401, 0.654271);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.17865, -0.156579, 0.63744, -1.36158, -1.20744, 0.490426);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.99394, -0.318553, 0.626573, -1.16195, -0.580867, 0.249352);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 13
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.491226, -0.943427, -0.361784, -1.19898, 0.497253, 0.508366);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.1178, -0.611314, -0.0171521, -1.10185, 0.461228, 0.270939);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.533177, -0.936557, -0.449405, -0.475272, 0.769836, 0.594222);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.399041, -0.853392, -0.49655, -0.410481, 0.985543, 1.2208);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.710392, -0.948506, 0.0996998, 0.216092, 0.420634, 0.663136);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.33697, -1.55614, 0.0721055, -0.21884, 0.349091, 0.64883);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.76044, -0.929564, 0.672546, -0.753776, 0.386642, 0.450713);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.58673, -1.19303, 1.29912, -0.905312, 0.290002, 0.277192);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.41302, -1.45649, 1.92569, -1.05685, 0.193362, 0.103671);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.40639, -1.46655, 1.9496, -1.06263, 0.189674, 0.0970494);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.99545, -0.839973, 1.6916, -0.577307, -0.0798846, 0.136506);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.03724, -0.520004, 1.06502, -1.10344, 0.411187, 0.321213);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.94613, -0.17226, 0.438452, -0.973427, 0.413602, -0.042939);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);


        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 14
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.491226, -0.943427, -0.361784, -1.19898, 0.497253, 0.508366);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.1178, -0.611314, -0.0171521, -1.10185, 0.461228, 0.270939);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.533177, -0.936557, -0.449405, -0.475272, 0.769836, 0.594222);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.399041, -0.853392, -0.49655, -0.410481, 0.985543, 1.2208);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.710392, -0.948506, 0.0996998, 0.216092, 0.420634, 0.663136);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.33697, -1.55614, 0.0721055, -0.21884, 0.349091, 0.64883);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.76044, -0.929564, 0.672546, -0.753776, 0.386642, 0.450713);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.58673, -1.19303, 1.29912, -0.905312, 0.290002, 0.277192);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.41302, -1.45649, 1.92569, -1.05685, 0.193362, 0.103671);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.40639, -1.46655, 1.9496, -1.06263, 0.189674, 0.0970494);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.99545, -0.839973, 1.6916, -0.577307, -0.0798846, 0.136506);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.03724, -0.520004, 1.06502, -1.10344, 0.411187, 0.321213);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.94613, -0.17226, 0.438452, -0.973427, 0.413602, -0.042939);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);


        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 15
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.491226, -0.943427, -0.361784, -1.19898, 0.497253, 0.508366);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.1178, -0.611314, -0.0171521, -1.10185, 0.461228, 0.270939);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.533177, -0.936557, -0.449405, -0.475272, 0.769836, 0.594222);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.399041, -0.853392, -0.49655, -0.410481, 0.985543, 1.2208);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.710392, -0.948506, 0.0996998, 0.216092, 0.420634, 0.663136);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.33697, -1.55614, 0.0721055, -0.21884, 0.349091, 0.64883);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.76044, -0.929564, 0.672546, -0.753776, 0.386642, 0.450713);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.58673, -1.19303, 1.29912, -0.905312, 0.290002, 0.277192);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.41302, -1.45649, 1.92569, -1.05685, 0.193362, 0.103671);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.40639, -1.46655, 1.9496, -1.06263, 0.189674, 0.0970494);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.99545, -0.839973, 1.6916, -0.577307, -0.0798846, 0.136506);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.03724, -0.520004, 1.06502, -1.10344, 0.411187, 0.321213);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.94613, -0.17226, 0.438452, -0.973427, 0.413602, -0.042939);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }
    // Path 16
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.360484, -1.52369, 0.573706, -1.05999, 0.626573, -0.189448);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.0899393, -1.75471, 0.877796, -0.433416, 0.378761, -0.0751112);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.716512, -1.36665, 1.09699, 0.164505, -0.123246, 0.325312);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.34309, -0.978593, 1.31619, 0.762426, -0.625253, 0.725736);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.17837, -0.35202, 1.25011, 1.02149, -0.790892, 0.439418);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.04849, 0.142067, 1.198, 1.22578, -0.921507, 0.21364);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.67506, -0.0530836, 1.33485, 1.02795, -0.619944, 0.620829);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.24495, -0.280043, 0.779234, 0.40138, -0.62821, 0.297529);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.05975, -0.16984, 0.15266, -0.169748, -0.00761526, 0.44577);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.04474, 0.136032, -0.293283, -0.796322, -0.320978, 0.237949);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.95459, -0.096433, -0.00973913, -1.42289, -0.626573, 0.350629);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);


        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 17
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.491226, -0.943427, -0.361784, -1.19898, 0.497253, 0.508366);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.1178, -0.611314, -0.0171521, -1.10185, 0.461228, 0.270939);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.533177, -0.936557, -0.449405, -0.475272, 0.769836, 0.594222);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.399041, -0.853392, -0.49655, -0.410481, 0.985543, 1.2208);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.710392, -0.948506, 0.0996998, 0.216092, 0.420634, 0.663136);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.33697, -1.55614, 0.0721055, -0.21884, 0.349091, 0.64883);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.76044, -0.929564, 0.672546, -0.753776, 0.386642, 0.450713);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.58673, -1.19303, 1.29912, -0.905312, 0.290002, 0.277192);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.41302, -1.45649, 1.92569, -1.05685, 0.193362, 0.103671);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.40639, -1.46655, 1.9496, -1.06263, 0.189674, 0.0970494);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.99545, -0.839973, 1.6916, -0.577307, -0.0798846, 0.136506);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.03724, -0.520004, 1.06502, -1.10344, 0.411187, 0.321213);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.94613, -0.17226, 0.438452, -0.973427, 0.413602, -0.042939);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);


        testpath.push_back(current_path);
        current_path.clear();
        }
    // Path 18
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.360484, -1.52369, 0.573706, -1.05999, 0.626573, -0.189448);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.0899393, -1.75471, 0.877796, -0.433416, 0.378761, -0.0751112);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.716512, -1.36665, 1.09699, 0.164505, -0.123246, 0.325312);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.34309, -0.978593, 1.31619, 0.762426, -0.625253, 0.725736);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.17837, -0.35202, 1.25011, 1.02149, -0.790892, 0.439418);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.04849, 0.142067, 1.198, 1.22578, -0.921507, 0.21364);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.67506, -0.0530836, 1.33485, 1.02795, -0.619944, 0.620829);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.24495, -0.280043, 0.779234, 0.40138, -0.62821, 0.297529);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.05975, -0.16984, 0.15266, -0.169748, -0.00761526, 0.44577);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.04474, 0.136032, -0.293283, -0.796322, -0.320978, 0.237949);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.95459, -0.096433, -0.00973913, -1.42289, -0.626573, 0.350629);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);


        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 19
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.544178, -1.30324, -0.551077, -1.19352, -0.626573, 0.271896);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.17075, -1.00804, -0.314064, -0.977057, -0.815909, -0.242292);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.79732, -0.712839, -0.0770524, -0.760598, -1.00524, -0.756479);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.34733, -0.453708, 0.130999, -0.570589, -1.17144, -1.20784);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.17068, -0.133204, 0.605486, -0.716398, -1.24303, -0.581263);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.93589, -0.278778, 0.272899, -1.34297, -0.626573, -0.0904876);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 20
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.626573, -1.24705, 0.164759, -0.971518, 0.137361, -0.172021);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.98446, -1.67285, 0.626274, -0.344945, 0.356188, 0.0400519);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.3152, -1.13251, 1.06637, 0.281628, 0.491079, -0.509003);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.76246, -1.39912, 0.636091, 0.908202, 0.677235, -0.775233);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.3348, -0.812858, 1.26266, 0.421325, 0.316521, -0.387423);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.77454, -0.362413, 1.74408, 0.0472439, 0.0393742, -0.0894574);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.14796, -0.233745, 1.15813, -0.360786, 0.192975, -0.240227);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.03557, -0.40221, 0.626573, -0.987359, -0.396566, -0.199488);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 21
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.415833, -1.05664, 0.527701, -0.970679, -0.0937168, -0.626573);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.877153, -0.430071, 0.649279, -0.373044, -0.687038, -0.242594);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.782563, -0.218894, 1.24829, 0.159119, -0.965063, -0.869167);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.855038, -0.506792, 1.68139, 0.785693, -0.428394, -0.575289);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.926414, -0.073657, 2.2293, 1.41227, -0.830661, -0.929071);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.20173, -0.220632, 2.03327, 1.00647, -0.822197, -0.480298);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.58611, -0.425837, 1.75958, 0.439891, -0.81038, 0.146276);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9705, -0.631041, 1.48589, -0.126683, -0.798563, 0.772849);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.35489, -0.836246, 1.21219, -0.693257, -0.786746, 1.39942);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.12007, -0.341319, 1.81279, -1.31983, -0.307156, 1.13393);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.0946, -0.349953, 1.23574, -1.9464, -0.238962, 0.704354);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.95008, -0.317745, 0.609172, -1.86838, 0.164947, 0.626573);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 22
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.348803, -0.943427, 0.187467, -0.969283, -0.267618, -0.260585);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.707687, -0.341604, 0.81404, -0.508508, -0.691404, -0.314169);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.33426, 0.119065, 1.42577, -0.287962, -0.392751, 0.0409066);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.96083, 0.133787, 1.13657, 0.270473, -0.886391, 0.0756824);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.07999, 0.136587, 1.08157, 0.376674, -0.980269, 0.0822959);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.94989, -0.412315, 0.556387, -0.2499, -0.868513, 0.0617594);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.89205, -0.107573, 0.591452, -0.876341, -0.34053, 0.688333);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.8751, -0.113404, 0.626573, -1.16946, -0.0274178, 0.0617594);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 23
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.18328, -1.28554, -0.208139, -1.04136, -0.375896, 0.626573);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.391124, -1.29395, -0.263387, -0.414791, -0.42619, 0.00828878);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.409046, -1.48692, 0.18985, 0.162887, 0.0558846, -0.618284);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.03562, -1.67889, 0.803018, 0.443463, 0.5366, -1.02637);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.11422, -2.30546, 1.42154, 0.7644, 0.751243, -0.891225);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.47125, -2.91212, 2.04811, 0.278138, 0.92163, -0.500848);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.58048, -2.72002, 1.97923, 0.315904, 0.823256, -0.519585);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.93676, -2.09345, 1.75455, 0.439085, 0.50239, -0.5807);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.29303, -1.46687, 1.52987, 0.562265, 0.181523, -0.641815);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.64931, -0.840301, 1.30519, 0.685446, -0.139343, -0.70293);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.27597, -0.499032, 1.5547, 0.058873, -0.31051, -0.327656);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.90239, -0.326678, 0.992571, -0.086994, -0.487577, 0.298917);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.96103, -0.206024, 0.365997, -0.7089, -0.11795, -0.310123);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.86194, -0.0428827, 0.110048, -1.33547, 0.367613, -0.626573);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);

        testpath.push_back(current_path);
        current_path.clear();
        }

    // Path 24
        {
    next_q = new Q(6, 4.43439e-17, -1.57, 2.44352e-17, -1.57, 3.55347e-17, 3.33121e-17);
    current_path.push_back(*next_q);
    next_q = new Q(6, -0.626573, -1.17184, 0.281959, -1.16142, 0.238923, -0.589434);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.11022e-16, -0.652981, -0.189413, -1.46575, -0.339098, -1.1423);
    current_path.push_back(*next_q);
    next_q = new Q(6, 0.504835, -0.69349, 0.382949, -0.839174, -0.347963, -1.75088);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.13141, -0.655742, 0.382799, -0.910718, -0.9445, -2.23149);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.14677, -0.0291691, 0.995868, -1.3686, -1.42192, -1.78887);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.75958, 0.0419319, 1.62244, -1.68878, -1.4731, -1.22306);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.38615, -0.390733, 1.6404, -2.2001, -2.07841, -1.25673);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.29152, -0.374124, 1.4268, -2.11754, -1.7444, -1.12412);
    current_path.push_back(*next_q);
    next_q = new Q(6, 2.11398, -0.342967, 1.02609, -1.96265, -1.11783, -0.875345);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.93645, -0.31181, 0.625377, -1.80776, -0.491258, -0.626573);
    current_path.push_back(*next_q);
    next_q = new Q(6, 1.9, 0, 0, -1.6, 0, 0);
    current_path.push_back(*next_q);
        testpath.push_back(current_path);
        current_path.clear();
        }






    Pathoptimizerr optimizer(testpath[1]);


    cout << "Path length:" << optimizer.internal_path.size() <<  endl;
    cout << optimizer.lenghtOfPathQSpace(optimizer.internal_path) << endl;
    optimizer.prunePath();
    cout << "path pruning done.." << endl;
    cout << "Path length:" << optimizer.internal_path.size() <<  endl;
    cout << optimizer.lenghtOfPathQSpace(optimizer.internal_path) << endl;
    optimizer.shortcutPath();
    cout << "path shortcut done.." << endl;
    cout << "Path length:" << optimizer.discretized_path.size() <<  endl;
    cout << optimizer.lenghtOfPathQSpace(optimizer.discretized_path) << endl;



    for (int i = 0; i < 4; i++)
    {
        cout << "rosservice call moveToQ -- '[" << optimizer.internal_path[i](0) << ", " << optimizer.internal_path[i](1) << ", "<< optimizer.internal_path[i](2) << ", "<< optimizer.internal_path[i](3) << ", "<< optimizer.internal_path[i](4) << ", "<< optimizer.internal_path[i](5) << "]'" << endl;
    }
    return 0;
}
