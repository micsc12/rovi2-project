
#include "path_optimizer.hpp"

using namespace rw::trajectory;
using namespace rw::math;
using namespace rw::models;
using namespace std;


Pathoptimizerr::Pathoptimizerr(QPath path)
{
    internal_path = path;
    dist_initial = lenghtOfPathQSpace(internal_path);
    wc = rw::loaders::WorkCellLoader::Factory::load("workcell/WC3_Scene.wc.xml");
    device = wc->findDevice("UR1");
    collisionChecks = 0;

    currentState = wc->getDefaultState();

    //rw::kinematics::State currentStatePtr = &currentState;
    // Using same collisionchecker as RWstudio, to make it possible to validate results
    colDetect = new rw::proximity::CollisionDetector(wc,rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    std::srand(std::time(0)); // Random seed
}
Pathoptimizerr::~Pathoptimizerr()
{
    delete colDetect.get();

    device.getSharedPtr().reset();
    wc.getSharedPtr().reset();
}

void Pathoptimizerr::prunePath()
{
    before_pruning = chrono::high_resolution_clock::now();
    if(internal_path.size() < 3)
        return; // Not able to prune a path with only 2 configs!

    uint i = 2;
    while (i < internal_path.size() && internal_path.size() > 2)
    {
        Q start = internal_path[i-2];
        Q goal = internal_path[i];
        if (checkPathSegment(start,goal,0.01))
        {
            // If collisionfree, delete redundant configuration!
            internal_path.erase(internal_path.begin()+i-1);
            if (i > 2)
                i--;
        }
        else
        {
            // Not possible to remove the configuration,
            i++;
        }

    }
    after_pruning = chrono::high_resolution_clock::now();
}

void Pathoptimizerr::shortcutPath()
{
    before_shortcut = chrono::high_resolution_clock::now();
    //discretizePath(0.1);
    //std::cout << discretized_path.size() << std::endl;
    int numberOfTrials = 100; // How many times the algorithm tries to shortcut the path.
    Q point1, point2;
    int index1, index2;
    int divisor = RAND_MAX/discretized_path.size();
    for (int i = 0; i < numberOfTrials; i++)
    {
        // Behaves strangely when number of points on path is too low. Therefore this check is added
        if(discretized_path.size() < 3)
        {
            break;
        }
        // Sample two points:
        index1 = std::rand() / divisor;
        index2 = std::rand() / divisor;
        while (index2 == index1 || index2-1 == index1 || index2+1 == index1) //Make sure that we are not trying to shortcut to and from the same node..
        {
            index2 = std::rand() / divisor;

        }



        //std::cout << "indexes:  " << index1 << " " << index2 << " size: " << discretized_path.size() << std::endl;

        point1 = discretized_path[index1];
        point2 = discretized_path[index2];

        // Try to connect them:
        if(checkPathSegment(point1, point2, 0.01))
        {
            //std::cout << i << std::endl;
            // Shortcut exists, remove every point between;
            if (index1 < index2)
                discretized_path.erase(discretized_path.begin()+index1+1,discretized_path.begin()+index2-1);
            else
                discretized_path.erase(discretized_path.begin()+index2+1,discretized_path.begin()+index1-1);

            if (discretized_path.size() == 3)
            {
                if (index1 < index2)
                    discretized_path.erase(discretized_path.begin()+index1+1);
                else
                    discretized_path.erase(discretized_path.begin()+index2+1);
            }
            // Calculate new divisor:
            divisor = RAND_MAX/discretized_path.size();
        }

    }

    after_shortcut = chrono::high_resolution_clock::now();
}


bool Pathoptimizerr::checkPathSegment(Q start, Q goal, double precision)
{
    Q current_pos;
    Q difference = goal - start;
    double dist = distance(goal,start);

    device->setQ(start,currentState);
    if (colDetect->inCollision(currentState))
        return false;


    device->setQ(goal,currentState);
    if (colDetect->inCollision(currentState))
        return false;

    collisionChecks += 2;

    // Ceil value + round it up.
    double min_steps = std::ceil(dist / precision);
    double steps = std::ceil(std::log2(min_steps));

    stepsneeded = std::pow(2,steps);


    Q step = difference / std::pow(2,steps);

    step2 = step;
    int current_step;

    int offset = 0;


    for (int i = 1; i <= steps; i++)
    {
        offset = (int)std::pow(2, steps - i);
        for (int j = 0; j < std::pow(2, i - 1); j++)
        {
            // Calculate which step to check
            current_step = offset + j * offset * 2;

            if (current_step > min_steps)
                break;

            // Do the check
            current_pos = start + (step * current_step);
            device->setQ(current_pos,currentState);
            if (colDetect->inCollision(currentState))
                return false;
            collisionChecks++;

        }
    }

    //No collisions detected:
    return true;

}

void Pathoptimizerr::discretizePath(double stepsize)
{

    double current_distance;
    int number_of_steps;
    Q stepQ,currentQ;
    for (uint i = 1; i < internal_path.size(); i++)
    {
        // Add start point:
        discretized_path.push_back(internal_path[i-1]);
        // Calculate distance,difference, and number of steps necesary
        current_distance = distance(internal_path[i-1],internal_path[i]);
        number_of_steps = std::floor(current_distance/stepsize);

        stepQ = (internal_path[i] - internal_path[i-1])/(current_distance/stepsize);

        for (int j = 1; j < number_of_steps; j++)
        {
            currentQ = internal_path[i-1]+j*stepQ;
            discretized_path.push_back(currentQ);
        }

    }
    // Add last point in path
    discretized_path.push_back(internal_path.back());

}


// Returns the length of the internal path
double Pathoptimizerr::lenghtOfPathQSpace(QPath path)
{
    double ret_val = 0;
    rw::math::Q lastQ,currentQ;
    for (uint i = 1; i < path.size(); i++)
    {
        ret_val += distance(path[i-1], path[i]);
    }
    return ret_val;
}

double Pathoptimizerr::distance(Q start, Q goal)
{
    double ret_val = 0;
    for (uint j = 0; j < start.size(); j++)
    {
        ret_val +=  std::pow(goal[j] - start[j],2);
    }
    return std::sqrt(ret_val);
}

