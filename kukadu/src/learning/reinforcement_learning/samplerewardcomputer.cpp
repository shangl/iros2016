#include "samplerewardcomputer.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    SampleRewardComputer::SampleRewardComputer(double slope, int degOfFreedom, double tmax, double stepSize) : TrajectoryBasedReward(degOfFreedom, tmax, stepSize) {
        this->slope = slope;
    }

    arma::vec SampleRewardComputer::computeFun(double t) {

        arma::vec retVec(1);
        double val = 0.0;

        retVec(0) = val = sin(t * 1.5);
        return retVec;

    }

}
