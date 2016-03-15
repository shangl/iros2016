#ifndef KUKADU_SAMPLEREWARDCOMPUTER
#define KUKADU_SAMPLEREWARDCOMPUTER

#include <armadillo>
#include <vector>
#include <cmath>

#include "../../utils/types.hpp"
#include "trajectorybasedreward.hpp"

namespace kukadu {

    class SampleRewardComputer : public TrajectoryBasedReward {

    private:

        double tmax;
        double slope;

    public:

        SampleRewardComputer(double slope, int degOfFreedom, double tmax, double stepSize);

        arma::vec computeFun(double t);

    };

}

#endif
