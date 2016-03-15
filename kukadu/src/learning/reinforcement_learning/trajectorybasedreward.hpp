#ifndef KUKADU_TRAJECTORYBASEDREWARD_H
#define KUKADU_TRAJECTORYBASEDREWARD_H

#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <armadillo>

#include "costcomputer.hpp"
#include "../../utils/types.hpp"
#include "../../types/kukadutypes.hpp"

namespace kukadu {

    class TrajectoryBasedReward : public CostComputer {

    private:

        int degOfFreedom;

        double step;
        double tmax;
        double slope;

        arma::vec rewardsWeights;

    public:

        TrajectoryBasedReward(int degOfFreedom, double tmax, double timeStep);
        TrajectoryBasedReward(int degOfFreedom, arma::vec rewardsWeights, double tmax, double timeStep);

        void writeToFile(std::string file, double tStart, double tEnd, double stepSize);

        double computeCost(KUKADU_SHARED_PTR<ControllerResult> results);

        KUKADU_SHARED_PTR<ControllerResult> getOptimalTraj();
        KUKADU_SHARED_PTR<ControllerResult> getOptimalTraj(double tmax);
        KUKADU_SHARED_PTR<ControllerResult> getOptimalTraj(double tmax, int freedomIdx);
        KUKADU_SHARED_PTR<ControllerResult> getOptimalTraj(double tmin, double tmax, int freedomIdx);

        std::vector<arma::vec> computeFun(arma::vec t);

        virtual arma::vec computeFun(double t) = 0;

    };

}

#endif
