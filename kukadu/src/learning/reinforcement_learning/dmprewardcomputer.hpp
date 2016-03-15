#ifndef KUKADU_DMPREWARDCOMPUTER_H
#define KUKADU_DMPREWARDCOMPUTER_H

#include <cmath>
#include <vector>
#include <armadillo>

#include "costcomputer.hpp"
#include "../../utils/types.hpp"
#include "../../utils/utils.hpp"
#include "trajectorybasedreward.hpp"
#include "../../types/kukadutypes.hpp"
#include "../../robot/plottingcontrolqueue.hpp"

namespace kukadu {

    class DmpRewardComputer : public TrajectoryBasedReward {

    private:

        double az;
        double bz;
        double timeStep;

        std::string file;

        KUKADU_SHARED_PTR<ControllerResult> executionResult;

        int binaryTimeSearch(arma::vec times, double t);

    public:

        DmpRewardComputer(std::string file, double az, double bz, double timeStep, int degOfFreedom, double tmax, double step);

        arma::vec computeFun(double t);

    };

}

#endif
