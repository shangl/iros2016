#ifndef KUKADU_POWER_H
#define KUKADU_POWER_H

#include <vector>
#include <cstdlib>
#include <utility>
#include <iostream>
#include <armadillo>

#include "costcomputer.hpp"
#include "generalreinforcer.hpp"
#include "../../utils/types.hpp"
#include "../../types/trajectory.hpp"
#include "../../types/kukadutypes.hpp"
#include "../../robot/controlqueue.hpp"
#include "../../trajectory/trajectoryexecutor.hpp"

namespace kukadu {

    class PoWER : public GeneralReinforcer {

    private:

        int updatesPerRollout;
        int importanceSamplingCount;

        double explorationSigma;

        KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx;

        std::vector<double> sigmas;
        std::vector<kukadu_normal_distribution> normals;
        std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp;
        std::vector<std::pair <double, KUKADU_SHARED_PTR<Trajectory> > > sampleHistory;

        kukadu_mersenne_twister generator;

        void construct(std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    public:

        PoWER(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, double explorationSigma, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr, unsigned seed);
        PoWER(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, std::vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr, unsigned seed);

        KUKADU_SHARED_PTR<Trajectory> updateStep();

        std::vector<KUKADU_SHARED_PTR<Trajectory> > getInitialRollout();
        std::vector<KUKADU_SHARED_PTR<Trajectory> > computeRolloutParamters();

    };

}

#endif
