#ifndef KUKADU_GENERALREINFORCER_H
#define KUKADU_GENERALREINFORCER_H

#include <vector>
#include <iostream>
#include <armadillo>

#include "costcomputer.hpp"
#include "../../types/trajectory.hpp"
#include "../../robot/controlqueue.hpp"
#include "../../trajectory/trajectoryexecutor.hpp"

namespace kukadu {

    /** \brief The GeneralReinforcer provides a general framework for reinforcement learning.
     *
     * It is an abstract class that implements elementary functionality that should be in common for all reinforcement learning methods (such as rollout execution).
     * A concrete implementation of the DMPReinforcer has to implement the missing parts such as the methods getInitialRollout and computeRolloutParamters.
     * \ingroup ControlPolicyFramework
     */
    class GeneralReinforcer {

    private:

        bool isFirstIteration;

        double ac;
        double tolAbsErr;
        double tolRelErr;
        double dmpStepSize;
        double lastUpdateCost;

        KUKADU_SHARED_PTR<CostComputer> cost;
        KUKADU_SHARED_PTR<Trajectory> lastUpdate;
        KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx;
        KUKADU_SHARED_PTR<ControlQueue> executionQueue;
        KUKADU_SHARED_PTR<ControlQueue> simulationQueue;
        KUKADU_SHARED_PTR<ControllerResult> lastUpdateRes;

        std::vector<double> lastCost;
        std::vector<KUKADU_SHARED_PTR<Trajectory> > rollout;
        std::vector<KUKADU_SHARED_PTR<ControllerResult> > dmpResult;

    public:

        /**
         * \brief constructor
         * \param cost CostComputer instance that computes the cost of a given rollout
         * \param movementQueue ControlQueue instance for robot execution
         * \param ac dmp phase stopping parameter
         * \param dmpStepSize step size for dmp execution
         * \param tolAbsErr absolute tolerated error for numerical approximation
         * \param tolRelErr relative tolerated error for numerical approximation
         */
        GeneralReinforcer(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue);

        /**
         * \brief returns true if the first iteration has not been performed yet
         */
        bool getIsFirstIteration();

        /**
         * \brief executes rollout. first, the trajectory is simulated and the user is asked, whether the trajectory really should be executed
         * \param doSimulation flag whether trajectory should be simulated
         * \param doExecution flag whether trajectory should be executed at robot
         */
        void performRollout(int doSimulation, int doExecution);
        void setLastUpdate(KUKADU_SHARED_PTR<Trajectory> lastUpdate);

        double getLastUpdateReward();

        KUKADU_SHARED_PTR<Trajectory> getLastUpdate();
        KUKADU_SHARED_PTR<ControllerResult> getLastUpdateRes();

        /**
         * \brief returns cost for the last executed rollout
         */
        std::vector<double> getLastRolloutCost();
        std::vector<KUKADU_SHARED_PTR<Trajectory> > getLastRolloutParameters();
        std::vector<KUKADU_SHARED_PTR<ControllerResult> > getLastExecutionResults();

        virtual KUKADU_SHARED_PTR<Trajectory> updateStep() = 0;

        /**
         * \brief returns the first rollout of the reinforcement learning algorithm
         */
        virtual std::vector<KUKADU_SHARED_PTR<Trajectory> > getInitialRollout() = 0;

        /**
         * \brief computes the dmp parameters for the next rollout
         */
        virtual std::vector<KUKADU_SHARED_PTR<Trajectory> > computeRolloutParamters() = 0;

    };

}

#endif
