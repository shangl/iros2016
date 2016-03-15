#include "dmpreinforcer.hpp"
#include "../../utils/gnuplot-cpp/gnuplot_i.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    DMPReinforcer::DMPReinforcer(CostComputer* cost, KUKADU_SHARED_PTR<ControlQueue> movementQueue, double ac, double tolAbsErr, double tolRelErr) {

        this->cost = cost;
        this->movementQueue = movementQueue;
        this->ac = ac;
        this->tolAbsErr = tolAbsErr;
        this->tolRelErr = tolRelErr;
        this->isFirstIteration = true;
        this->lastCost.push_back(-1.0);

    }

    bool DMPReinforcer::getIsFirstIteration() {
        return isFirstIteration;
    }

    std::vector<double> DMPReinforcer::getLastRolloutCost() {
        return lastCost;
    }

    std::vector<KUKADU_SHARED_PTR<Dmp> > DMPReinforcer::getLastRolloutParameters() {
        return rollout;
    }

    std::vector<KUKADU_SHARED_PTR<ControllerResult> > DMPReinforcer::getLastExecutionResults() {
        return dmpResult;
    }

    double DMPReinforcer::getTolAbsErr() {
        return tolAbsErr;
    }

    double DMPReinforcer::getTolRelErr() {
        return tolRelErr;
    }

    KUKADU_SHARED_PTR<ControllerResult> DMPReinforcer::getLastUpdateRes() {
        return lastUpdateRes;
    }

    void DMPReinforcer::performRollout(int doSimulation, int doExecution) {

        char cont = 'y';
        vector<Gnuplot*> gs;
        Gnuplot* g1 = NULL;

        if(isFirstIteration) {
            rollout = getInitialRollout();
            isFirstIteration = false;

            DMPExecutor dmpsim(rollout.at(0), movementQueue);

            // TODO: switch this to new class scheme (not explicetely use DMPExecutor, but trajectory executor)
            lastUpdateRes = dmpsim.simulateTrajectory(0, rollout.at(0)->getTmax(), tolAbsErr, tolRelErr);


        }
        else {

            rollout = computeRolloutParamters();

        }

        lastCost.clear();
        dmpResult.clear();

        for(int k = 0; k < rollout.size(); ++k) {

            DMPExecutor dmpsim(rollout.at(k), movementQueue);

            if(doSimulation) {

                KUKADU_SHARED_PTR<ControllerResult> simRes = dmpsim.simulateTrajectory(0, rollout.at(k)->getTmax(), tolAbsErr, tolRelErr);
                dmpResult.push_back(simRes);

            }

            if(doExecution) {

                cout << "(DMPReinforcer) do you want to execute this trajectory? (y/N) ";
                cin >> cont;

                if(doExecution && (cont == 'y' || cont == 'Y')) {

                    cout << "(DMPReinforcer) executing rollout" << endl;

                    arma::vec startingJoints = rollout.at(k)->getY0();

                    movementQueue->setStartingJoints(startingJoints);
                    movementQueue->setStiffness(2200, 300, 1.0, 15000, 150, 2.0);
                    KUKADU_SHARED_PTR<kukadu_thread> thr = movementQueue->startQueueThread();

                    dmpResult.push_back(dmpsim.executeTrajectory(ac, 0, rollout.at(k)->getTmax(), tolAbsErr, tolRelErr));

                    movementQueue->setFinish();
                    thr->join();

                }

            }

            double delta = cost->computeCost(dmpResult.at(k));
            lastCost.push_back(delta);

        }

        lastUpdate = updateStep();
        DMPExecutor dmpsim(lastUpdate, movementQueue);
        lastUpdateRes = dmpsim.simulateTrajectory(0, lastUpdate->getTmax(), tolAbsErr, tolRelErr);

        double lastUpdateCost = cost->computeCost(lastUpdateRes);

        this->lastUpdate = lastUpdate;

        cout << "(DMPReinforcer) last update reward/cost: " << lastUpdateCost << endl << endl;

    }

    KUKADU_SHARED_PTR<Dmp> DMPReinforcer::getLastUpdate() {

        return lastUpdate;

    }

    void DMPReinforcer::setLastUpdate(KUKADU_SHARED_PTR<Dmp> lastUpdate) {
        this->lastUpdate = lastUpdate;
    }

}
