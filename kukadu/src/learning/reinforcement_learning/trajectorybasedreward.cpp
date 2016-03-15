#include "trajectorybasedreward.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    TrajectoryBasedReward::TrajectoryBasedReward(int degOfFreedom, double tmax, double step) {

        this->step = step;
        this->tmax = tmax;
        this->degOfFreedom = degOfFreedom;
        rewardsWeights = vec(degOfFreedom);
        rewardsWeights.fill(1.0);

    }

    TrajectoryBasedReward::TrajectoryBasedReward(int degOfFreedom, arma::vec rewardsWeights, double tmax, double step) {

        this->step = step;
        this->degOfFreedom = degOfFreedom;
        this->rewardsWeights = rewardsWeights;
        this->tmax = tmax;

    }

    double TrajectoryBasedReward::computeCost(KUKADU_SHARED_PTR<ControllerResult> results) {

        int tCount = results->getTimes().n_elem;
        tmax = results->getTimes()(tCount - 1);

        double reward = 0.0;

        vector<vec> funVals = computeFun(results->getTimes());
        for(int i = 0; i < degOfFreedom; ++i) {
            vec y = results->getYs().at(i);

            vec diffVec = funVals.at(i) - y;
            vec rewardVec = diffVec.t() * diffVec;
            reward += rewardsWeights(i) *  rewardVec(0);

        }

        reward = reward / (tCount * sum(rewardsWeights));
        reward = 1.0 / exp(sqrt(reward));

        return reward;

    }

    void TrajectoryBasedReward::writeToFile(std::string file, double tStart, double tEnd, double stepSize) {

        ofstream outFile;
        outFile.open(file.c_str());

        for(; tStart < tEnd; tStart += stepSize) {
            arma::vec currentY = computeFun(tStart);
            outFile << tStart << "\t" << currentY.t() << endl;
        }

        outFile.close();

    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryBasedReward::getOptimalTraj() {
        return getOptimalTraj(tmax);
    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryBasedReward::getOptimalTraj(double tmax) {

        double tmin = 0;

        int size = (int) ( (double) (tmax - tmin) / (double) step);

        vec retT = vec(size);
        int i = 0;

        for(double t = tmin; i < size; t = t + step, ++i) {
            retT(i) = t - tmin;
        }

        return KUKADU_SHARED_PTR<ControllerResult>(new ControllerResult(retT, computeFun(retT)));

    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryBasedReward::getOptimalTraj(double tmax, int freedomIdx) {
        return getOptimalTraj(0, tmax, freedomIdx);
    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryBasedReward::getOptimalTraj(double tmin, double tmax, int freedomIdx) {


        //TODO: revise this
        double step = 0.1;

        int size = (int) ( (double) (tmax - tmin) / (double) step);

        vec ys = vec(size);
        vec retT = vec(size);
        int i = 0;

        for(double t = tmin; i < size; t = t + step, ++i) {
            retT(i) = t - tmin;
            ys(i) = computeFun(t).at(freedomIdx);
        }

        vector<vec> yss;
        yss.push_back(ys);
        return KUKADU_SHARED_PTR<ControllerResult>(new ControllerResult(retT, yss));

    }

    std::vector<arma::vec> TrajectoryBasedReward::computeFun(arma::vec t) {

        vector<vec> retYs;
        for(int i = 0; i < degOfFreedom; ++i)
            retYs.push_back(vec(t.n_elem));

        for(int i = 0; i < t.n_elem; ++i) {
            vec funVal = computeFun(t(i));
            // very inefficient --> switch to arma::mat
            for(int j = 0; j < degOfFreedom; ++j) {
                vec yVec = retYs.at(j);
                yVec(i) = funVal(j);
                retYs.at(j) = yVec;
            }
        }

        return retYs;

    }

}
