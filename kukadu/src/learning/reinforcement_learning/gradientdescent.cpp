#include "gradientdescent.hpp"

using namespace arma;
using namespace std;

namespace kukadu {

    GradientDescent::GradientDescent(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, double explorationSigma, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : GeneralReinforcer(trajEx, cost, simulationQueue, executionQueue) {

        throw KukaduException("(GradientDescent) currently broken");

        vector<double> intSigmas;

        // init sampler
        for(int i = 0; i < initDmp.at(0)->getCoefficients().at(0).n_elem; ++i) {
            vec currCoeff = initDmp.at(0)->getCoefficients().at(0);
            kukadu_normal_distribution normal(0, explorationSigma);
            normals.push_back(normal);
            intSigmas.push_back(abs(explorationSigma));
        }

        construct(initDmp, intSigmas, updatesPerRollout, importanceSamplingCount, cost, simulationQueue, executionQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);

    }

    GradientDescent::GradientDescent(KUKADU_SHARED_PTR<TrajectoryExecutor> trajEx, std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : GeneralReinforcer(trajEx, cost, simulationQueue, executionQueue) {

        throw KukaduException("(GradientDescent) currently broken");

        // init sampler
        for(int i = 0; i < initDmp.at(0)->getCoefficients().at(0).n_elem; ++i) {

            vec currCoeff = initDmp.at(0)->getCoefficients().at(0);
            kukadu_normal_distribution normal(0, explorationSigmas.at(i));
            normals.push_back(normal);

        }

        construct(initDmp, explorationSigmas, updatesPerRollout, importanceSamplingCount, cost, simulationQueue, executionQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);

    }

    void GradientDescent::construct(std::vector<KUKADU_SHARED_PTR<Trajectory> > initDmp, vector<double> explorationSigmas, int updatesPerRollout, int importanceSamplingCount, KUKADU_SHARED_PTR<CostComputer> cost, KUKADU_SHARED_PTR<ControlQueue> simulationQueue, KUKADU_SHARED_PTR<ControlQueue> executionQueue, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) {

        setLastUpdate(initDmp.at(0));

        this->initDmp = initDmp;
        this->explorationSigma = explorationSigma;
        this->sigmas = explorationSigmas;
        this->updatesPerRollout = updatesPerRollout;
        this->importanceSamplingCount = importanceSamplingCount;
        this->updateNum = 0;

    }

    std::vector<KUKADU_SHARED_PTR<Trajectory> > GradientDescent::getInitialRollout() {
        vector<KUKADU_SHARED_PTR<Trajectory> > ret;
        ret.push_back(initDmp.at(0));
        return ret;
    }

    std::vector<KUKADU_SHARED_PTR<Trajectory> > GradientDescent::computeRolloutParamters() {

        KUKADU_SHARED_PTR<Trajectory> lastUp = getLastUpdate();
        vector<vec> dmpCoeffs = lastUp->getCoefficients();
        vector<KUKADU_SHARED_PTR<Trajectory> > nextCoeffs;

        for(int k = 0; k < updatesPerRollout; ++k) {

            for(int i = 0; i < dmpCoeffs.size(); ++i) {

                vec currCoeff = dmpCoeffs.at(i);

                for(int j = 0; j < currCoeff.n_elem; ++j) {

                    kukadu_normal_distribution normal = normals.at(j);
                    double eps = normal(generator);

                    currCoeff(j) += eps;

                }

                dmpCoeffs[i] = currCoeff;

            }

            KUKADU_SHARED_PTR<Trajectory> nextUp = lastUp->copy();
            nextUp->setCoefficients(dmpCoeffs);

            nextCoeffs.push_back(nextUp);

        }

        return nextCoeffs;

    }

    KUKADU_SHARED_PTR<Trajectory> GradientDescent::updateStep() {

        ++updateNum;
        KUKADU_SHARED_PTR<Trajectory> lastUp = getLastUpdate();
        KUKADU_SHARED_PTR<Trajectory> newUp = KUKADU_SHARED_PTR<Trajectory>();

        vector<KUKADU_SHARED_PTR<Trajectory> > lastDmps = getLastRolloutParameters();
        vector<double> lastRewards = getLastRolloutCost();

        vec lastEstimate = lastDmps.at(0)->getCoefficients().at(0);

        if(lastDmps.size() > 1) {

            // add rollouts to history
            for(int i = 0; i < lastRewards.size(); ++i) {
                pair <double, KUKADU_SHARED_PTR<Trajectory> > p(lastRewards.at(i), lastDmps.at(i));
                sampleHistory.push_back(p);
            }

            double lastUpdateRew = getLastUpdateReward();
            mat deltaTheta(lastDmps.at(0)->getCoefficients().at(0).n_elem, lastRewards.size());
            vec deltaJ(lastRewards.size());
            for(int i = 0; i < deltaTheta.n_cols; ++i) {
                KUKADU_SHARED_PTR<Trajectory> currDmp = lastDmps.at(i);
                vector<vec> currCoeffs = currDmp->getCoefficients();
                deltaJ(i) = lastRewards.at(i) - lastUpdateRew;
                for(int j = 0; j < deltaTheta.n_rows; ++j) {
                    vec currCoeffsVec = currCoeffs.at(0);
                    deltaTheta(j, i) = currCoeffsVec(j);
                }

            }

            vec gradEstimate = inv(deltaTheta * deltaTheta.t()) * deltaTheta * deltaJ;
            vec newEstimate = lastEstimate + gradEstimate / updateNum;
            cout << "last estimate: " << lastEstimate.t();
            cout << "gradient estimate" << gradEstimate.t() / updateNum << endl << endl;
            vector<vec> newCoeffs;
            newCoeffs.push_back(newEstimate);

            newUp = lastUp->copy();
            newUp->setCoefficients(newCoeffs);

        } else {
            newUp = lastDmps.at(0);
        }

        return newUp;

    }

}
