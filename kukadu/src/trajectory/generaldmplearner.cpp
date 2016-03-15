#include "generaldmplearner.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    void GeneralDmpLearner::construct(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, mat joints, int degFreedom) {
        this->dmpBase = dmpBase;
        this->tau = tau; this->az = az; this->bz = bz; this->ax = ax;
        this->joints = joints;
        this->degFreedom = degFreedom;
    }

    GeneralDmpLearner::GeneralDmpLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, mat joints) {
        this->construct(dmpBase, tau, az, bz, ax, joints, joints.n_cols - 1);
    }

    GeneralDmpLearner::GeneralDmpLearner(vector<double> mysDef, vector<double> sigmasDef, double az, double bz, string file) {

        // reading in file
        mat joints = readMovements(file);

        double tau = joints(joints.n_rows - 1, 0);
        double ax = -std::log(0.1) / tau / tau;

        int degFreedom = joints.n_cols - 1;

        vector<DMPBase> baseDef = buildDMPBase(mysDef, sigmasDef, ax, tau);

        this->construct(baseDef, tau, az, bz, ax, joints, degFreedom);

    }

    GeneralDmpLearner::GeneralDmpLearner(double az, double bz, std::string file) {

        vector<double> tmpmys;

        vector<double> tmpsigmas;
        tmpsigmas.push_back(0.2); tmpsigmas.push_back(0.8);
        mat joints = readMovements(file);
        int degFreedom = joints.n_cols - 1;

        tmpmys = constructDmpMys(joints);

        double tau = joints(joints.n_rows - 1, 0);
        double ax = -std::log(0.1) / tau / tau;

        vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
        this->construct(baseDef, tau, az, bz, ax, joints, degFreedom);

    }

    GeneralDmpLearner::GeneralDmpLearner(double az, double bz, arma::mat joints) {

        vector<double> tmpmys;
        vector<double> tmpsigmas;
        tmpsigmas.push_back(0.2); tmpsigmas.push_back(0.8);
        int degFreedom = joints.n_cols - 1;

        tmpmys = constructDmpMys(joints);

        double tau = joints(joints.n_rows - 1, 0);
        double ax = -std::log(0.1) / tau / tau;

        vector<DMPBase> baseDef = buildDMPBase(tmpmys, tmpsigmas, ax, tau);
        this->construct(baseDef, tau, az, bz, ax, joints, degFreedom);

    }

    KUKADU_SHARED_PTR<Dmp> GeneralDmpLearner::fitTrajectories() {

        int dataPointsNum = joints.n_rows;

        vec g(degFreedom);
        vec y0(degFreedom);
        vec dy0(degFreedom);
        vec ddy0(degFreedom);

        vector<vec> dmpCoeffs;
        vector<vec> sampleYs;
        vector<vec> fitYs;

        vector<mat> designMatrices;
        vec timeVec = joints.col(0);

        mat all_y;
        mat all_dy;
        mat all_ddy;

        // retrieve all columns for different degrees of freedom
        vector<vec> trajectories;
        for(int i = 1; i <= degFreedom; ++i) {

            vec trajectory = joints.col(i);
            trajectories.push_back(trajectory);

            vec vec_dy = computeDiscreteDerivatives(timeVec, trajectory);
            vec vec_ddy = computeDiscreteDerivatives(timeVec, vec_dy);

            all_y = join_rows(all_y, trajectory);
            all_dy = join_rows(all_dy, vec_dy);
            all_ddy = join_rows(all_ddy, vec_ddy);

        }

        vector<trajectory_learner_internal> dmpResAll = fitTrajectory(timeVec, all_y, all_dy, all_ddy);

        for(int i = 0; i < dmpResAll.size(); ++i) {

            trajectory_learner_internal dmpRes = dmpResAll.at(i);
            vec dmpCoeff = dmpRes.coeff;
            vec fity = dmpRes.fity;

            g(i) = (all_y.col(i))(dataPointsNum - 1);
            y0(i) = all_y.col(i)(0);
            dy0(i) = all_dy.col(i)(0);
            ddy0(i) = all_ddy.col(i)(0);
            dmpCoeffs.push_back(dmpCoeff);

            fitYs.push_back(fity);
            designMatrices.push_back(dmpRes.desMat);

        }

        for (int i = 0; i < all_y.n_cols; ++i) sampleYs.push_back(all_y.col(i));
        return createDmpInstance(timeVec, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax);

    }

    std::vector<trajectory_learner_internal> GeneralDmpLearner::fitTrajectory(vec time, mat y, mat dy, mat ddy) {

        int dataPointsNum = time.n_elem;

        trajectory_learner_internal ret;
        vec vec_g = y.row(dataPointsNum - 1).t();

        mat fity = computeFitY(time, y, dy, ddy, vec_g);

        vector<trajectory_learner_internal> retVec;

        for(int i = 0; i < fity.n_rows; ++i) {

            trajectory_learner_internal ret;
            DMPTrajectoryGenerator dmpTrajGen(dmpBase, ax, tau);
            GeneralFitter dmpGenFit(time, fity.row(i).t(), dataPointsNum, &dmpTrajGen);
            mat designMatrix = dmpGenFit.computeDesignMatrix();
            vec dmpCoeff = dmpGenFit.computeLinFitCoefficients(designMatrix);

            ret.fity = fity.row(i).t();
            ret.coeff = dmpCoeff;
            ret.desMat = designMatrix;

            retVec.push_back(ret);

        }

        return retVec;

    }

}
