#include "jointdmplearner.hpp"
#include "../types/jointdmp.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    JointDMPLearner::JointDMPLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::mat joints) : GeneralDmpLearner(dmpBase, tau, az, bz, ax, joints) {

    }

    JointDMPLearner::JointDMPLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz, std::string file) : GeneralDmpLearner(mysDef, sigmasDef, az, bz, file) {

    }

    JointDMPLearner::JointDMPLearner(double az, double bz, std::string file) : GeneralDmpLearner(az, bz, file) {

    }

    JointDMPLearner::JointDMPLearner(double az, double bz, arma::mat joints) : GeneralDmpLearner(az, bz, joints) {

    }

    KUKADU_SHARED_PTR<Dmp> JointDMPLearner::createDmpInstance(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                                           double tau, double az, double bz, double ax) {

        return KUKADU_SHARED_PTR<Dmp>(new JointDmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax));

    }

    arma::mat JointDMPLearner::computeFitY(arma::vec& time, arma::mat &y, arma::mat &dy, arma::mat &ddy, arma::vec& vec_g) {

        mat retMat(y.n_cols, y.n_rows);
        for(int i = 0; i < y.n_rows; ++i) {

            for(int j = 0; j < y.n_cols; ++j) {

                double yVal = y(i, j);
                double dyVal = dy(i, j);
                double ddyVal = ddy(i, j);
                retMat(j, i) = tau * tau * ddyVal - az * (bz * (vec_g(j) - yVal) - tau * dyVal);

            }

        }

        return retMat;

    }

}
