#include "../utils/utils.hpp"
#include "cartesiandmplearner.hpp"
#include "../types/cartesiandmp.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    CartesianDMPLearner::CartesianDMPLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::mat joints) : GeneralDmpLearner(dmpBase, tau, az, bz, ax, joints) {

    }

    CartesianDMPLearner::CartesianDMPLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz, std::string file) : GeneralDmpLearner(mysDef, sigmasDef, az, bz, file) {

    }

    CartesianDMPLearner::CartesianDMPLearner(double az, double bz, std::string file) : GeneralDmpLearner(az, bz, file) {

    }

    CartesianDMPLearner::CartesianDMPLearner(double az, double bz, arma::mat joints) : GeneralDmpLearner(az, bz, joints) {

    }

    KUKADU_SHARED_PTR<Dmp> CartesianDMPLearner::createDmpInstance(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                                                                double tau, double az, double bz, double ax) {

        return KUKADU_SHARED_PTR<Dmp>(new CartesianDMP(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax));

    }

    arma::mat CartesianDMPLearner::computeFitY(arma::vec& time, arma::mat &y, arma::mat &dy, arma::mat &ddy, arma::vec& vec_g) {

        // position
        mat retMat(y.n_cols - 1, y.n_rows);

        for(int i = 0; i < y.n_rows; ++i) {

            for(int j = 0; j < 3; ++j) {

                double yVal = y(i, j);
                double dyVal = dy(i, j);
                double ddyVal = ddy(i, j);
                retMat(j, i) = /*1 / (vec_g(j) - y(0, j)) */ tau * tau * ddyVal - az * (bz * (vec_g(j) - yVal) - tau * dyVal);

            }

        }

        // orientation
        arma::mat omega(y.n_rows, 3);
        arma::mat domega;
        arma::mat eta;
        arma::mat deta;

        for (int j = 0; j < y.n_rows - 1; ++j) {

            vec logL= log(tf::Quaternion(y(j + 1, 3), y(j + 1,  4), y(j + 1, 5), y(j + 1, 6)) * tf::Quaternion(y(j, 3), y(j, 4), y(j, 5), y(j, 6)).inverse());

            for (int i = 0; i < 3; i++)
                omega(j, i) = 2 * logL(i) / (time(1)-time(0));

            if (j == y.n_rows - 2)
                for (int i = 0; i < 3; i++)
                    omega(y.n_rows - 1, i) = 2 * logL(i) / (time(1)-time(0));

        }

        for(int i = 0; i < 3 ; ++i) {

            vec trajectory = omega.col(i);
            vec domegaV = computeDiscreteDerivatives(time, trajectory);
            domega = join_rows(domega, domegaV);

        }


        eta = tau * omega;
        deta = tau * domega;

        for (int i = 0; i < y.n_rows; ++i) {

            vec logL = log(tf::Quaternion(vec_g(3), vec_g(4), vec_g(5), vec_g(6)) * tf::Quaternion(y(i, 3), y(i, 4), y(i, 5), y(i, 6)).inverse());
            for (int j = 3; j < retMat.n_rows; ++j)
                retMat(j, i) = tau * deta (i, j - 3) - az * (bz * 2 * logL(j - 3) - eta(i, j - 3));

        }

        return retMat;

    }

}
