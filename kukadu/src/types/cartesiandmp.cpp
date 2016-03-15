#include "cartesiandmp.hpp"
#include "../utils/utils.hpp"

#include <iostream>

namespace kukadu {

    CartesianDMP::CartesianDMP(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                               double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : Dmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices,
                                                                                                                                                     tau, az, bz, ax, ac, dmpStepSize, tolAbsErr, tolRelErr) {

    }

    CartesianDMP::CartesianDMP(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                               double tau, double az, double bz, double ax) : Dmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax) {

    }

    CartesianDMP::CartesianDMP() {
    }

    bool CartesianDMP::isCartesian() {
        return true;
    }

    KUKADU_SHARED_PTR<Trajectory> CartesianDMP::copy() {

        return KUKADU_SHARED_PTR<Trajectory>(new CartesianDMP(*this));

    }

    tf::Quaternion CartesianDMP::getQ0() {
        return getQByIdx(0);
    }

    tf::Quaternion CartesianDMP::getQByIdx(int idx) {
        return tf::Quaternion(sampleYs.at(3)(idx), sampleYs.at(4)(idx), sampleYs.at(5)(idx), sampleYs.at(6)(idx));
    }

    arma::vec CartesianDMP::getEta0() {

        return getEtaByIdx(0);

    }

    arma::vec CartesianDMP::getEtaByIdx(int idx) {

        arma::vec omega(3);
        omega.fill(0.0);

        if(idx < getSampleCount()) {
            tf::Quaternion q0 = getQByIdx(idx);
            tf::Quaternion q1 = getQByIdx(idx + 1);

            omega = 2 * kukadu::log(q1 * q0.inverse());

        }

        return getTau() * omega / this->dmpStepSize;
    }

    tf::Quaternion CartesianDMP::getQg() {
        return getQByIdx(getSampleCount() - 1);
    }

}
