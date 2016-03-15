#include "jointdmp.hpp"

namespace kukadu {

    JointDmp::JointDmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                            double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) : Dmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices,
                                                                                                                                                  tau, az, bz, ax, ac, dmpStepSize, tolAbsErr, tolRelErr) {
    }

    JointDmp::JointDmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                       double tau, double az, double bz, double ax) : Dmp(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax) {

    }

    JointDmp::JointDmp(std::string dmpFile) : Dmp(dmpFile) {

    }

    JointDmp::JointDmp() {

    }

    bool JointDmp::isCartesian() {
        return false;
    }

    KUKADU_SHARED_PTR<Trajectory> JointDmp::copy() {

        return KUKADU_SHARED_PTR<Trajectory>(new JointDmp(*this));

    }

}
