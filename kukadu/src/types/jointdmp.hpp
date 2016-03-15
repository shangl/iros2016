#ifndef KUKADU_JOINTDMP_H
#define KUKADU_JOINTDMP_H

#include <vector>
#include <armadillo>

#include "../types/dmp.hpp"
#include "../types/kukadutypes.hpp"

namespace kukadu {

    class JointDmp : public Dmp {
    public:

        JointDmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                 double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

        JointDmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                 double tau, double az, double bz, double ax);

        JointDmp(std::string dmpFile);

        JointDmp();

        bool isCartesian();
        virtual KUKADU_SHARED_PTR<Trajectory> copy();

    };

}

#endif
