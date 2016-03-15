#ifndef KUKADU_JOINTDMPLEARNER_H
#define KUKADU_JOINTDMPLEARNER_H

#include <queue>
#include <vector>
#include <cstdlib>
#include <armadillo>

#include "../utils/types.hpp"
#include "../utils/utils.hpp"
#include "../types/dmpbase.hpp"
#include "../types/jointdmp.hpp"
#include "../types/kukadutypes.hpp"
#include "../learning/generalfitter.hpp"
#include "../trajectory/generaldmplearner.hpp"
#include "../trajectory/dmptrajectorygenerator.hpp"

namespace kukadu {

    /** \brief The TrajectoryDMPLearner encapsulates the dmp learning process
     *
     * Dynamic movement primitives can be easily learned by using this class and providing the joint data or a file containing this data. Basically this is a helper
     * that enables the programmer to reduce code complexity.
     * \ingroup ControlPolicyFramework
     */
    class JointDMPLearner : public GeneralDmpLearner {

    protected:

        KUKADU_SHARED_PTR<Dmp> createDmpInstance(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                                               double tau, double az, double bz, double ax);

        arma::mat computeFitY(arma::vec& time, arma::mat& y, arma::mat& dy, arma::mat& ddy, arma::vec& vec_g);

    public:

        /**
         * \brief constructor
         * \param dmpBase dmp basis function definition
         * \param tau dmp timing constant
         * \param az dmp az constant
         * \param bz dmp bz constant
         * \param ax dmp ax constant
         * \param joints measured joints
         * \param degFreedom robots degrees of freedom
         */
        JointDMPLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::mat joints);

        /**
         * \brief constructor
         * \param dmpBase dmp basis function definition
         * \param tau dmp timing constant
         * \param az dmp az constant
         * \param bz dmp bz constant
         * \param ax dmp ax constant
         * \param file file containing the measured joints
         * \param degFreedom robots degrees of freedom
         */
        JointDMPLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz, std::string file);

        JointDMPLearner(double az, double bz, std::string file);

        JointDMPLearner(double az, double bz, arma::mat joints);


    };

}

#endif
