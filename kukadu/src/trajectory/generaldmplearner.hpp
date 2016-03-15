#ifndef KUKADU_GENERALDMPLEARNER_H
#define KUKADU_GENERALDMPLEARNER_H

#include <queue>
#include <vector>
#include <cstdlib>
#include <armadillo>

#include "../utils/utils.hpp"
#include "../utils/types.hpp"
#include "../types/dmpbase.hpp"
#include "../types/kukadutypes.hpp"
#include "../learning/generalfitter.hpp"
#include "../trajectory/dmptrajectorygenerator.hpp"

namespace kukadu {

    class GeneralDmpLearner {

    private:

        int degFreedom;
        arma::mat joints;

        std::vector<DMPBase> dmpBase;

        void construct(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::mat joints, int degFreedom);
        std::vector<trajectory_learner_internal> fitTrajectory(arma::vec time, arma::mat y, arma::mat dy, arma::mat ddy);

    protected:

        double az;
        double bz;
        double ax;
        double tau;

        virtual KUKADU_SHARED_PTR<Dmp> createDmpInstance(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
                                                       double tau, double az, double bz, double ax) = 0;

        virtual arma::mat computeFitY(arma::vec& time, arma::mat& y, arma::mat& dy, arma::mat& ddy, arma::vec& vec_g) = 0;

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
        GeneralDmpLearner(std::vector<DMPBase> dmpBase, double tau, double az, double bz, double ax, arma::mat joints);

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
        GeneralDmpLearner(std::vector<double> mysDef, std::vector<double> sigmasDef, double az, double bz, std::string file);

        GeneralDmpLearner(double az, double bz, std::string file);

        GeneralDmpLearner(double az, double bz, arma::mat joints);

        /**
         * \brief fit the specified trajectories
         */
        KUKADU_SHARED_PTR<Dmp> fitTrajectories();

    };

}

#endif
