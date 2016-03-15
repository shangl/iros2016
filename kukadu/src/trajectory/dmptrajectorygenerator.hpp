#ifndef KUKADU_DMPTRAJECTORYGENERATOR
#define KUKADU_DMPTRAJECTORYGENERATOR

#include "trajectorygenerator.hpp"
#include "../types/dmp.hpp"
#include "../utils/types.hpp"
#include "../types/dmpbase.hpp"

#include <armadillo>
#include <vector>
#include <iostream>
#include <math.h>
#include <cfloat>

namespace kukadu {

    /** \brief Implements the TrajectoryGenerator interface for dynamic movement primitives
     *
     * This class provides the basis functions for learning dynamic movement primitives with linear regression (see papers on dmps)
     * \ingroup ControlPolicyFramework
     */
    class DMPTrajectoryGenerator : public TrajectoryGenerator {

    private:


        std::vector<DMPBase> baseDef;

        double ax, tau;
        // for optimization
        double previousX;
        arma::vec prevBasFun;
        double previousNorm;

        int baseFunctionCount;
        int myssize;
        int sigmassize;

        double computeNormalization(double x);

    public:

        /**
         * \brief constructor
         * \param baseDef defines the basis functions by setting the means and variances of the Gaussian basis functions
         * \param ax dmp constant
         * \param tau dmp time constant
         */
        DMPTrajectoryGenerator(std::vector<DMPBase> baseDef, double ax, double tau);

        double evaluateBasisFunction(double x, int fun);

        /**
         * \brief computes the basis function value by setting x = e^(-ax / tau * x)
         * \param x position to evaluate
         * \param fun basis function index
         */
        double evaluateBasisFunctionNonExponential(double x, int fun);

        double evaluateByCoefficientsSingle(double x, arma::vec coeff);

        /**
         * \brief computes the linear combination of basis functions values by setting x = e^(-ax / tau * x)
         * \param x position to evaluate
         * \param coeff vector of basis function coefficients
         */
        double evaluateByCoefficientsSingleNonExponential(double x, arma::vec coeff);

        arma::vec evaluateByCoefficientsMultiple(arma::vec x, int sampleCount, arma::vec coeff);

        /**
         * \brief computes the linear combination of basis functions values for multiple positions by setting x = e^(-ax / tau * x)
         * \param x vector of positions to evaluate
         * \param sampleCount size of vector x
         * \param coeff vector of basis function coefficients
         */
        arma::vec evaluateByCoefficientsMultipleNonExponential(arma::vec x, int sampleCount, arma::vec coeff);

        int getBasisFunctionCount();

        std::string getTrajectoryType();

    };

}

#endif
