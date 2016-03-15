#ifndef KUKADU_GAUSSIANKERNEL
#define KUKADU_GAUSSIANKERNEL

#include "generickernel.hpp"

#include <armadillo>
#include <math.h>
#include <iostream>

namespace kukadu {

    /** \brief Implements a Gaussian kernel according to the GenericKernel specifiction.
     *
     * This class implements a Gaussian kernel given by the function K(u) = theta0 e^(- theta1 /2 u^2)
     * \ingroup LearningFramework
     */
    class GaussianKernel : public GenericKernel {

    private:

        double theta0;
        double theta1;

    public:

        /**
         * \brief constructor
         * \param theta0 paremeter theta0 of the kernel according to the given kernel function
         * \param theta1 paremeter theta1 of the kernel according to the given kernel function
         */
        GaussianKernel(double theta0, double theta1);

        double evaluateKernel(arma::vec q1, arma::vec q2, void* kernelParam);

    };

}

#endif
