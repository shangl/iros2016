#ifndef KUKADU_TRICUBEKERNEL
#define KUKADU_TRICUBEKERNEL

#include "generickernel.hpp"
#include <armadillo>
#include <math.h>
#include <iostream>

namespace kukadu {

    /** \brief Implements a tricube kernel according to the GenericKernel specifiction.
     *
     * This class implements a tricube kernel given by the function K(u) = 70/81 (1 - |u|^3)^3
     * \ingroup LearningFramework
     */
    class TricubeKernel : public GenericKernel {

    private:

    public:

        /**
         * \brief constructor
         */
        TricubeKernel();

        double evaluateKernel(arma::vec q1, arma::vec q2, void* kernelParam);

    };

}

#endif
