#ifndef KUKADU_GENERICKERNEL
#define KUKADU_GENERICKERNEL

#include <armadillo>

namespace kukadu {

    /** \brief Provides an interface for generic kernel functions.
     *
     * The GenericKernel class is used by the KernelRegressor. Implementations of GenericKernel have to provide a certain kernel function, according to the kernel criteria.
     * \ingroup LearningFramework
     */
    class GenericKernel {

    private:

    public:

        /**
         * \brief constructor
         */
        GenericKernel();

        /**
         * \brief computes kernel values with given vectors q1 and q2 and passes a not further specified kernel parameter that can be used by the kernel implementation
         * \param q1 vector q1 with K = K(d(q1, q2))
         * \param q2 vector q2 with K = K(d(q1, q2))
         * \param kernelParam arbitrary kernel parameter
         */
        virtual double evaluateKernel(arma::vec q1, arma::vec q2, void* kernelParam) = 0;

    };

}

#endif
