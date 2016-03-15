#ifndef KUKADU_KERNELREGRESSOR
#define KUKADU_KERNELREGRESSOR

#include <vector>
#include <armadillo>

namespace kukadu {

    /** \brief Provides arbitrary interface for kernel methods
     *
     * This class provides the interface for kernel machine learning methods such as Gaussian process regression or locally weighted regression
     * \ingroup LearningFramework
     */
    class KernelRegressor {

    private:


    public:

        /**
         * \brief constructor
         */
        KernelRegressor();

        /**
         * \brief performs the kernel method for predicting the functin value at a given position
         * \param pos vector defining the required position
         */
        virtual arma::vec fitAtPosition(arma::vec pos) = 0;

    };

}

#endif
