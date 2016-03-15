#ifndef KUKADU_LWRREGRESSOR
#define KUKADU_LWRREGRESSOR

#include <armadillo>
#include <vector>

#include "generickernel.hpp"
#include "../utils/utils.hpp"
#include "kernelregressor.hpp"

namespace kukadu {

    /** \brief Implements the locally weighted regression method
     *
     * This class inherits from the KernelRegressor and implements locally weighted regression. It reuses the design matrices computed by the
     * GeneralFitter class.
     * \ingroup LearningFramework
     */
    class LWRRegressor : public KernelRegressor {

    private:

        std::vector<arma::vec> sampleXs;
        std::vector<arma::vec> sampleTs;

        std::vector<arma::mat> designMatrices;

        GenericKernel* kernel;

        double computeKernelNormValue();

    public:

        /**
         * \brief constructor, setting all the sample data, the selected kernel and a vector of design matrices (one design matrix for each sample)
         * \param sampleXs vector of samples (x-axis)
         * \param sampleTs vector of samples (y-axis)
         * \param kernel the selected kernel implementation
         * \param designMatrices vector of design matrices
         */
        LWRRegressor(std::vector<arma::vec> sampleXs, std::vector<arma::vec> sampleTs, GenericKernel* kernel, std::vector<arma::mat> designMatrices);
        arma::vec fitAtPosition(arma::vec pos);

    };

}

#endif
