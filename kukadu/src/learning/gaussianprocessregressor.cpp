#include "gaussianprocessregressor.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    GaussianProcessRegressor::GaussianProcessRegressor(vector<vec> sampleXs, vec sampleTs, GenericKernel* kernel, double beta) {
        this->sampleXs = sampleXs;
        this->sampleTs = sampleTs;
        this->kernel = kernel;
        coVarMatrix = computeCovarianceMatrix(kernel, beta);
    }

    vec GaussianProcessRegressor::fitAtPosition(vec pos) {

        vec retVector(1);
        double normVal = computeKernelNormValue();
        int sampleSize = sampleXs.size();

        vec k(sampleSize);
        vec t(sampleSize);
        for(int i = 0; i < sampleSize; ++i) {
            vec currentItem = sampleXs.at(i);
            k(i) = kernel->evaluateKernel(currentItem, pos, &normVal);
        }

        mat ret = k.t() * inv(coVarMatrix) * sampleTs;
        retVector(0) = ret(0,0);
        return retVector;

    }

    mat GaussianProcessRegressor::computeCovarianceMatrix(GenericKernel* kernel, double beta) {

        int retDim = sampleXs.size();
        double normVal = computeKernelNormValue();
        mat ret(retDim, retDim);
        for(int i = 0; i < retDim; ++i) {
            for(int j = 0; j < retDim; ++j) {
                vec iItem = sampleXs.at(i);
                vec jItem = sampleXs.at(j);
                double k = kernel->evaluateKernel(iItem, jItem, &normVal);
                if(i != j) ret(i, j) = k;
                else ret(i, j) = k + 1/beta;
            }
        }

        return ret;

    }

    double GaussianProcessRegressor::computeKernelNormValue() {

        double ret = 0.0;
        for(int i = 0; i < sampleXs.size(); ++i) {
            vec currentItem = sampleXs.at(i);
            ret += norm(currentItem, 2);
        }

        return ret / sampleXs.size();

    }

}
