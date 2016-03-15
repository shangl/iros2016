#include "lwrregressor.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    LWRRegressor::LWRRegressor(vector<vec> sampleXs, vector<vec> sampleTs, GenericKernel* kernel, vector<mat> designMatrices) {
        this->sampleXs = sampleXs;
        this->sampleTs = sampleTs;
        this->kernel = kernel;
        this->designMatrices = designMatrices;
    }

    double LWRRegressor::computeKernelNormValue() {
        double ret = 0.0;
        for(int i = 0; i < sampleXs.size(); ++i) {
            vec currentItem = sampleXs.at(i);
            ret += norm(currentItem, 2);
        }
        return ret / sampleXs.size();
    }

    vec LWRRegressor::fitAtPosition(vec pos) {

        int sampleSize = sampleXs.size();
        int basisFunctionCount = designMatrices.at(0).n_cols;
        double normVal = computeKernelNormValue();

        mat part1(basisFunctionCount, basisFunctionCount);
        vec part2(basisFunctionCount);

        part1.zeros();
        part2.zeros();

        for(int i = 0; i < sampleSize; ++i) {

            vec currentItem = sampleXs.at(i);
            vec currentfity = sampleTs.at(i);

            int samplePointCount = sampleTs.at(i).size();
            mat currentDesMat = designMatrices.at(i);
            double kernelVal = kernel->evaluateKernel(pos, currentItem, &normVal);

            part1 += (kernelVal * currentDesMat.t() * currentDesMat);
            part2 += (kernelVal * currentDesMat.t() * currentfity);

        }

        vec res = solve(part1, part2);

        return res;

    }

}
