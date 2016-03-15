#ifndef KUKADU_MALANOBISLEARNER
#define KUKADU_MALANOBISLEARNER

#include "mahalanobis.hpp"

#include <math.h>
#include <vector>
#include <armadillo>

namespace kukadu {

    class MahalanobisLearner {

    private:

        Mahalanobis metric;

    protected:

        std::vector<arma::vec> x1s;
        std::vector<arma::vec> x2s;
        std::vector<double> distances;

    public:

        MahalanobisLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances);

        void addSample(arma::vec x1, arma::vec x2, double distance);

        int getSampleCount();
        int getVectorDim();

        double getSampleDistance(int idx);
        arma::vec getX1(int idx);
        arma::vec getX2(int idx);

        Mahalanobis getMetric();

        virtual Mahalanobis learnMetric() = 0;

    };

}

#endif
