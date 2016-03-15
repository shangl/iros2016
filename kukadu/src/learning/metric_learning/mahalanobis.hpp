#ifndef MAHALANOBIS
#define MAHALANOBIS

#include <math.h>
#include <armadillo>

namespace kukadu {

    class Mahalanobis {

    private:

        arma::mat M;

    public:

        Mahalanobis();
        Mahalanobis(int dim);
        Mahalanobis(arma::mat M);
        Mahalanobis(const Mahalanobis& maha);

        double computeSquaredDistance(arma::vec vec1, arma::vec vec2);

        arma::vec getCoefficients();

        void setM(arma::mat M);
        arma::mat getM() const;
        arma::mat getDecomposition();

    };

}

#endif
