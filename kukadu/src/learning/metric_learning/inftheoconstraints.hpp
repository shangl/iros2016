#ifndef INFTHEOCONSTRAINTS
#define INFTHEOCONSTRAINTS

#include <math.h>
#include <vector>
#include <armadillo>

namespace kukadu {

    class InfTheoConstraints {

    private:

        std::vector<arma::vec> x1s;
        std::vector<arma::vec> x2s;
        std::vector<double> slacks;
        std::vector<double> lambdas;

    public:

        InfTheoConstraints();

        void addConstraint(arma::vec x1, arma::vec x2, double slack);
        void flush();

        int getConstraintCount();

        arma::vec getX1(int idx);
        arma::vec getX2(int idx);
        double getSlack(int idx);
        double getLambda(int idx);

        void setSlack(int idx, double slack);
        void setLambda(int idx, double lambda);
    };

}

#endif
