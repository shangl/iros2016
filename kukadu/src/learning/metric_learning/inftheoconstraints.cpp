#include "inftheoconstraints.hpp"

namespace kukadu {

    InfTheoConstraints::InfTheoConstraints() {
    }

    void InfTheoConstraints::addConstraint(arma::vec x1, arma::vec x2, double slack) {

        x1s.push_back(x1);
        x2s.push_back(x2);

        slacks.push_back(slack);
        lambdas.push_back(0.0);

    }

    void InfTheoConstraints::flush() {
        x1s.clear();
        x2s.clear();
        slacks.clear();
        lambdas.clear();
    }

    int InfTheoConstraints::getConstraintCount() {
        return x1s.size();
    }

    arma::vec InfTheoConstraints::getX1(int idx) {
        return x1s.at(idx);
    }

    arma::vec InfTheoConstraints::getX2(int idx) {
        return x2s.at(idx);
    }

    double InfTheoConstraints::getSlack(int idx) {
        return slacks.at(idx);
    }

    double InfTheoConstraints::getLambda(int idx) {
        return lambdas.at(idx);
    }

    void InfTheoConstraints::setSlack(int idx, double slack) {
        slacks.at(idx) = slack;
    }

    void InfTheoConstraints::setLambda(int idx, double lambda) {
        lambdas.at(idx) = lambda;
    }

}
