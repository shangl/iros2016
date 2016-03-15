#include "polytrajectorygenerator.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    PolyTrajectoryGenerator::PolyTrajectoryGenerator(int basisFunctionCount) {
        this->basisFunctionCount = basisFunctionCount;
    }

    double PolyTrajectoryGenerator::evaluateBasisFunction(double x, int fun) {
        return pow(x, fun);
    }

    double PolyTrajectoryGenerator::evaluateByCoefficientsSingle(double x, vec coeff) {
        double ret = 0.0;
        int coeffDegree = this->getBasisFunctionCount();
        for(int i = 0; i < coeffDegree; ++i) ret += coeff(i) * pow(x, i + 1);
        return ret;
    }

    vec PolyTrajectoryGenerator::evaluateByCoefficientsMultiple(vec x, int sampleCount, vec coeff) {
        int coeffDegree = this->getBasisFunctionCount();
        vec evals(sampleCount);
        for(int i = 0; i < sampleCount; ++i) {
            evals(i) = evaluateByCoefficientsSingle(x(i), coeff);
        }
        return evals;
    }

    int PolyTrajectoryGenerator::getBasisFunctionCount() {
        return basisFunctionCount;
    }

    string PolyTrajectoryGenerator::getTrajectoryType() {
        return "polynomial";
    }

}
