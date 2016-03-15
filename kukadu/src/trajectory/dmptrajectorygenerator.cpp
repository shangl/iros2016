#include "dmptrajectorygenerator.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    DMPTrajectoryGenerator::DMPTrajectoryGenerator(std::vector<DMPBase> baseDef, double ax, double tau) {

        this->baseFunctionCount = -1;
        this->baseDef = baseDef;

        this->ax = ax;
        this->tau = tau;

        this->previousX = -1;
        this->prevBasFun = vec(getBasisFunctionCount());

        myssize = baseDef.size();
        sigmassize = baseDef.at(0).getSigmas().size();


    }

    double DMPTrajectoryGenerator::evaluateByCoefficientsSingle(double x, vec coeff) {
        int coeffDegree = this->getBasisFunctionCount();
        double val = 0.0;

        if(previousX != x) {
            for(int i = 0; i < coeffDegree; ++i) {
                prevBasFun(i) = evaluateBasisFunctionNonExponential(x, i);
                previousX = x;
            }
        }

        for(int i = 0; i < coeffDegree; ++i) {
            val += coeff(i) * prevBasFun(i);
        }
        return val;
    }

    double DMPTrajectoryGenerator::evaluateByCoefficientsSingleNonExponential(double x, vec coeff) {

        int coeffDegree = this->getBasisFunctionCount();
        double val = 0.0;

        if(previousX != x) {

            for(int i = 0; i < coeffDegree; ++i) {
                prevBasFun(i) = evaluateBasisFunctionNonExponential(x, i);
                previousX = x;
            }
        }

        for(int i = 0; i < coeffDegree; ++i) {
            val += coeff(i) * prevBasFun(i);
        }

        return val;

    }


    vec DMPTrajectoryGenerator::evaluateByCoefficientsMultiple(vec x, int sampleCount, vec coeff) {
        int coeffDegree = this->getBasisFunctionCount();
        vec values(sampleCount);
        for(int i = 0; i < sampleCount; ++i) {
            values(i) = evaluateByCoefficientsSingle(x(i), coeff);
        }
        return values;
    }

    vec DMPTrajectoryGenerator::evaluateByCoefficientsMultipleNonExponential(vec x, int sampleCount, vec coeff) {
        int coeffDegree = this->getBasisFunctionCount();
        vec values(sampleCount);
        for(int i = 0; i < sampleCount; ++i) {
            values(i) = evaluateByCoefficientsSingleNonExponential(x(i), coeff);
        }
        return values;
    }

    double DMPTrajectoryGenerator::computeNormalization(double x) {

        int myssize = baseDef.size();

        if(previousX != x) {

            double normVal = 0.0;

            for(int i = 0; i < myssize; ++i) {
                double my = baseDef.at(i).getMy();
                vector<double> currentSigmas = baseDef.at(i).getSigmas();
                for(int j = 0; j < currentSigmas.size(); ++j) {
                    double sigma = currentSigmas.at(j);
                    normVal += exp(  -pow(x - my, 2) / (2 * pow(sigma, 2))   );
                }
            }
            previousNorm = normVal;
        }

        return previousNorm;

    }

    // with this implementation, currently all sigmas have to be of the same size (see DMPTrajectoryGenerator::evaluateBasisFunction and DMPTrajectoryGenerator::getBasisFunctionCount)
    double DMPTrajectoryGenerator::evaluateBasisFunction(double x, int fun) {

        int mypos = fun / sigmassize;
        int sigmapos = fun % sigmassize;

        double my = baseDef.at(mypos).getMy();
        double sigma = baseDef.at(mypos).getSigmas().at(sigmapos);

        double expVal = exp( -ax / tau * x );

        double base = exp(  - pow( expVal  - my, 2) / (2 * pow(sigma, 2))   ) * expVal;
        double normVal = computeNormalization(exp( -ax / tau * x ));

        return base / normVal;

    }

    double DMPTrajectoryGenerator::evaluateBasisFunctionNonExponential(double x, int fun) {

        int mypos = fun / sigmassize;
        int sigmapos = fun % sigmassize;

        double my = baseDef.at(mypos).getMy();
        double sigma = baseDef.at(mypos).getSigmas().at(sigmapos);
        double base = exp( -pow( x - my , 2 ) / (2 * pow(sigma, 2)) ) * x;
        double normVal = computeNormalization(x);

        return base / normVal;

    }

    // with this implementation, currently all sigmas have to be of the same size (see DMPTrajectoryGenerator::evaluateBasisFunction and DMPTrajectoryGenerator::getBasisFunctionCount)
    int DMPTrajectoryGenerator::getBasisFunctionCount() {

        if(baseFunctionCount == -1) {
            int myssize = baseDef.size();
            int sigmassize = baseDef.at(0).getSigmas().size();
            baseFunctionCount = myssize * sigmassize;
        }

        return baseFunctionCount;

    }

    string DMPTrajectoryGenerator::getTrajectoryType() {
        return "dmp";
    }

}
