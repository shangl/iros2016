#include "gaussiankernel.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    GaussianKernel::GaussianKernel(double theta0, double theta1) {
        this->theta0 = theta0;
        this->theta1 = theta1;
    }

    double GaussianKernel::evaluateKernel(vec q1, vec q2, void* kernelParam) {

        double ret = 0.0;
        double dist = 0.0;

        vec sub = (q1 - q2);
        dist = norm(sub, 2);

        ret = theta0 * pow(M_E, - theta1 / 2.0 * pow(dist, 2));

        return ret;

    }

}
