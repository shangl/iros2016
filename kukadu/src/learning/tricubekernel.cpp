#include "tricubekernel.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    TricubeKernel::TricubeKernel() { }

    double TricubeKernel::evaluateKernel(vec q1, vec q2, void* kernelParam) {

        double ret = 0.0;
        double dist = 0.0;
        double normal = *( (double*) kernelParam );

        vec sub = 1/normal * (q1 - q2);
        dist = norm(sub, 2);

        if(dist >= 1.0) dist = 1.0;
        ret = 70.0/81.0 * pow(1 - pow(dist, 3), 3);

        return ret;

    }

}
