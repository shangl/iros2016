#ifndef KUKADU_TRAJECTORYCOMPARATOR
#define KUKADU_TRAJECTORYCOMPARATOR

#include "../types/trajectory.hpp"

namespace kukadu {

    class TrajectoryComparator {

    public:

        virtual double computeDistance() = 0;

    };

}

#endif
