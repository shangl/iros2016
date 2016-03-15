#ifndef KUKADU_PSEVALUATOR_H
#define KUKADU_PSEVALUATOR_H

#include <string>
#include <fstream>
#include <iostream>

#include "../core/reward.hpp"
#include "../../../types/kukadutypes.hpp"
#include "../core/projectivesimulator.hpp"

namespace kukadu {

    class PSEvaluator {

    public:

        PSEvaluator();

        static void produceStatistics(KUKADU_SHARED_PTR<ProjectiveSimulator> ps, KUKADU_SHARED_PTR<Reward> reward, int numberOfWalks, int clipImmunity, int rewardValue, std::ostream& outStream);

        static std::vector<double> evaluateStatistics(std::vector<KUKADU_SHARED_PTR<std::ifstream> >& inputStreams);
        static std::pair<std::vector<int>, std::vector<double> > evaluateStatistics(std::vector<std::string> inputFiles, std::vector<int> inputPos);

    };

}

#endif
