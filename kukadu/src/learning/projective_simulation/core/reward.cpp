#include "reward.hpp"

using namespace std;

namespace kukadu {

    Reward::Reward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, bool collectPrevRewards) {
        this->generator = generator;
        this->collectPrevRewards = collectPrevRewards;
    }

    double Reward::computeReward(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {
        double retrievedReward = computeRewardInternal(providedPercept, takenAction);
        if(collectPrevRewards)
            previousRewards.push_back(retrievedReward);
        return retrievedReward;
    }

    std::vector<double> Reward::getPreviousRewards() {
        return previousRewards;
    }

}
