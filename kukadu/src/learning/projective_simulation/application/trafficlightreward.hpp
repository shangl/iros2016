#ifndef KUKADU_TRAFFICLIGHTREWARD_H
#define KUKADU_TRAFFICLIGHTREWARD_H

#include "../core/reward.hpp"
#include "../../../types/kukadutypes.hpp"

#define TRAFFICLIGHTREWARD_GREEN_RIGHT 0 * 2 + 0
#define TRAFFICLIGHTREWARD_GREEN_LEFT 0 * 2 + 1
#define TRAFFICLIGHTREWARD_RED_RIGHT 1 * 2 + 0
#define TRAFFICLIGHTREWARD_RED_LEFT 1 * 2 + 1

#define TRAFFICLIGHTREWARD_DRIVE 0
#define TRAFFICLIGHTREWARD_STOP 1

namespace kukadu {

    class TrafficLightReward : public Reward {

    private:

        int currentT;
        kukadu_uniform_distribution intDist;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > actionClips;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > perceptClips;

    protected:

        double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

    public:

        TrafficLightReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, bool collectPrevRewards);

        int getDimensionality();
        KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity);

        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips();

    };

}

#endif // TRAFFICLIGHTREWARD_H
