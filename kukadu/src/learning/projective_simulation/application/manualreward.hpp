#ifndef KUKADU_MANUALREWARD_H
#define KUKADU_MANUALREWARD_H

#include "../core/reward.hpp"
#include "../../../types/kukadutypes.hpp"

namespace kukadu {

    class ManualReward : public Reward {

    private:

        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > actionClips;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > perceptClips;

        int nextPerceptId;
        int numberOfActions;
        int numberOfPercepts;

        double stdReward;

    protected:

        double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

    public:

        ManualReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int numberOfActions, int numberOfPercepts, bool collectPrevRewards, double stdReward);
        ~ManualReward();

        void setNextPerceptId(int nextId);

        int getDimensionality();

        KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity);
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips();

    };

}

#endif
