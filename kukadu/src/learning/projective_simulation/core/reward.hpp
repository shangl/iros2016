#ifndef KUKADU_REWARD_H
#define KUKADU_REWARD_H

#include <vector>

#include "actionclip.hpp"
#include "perceptclip.hpp"
#include "../../../types/kukadutypes.hpp"

namespace kukadu {

    class Reward {

    private:

        bool collectPrevRewards;

        std::vector<double> previousRewards;

    protected:

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;

        virtual double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) = 0;

    public:

        Reward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, bool collectPrevRewards);

        virtual int getDimensionality() = 0;

        double computeReward(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

        std::vector<double> getPreviousRewards();

        virtual KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity) = 0;
        virtual KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips() = 0;
        virtual KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips() = 0;

    };

}

#endif
