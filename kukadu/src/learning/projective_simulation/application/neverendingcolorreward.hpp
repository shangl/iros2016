#ifndef KUKADU_NEVERENDINGCOLORREWARD_H
#define KUKADU_NEVERENDINGCOLORREWARD_H

#include "../core/reward.hpp"
#include "../../../types/kukadutypes.hpp"

#define NEVERENDINGCOLORREWARD_LEFT 0
#define NEVERENDINGCOLORREWARD_RIGHT 1

// used for experiment with ranking
//#define NEVERENDINGCOLORREWARD_SUCCESSFUL_REWARD 1000

// used for experiment without ranking to get faster convergence
#define NEVERENDINGCOLORREWARD_SUCCESSFUL_REWARD 10000

namespace kukadu {

    class NeverendingColorReward : public Reward {

    private:

        int currentId;
        int currentT;
        int numberOfActions;
        int numberOfCategories;
        kukadu_uniform_distribution intDist;
        std::vector<KUKADU_SHARED_PTR<ActionClip> >* actionClips;
        std::vector<KUKADU_SHARED_PTR<PerceptClip> >* perceptClips;

    protected:

        double computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction);

    public:

        NeverendingColorReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int numberOfActions, int numberOfCategories, bool collectPrevRewards);
        ~NeverendingColorReward();

        int getDimensionality();
        KUKADU_SHARED_PTR<PerceptClip> generateNextPerceptClip(int immunity);

        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > generateActionClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > generatePerceptClips();

    };

}

#endif
