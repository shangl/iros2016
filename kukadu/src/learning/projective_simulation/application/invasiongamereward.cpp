#include "invasiongamereward.hpp"

using namespace std;

namespace kukadu {

    InvasionGameReward::InvasionGameReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, bool collectPrevRewards) : Reward(generator, collectPrevRewards) {
        currentTimeStep = 0;
        intDist = kukadu_uniform_distribution(0, 1);
    }

    double InvasionGameReward::computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {

        int perceptId = providedPercept->getPerceptId();
        int actionId = takenAction->getActionId();
        if(++currentTimeStep < 250)
            return INVASIONGAMEREWARD_REW * (1 - abs(perceptId - actionId));
        else
            return INVASIONGAMEREWARD_REW * (abs(perceptId - actionId));

    }

    KUKADU_SHARED_PTR<PerceptClip> InvasionGameReward::generateNextPerceptClip(int immunity) {
        return perceptClips->at(intDist(*generator));
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > InvasionGameReward::generateActionClips() {
        KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<ActionClip> > > actionClips = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<ActionClip> > >(new vector<KUKADU_SHARED_PTR<ActionClip> >());
        actionClips->push_back(KUKADU_SHARED_PTR<ActionClip>(new ActionClip(INVASIONGAMEREWARD_H_LEFT_ACTION, 1, "left action", generator)));
        actionClips->push_back(KUKADU_SHARED_PTR<ActionClip>(new ActionClip(INVASIONGAMEREWARD_H_RIGHT_ACTION, 1, "right action", generator)));
        return actionClips;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > InvasionGameReward::generatePerceptClips() {
        KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > > perceptClips = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > >(new vector<KUKADU_SHARED_PTR<PerceptClip> >());

        vector<int>* invasionLeftPercept = new vector<int>();
        invasionLeftPercept->push_back(INVASIONGAMEREWARD_H_LEFT_PERCEPT);
        vector<int>* invasionrRightPercept = new vector<int>();
        invasionrRightPercept->push_back(INVASIONGAMEREWARD_H_RIGHT_PERCEPT);
        perceptClips->push_back(KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(INVASIONGAMEREWARD_H_LEFT_PERCEPT, "left sign", generator, KUKADU_SHARED_PTR<vector<int> >(invasionLeftPercept), PS_DEFAULT_IMMUNITY)));
        perceptClips->push_back(KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(INVASIONGAMEREWARD_H_RIGHT_PERCEPT, "right sign", generator, KUKADU_SHARED_PTR<vector<int> >(invasionrRightPercept), PS_DEFAULT_IMMUNITY)));
        return KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > >(this->perceptClips = perceptClips);
    }

    int InvasionGameReward::getDimensionality() {
        return 1;
    }

}
