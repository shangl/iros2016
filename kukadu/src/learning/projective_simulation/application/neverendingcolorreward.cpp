#include "neverendingcolorreward.hpp"

#include <sstream>

using namespace std;

namespace kukadu {

    NeverendingColorReward::NeverendingColorReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int numberOfActions, int numberOfCategories, bool collectPrevRewards) : Reward(generator, collectPrevRewards) {

        if(numberOfActions > 9)
            throw KukaduException("(NeverEndingColorReward) number of actions must be smaller than 10");

        currentT = 0;
        this->numberOfActions = numberOfActions;
        this->numberOfCategories = numberOfCategories + 1;

        intDist = kukadu_uniform_distribution(0, numberOfActions - 1);

        actionClips = new vector<KUKADU_SHARED_PTR<ActionClip> >();
        for(int i = 0; i < numberOfActions; ++i) {
            stringstream s;
            s << "action" << i;
            actionClips->push_back(KUKADU_SHARED_PTR<ActionClip>(new ActionClip(i, this->numberOfCategories, s.str(), generator)));
        }

        perceptClips = new vector<KUKADU_SHARED_PTR<PerceptClip> >();
        currentId = 0;

    }

    NeverendingColorReward::~NeverendingColorReward() {
        delete actionClips;
        delete perceptClips;
    }

    KUKADU_SHARED_PTR<PerceptClip> NeverendingColorReward::generateNextPerceptClip(int immunity) {

        int currentColor = currentT;

        stringstream s;
        s << "(color" << currentColor;

        KUKADU_SHARED_PTR<vector<int> > idVec = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
        idVec->push_back(currentT);
        for(int i = 1; i < numberOfCategories - 1; ++i) {

            int nextVal = intDist(*generator);
            idVec->push_back(nextVal);
            s << ", action" << nextVal;

        }

        int actionNumber = intDist(*generator);
        s << ", " << actionNumber << ")";
        idVec->push_back(actionNumber);
        return KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(currentId++, s.str(), generator, idVec, immunity));

    }

    double NeverendingColorReward::computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {

        double returnedReward = 0.0;
        ++currentT;

        KUKADU_SHARED_PTR<vector<int> > perceptIds = providedPercept->getClipDimensions();

        if(perceptIds->at(1) == takenAction->getActionId())
            returnedReward = NEVERENDINGCOLORREWARD_SUCCESSFUL_REWARD;

        return returnedReward;

    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > NeverendingColorReward::generateActionClips() {
        return KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > >(new vector<KUKADU_SHARED_PTR<ActionClip> >(actionClips->begin(), actionClips->end()));
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > NeverendingColorReward::generatePerceptClips() {
        return KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > >(new vector<KUKADU_SHARED_PTR<PerceptClip> >(perceptClips->begin(), perceptClips->end()));
    }

    int NeverendingColorReward::getDimensionality() {
        return numberOfCategories + 1;
    }

}
