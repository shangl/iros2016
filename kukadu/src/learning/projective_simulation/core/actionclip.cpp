#include "actionclip.hpp"

using namespace std;

namespace kukadu {

    ActionClip::ActionClip(int actionId, int perceptDimensionality, std::string label, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) : Clip(CLIP_H_LEVEL_FINAL, generator, KUKADU_SHARED_PTR<vector<int> >(new vector<int>()), INT_MAX) {
        this->actionId = actionId;
        this->label = label;
        KUKADU_SHARED_PTR<vector<int> > dummyVals = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());
        for(int i = 0; i < perceptDimensionality; ++i)
            dummyVals->push_back(-actionId - 1);
        setClipDimensionValues(dummyVals);
    }

    int ActionClip::getActionId() {
        return actionId;
    }

    std::string ActionClip::getLabel() {
        return label;
    }

    std::pair<int, KUKADU_SHARED_PTR<Clip> > ActionClip::jumpNextRandom() {
        return pair<int, KUKADU_SHARED_PTR<Clip> >(0, shared_from_this());
    }

    std::string ActionClip::toString() const {
        return label;
    }

}
