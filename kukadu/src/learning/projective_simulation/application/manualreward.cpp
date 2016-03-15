#include "manualreward.hpp"

#include <sstream>

using namespace std;

namespace kukadu {

    ManualReward::ManualReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int numberOfActions, int numberOfPercepts, bool collectPrevRewards, double stdReward) : Reward(generator, collectPrevRewards) {

        this->numberOfActions = numberOfActions;
        this->numberOfPercepts = numberOfPercepts;
        this->nextPerceptId = 0;
        this->stdReward = stdReward;

        perceptClips = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<PerceptClip> > >(new vector<KUKADU_SHARED_PTR<PerceptClip> >());
        for(int i = 0; i < numberOfPercepts; ++i) {
            stringstream s;
            s << i;
            KUKADU_SHARED_PTR<std::vector<int> > clipDimensionValues = KUKADU_SHARED_PTR<std::vector<int> >(new vector<int>());
            clipDimensionValues->push_back(i);
            perceptClips->push_back(KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(i, s.str(), generator, clipDimensionValues, PS_DEFAULT_IMMUNITY)));
        }

        actionClips = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<ActionClip> > >(new vector<KUKADU_SHARED_PTR<ActionClip> >());
        for(int i = 0; i < numberOfActions; ++i) {
            stringstream s;
            s << i;
            KUKADU_SHARED_PTR<std::vector<int> > clipDimensionValues = KUKADU_SHARED_PTR<std::vector<int> >(new vector<int>());
            clipDimensionValues->push_back(i);
            actionClips->push_back(KUKADU_SHARED_PTR<ActionClip>(new ActionClip(i, 1, s.str(), generator)));
        }

    }

    ManualReward::~ManualReward() {

    }

    void ManualReward::setNextPerceptId(int nextId) {
        this->nextPerceptId = nextId;
    }

    int ManualReward::getDimensionality() {

        return 1;

    }

    KUKADU_SHARED_PTR<PerceptClip> ManualReward::generateNextPerceptClip(int immunity) {

        return perceptClips->at(nextPerceptId);

    }

    double ManualReward::computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {

        int worked = 0;
        double retReward = 0.0;
        cout << "selected percept " << *providedPercept << " resulted in action " << *takenAction << endl;
        cout << "was it the correction action (0 = no / 1 = yes)" << endl;
        cin >> worked;

        if(worked) {
            cout << "preparation action worked; rewarded with " << stdReward << endl;
            retReward = stdReward;
        } else {
            cout << "preparation action didn't work; no reward given" << endl;
            retReward = 0.0;
        }

        return retReward;

    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > ManualReward::generateActionClips() {
        return actionClips;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > ManualReward::generatePerceptClips() {
        return perceptClips;
    }

}
