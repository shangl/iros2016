#include "trafficlightreward.hpp"

using namespace std;

namespace kukadu {

    TrafficLightReward::TrafficLightReward(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, bool collectPrevRewards) : Reward(generator, collectPrevRewards) {

        currentT = 0;
        intDist = kukadu_uniform_distribution(0, 3);
        actionClips = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > >(new vector<KUKADU_SHARED_PTR<ActionClip> >());
        actionClips->push_back(KUKADU_SHARED_PTR<ActionClip>(new ActionClip(0, 2, "drive", generator)));
        actionClips->push_back(KUKADU_SHARED_PTR<ActionClip>(new ActionClip(1, 2, "stop", generator)));

        perceptClips = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > >(new vector<KUKADU_SHARED_PTR<PerceptClip> >());

        vector<int>* tmp1 = new vector<int>();
        vector<int>* tmp2 = new vector<int>();
        vector<int>* tmp3 = new vector<int>();
        vector<int>* tmp4 = new vector<int>();
        tmp1->push_back(0); tmp1->push_back(0);
        tmp2->push_back(0); tmp2->push_back(1);
        tmp3->push_back(1); tmp3->push_back(0);
        tmp4->push_back(1); tmp4->push_back(1);
        perceptClips->push_back(KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(TRAFFICLIGHTREWARD_GREEN_RIGHT, "(green, left)", generator, KUKADU_SHARED_PTR<vector<int> >(tmp1), PS_DEFAULT_IMMUNITY)));
        perceptClips->push_back(KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(TRAFFICLIGHTREWARD_GREEN_LEFT, "(green, right)", generator, KUKADU_SHARED_PTR<vector<int> >(tmp2), PS_DEFAULT_IMMUNITY)));
        perceptClips->push_back(KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(TRAFFICLIGHTREWARD_RED_RIGHT, "(red, left)", generator, KUKADU_SHARED_PTR<vector<int> >(tmp3), PS_DEFAULT_IMMUNITY)));
        perceptClips->push_back(KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(TRAFFICLIGHTREWARD_RED_LEFT, "(red, right)", generator, KUKADU_SHARED_PTR<vector<int> >(tmp4), PS_DEFAULT_IMMUNITY)));

    }

    KUKADU_SHARED_PTR<PerceptClip> TrafficLightReward::generateNextPerceptClip(int immunity) {

        int idx = intDist(*generator);
        return perceptClips->at(idx);

    }

    double TrafficLightReward::computeRewardInternal(KUKADU_SHARED_PTR<PerceptClip> providedPercept, KUKADU_SHARED_PTR<ActionClip> takenAction) {

        double reward;
        ++currentT;

        int perceptId = providedPercept->getPerceptId();
        int actionId = takenAction->getActionId();

        // ignore arrow (green -> drive, red -> stop)
        if(currentT <= 1000) {
            if(
                    (perceptId == TRAFFICLIGHTREWARD_GREEN_RIGHT && actionId == TRAFFICLIGHTREWARD_DRIVE) ||
                    (perceptId == TRAFFICLIGHTREWARD_GREEN_LEFT && actionId == TRAFFICLIGHTREWARD_DRIVE) ||
                    (perceptId == TRAFFICLIGHTREWARD_RED_RIGHT && actionId == TRAFFICLIGHTREWARD_STOP) ||
                    (perceptId == TRAFFICLIGHTREWARD_RED_LEFT && actionId == TRAFFICLIGHTREWARD_STOP)
                    ) {
                reward = 1.0;
            } else {
                reward = 0.0;
            }
        }
        // doing the opposite to the first case (green -> stop, red -> drive)
        else if(currentT <= 2000) {
            if(
                    (perceptId == TRAFFICLIGHTREWARD_GREEN_RIGHT && actionId == TRAFFICLIGHTREWARD_DRIVE) ||
                    (perceptId == TRAFFICLIGHTREWARD_GREEN_LEFT && actionId == TRAFFICLIGHTREWARD_DRIVE) ||
                    (perceptId == TRAFFICLIGHTREWARD_RED_RIGHT && actionId == TRAFFICLIGHTREWARD_STOP) ||
                    (perceptId == TRAFFICLIGHTREWARD_RED_LEFT && actionId == TRAFFICLIGHTREWARD_STOP)
                    ) {
                reward = 0.0;
            } else {
                reward = 1.0;
            }
        }
        // ignore color (left -> drive, right -> stop)
        else if(currentT <= 3000) {
            if(
                    (perceptId == TRAFFICLIGHTREWARD_GREEN_RIGHT && actionId == TRAFFICLIGHTREWARD_STOP) ||
                    (perceptId == TRAFFICLIGHTREWARD_GREEN_LEFT && actionId == TRAFFICLIGHTREWARD_DRIVE) ||
                    (perceptId == TRAFFICLIGHTREWARD_RED_RIGHT && actionId == TRAFFICLIGHTREWARD_STOP) ||
                    (perceptId == TRAFFICLIGHTREWARD_RED_LEFT && actionId == TRAFFICLIGHTREWARD_DRIVE)
                    ) {
                reward = 1.0;
            } else {
                reward = 0.0;
            }
        }
        // always reward drive
        else {
            if(actionId == TRAFFICLIGHTREWARD_DRIVE)
                reward = 1.0;
            else
                reward = 0.0;
        }

        return reward;

    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > TrafficLightReward::generateActionClips() {
        return actionClips;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > TrafficLightReward::generatePerceptClips() {
        return KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > >(new std::vector<KUKADU_SHARED_PTR<PerceptClip> >());
    }

    int TrafficLightReward::getDimensionality() {
        return 2;
    }

}
