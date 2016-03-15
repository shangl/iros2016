#include "intermediateeventclip.hpp"

#include <cstdio>
#include <iostream>

using namespace std;

namespace kukadu {

    IntermediateEventClip::IntermediateEventClip(KUKADU_SHARED_PTR<SensingController> sensingEvent,
                                                 int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, KUKADU_SHARED_PTR<std::vector<int> > clipValues, int immunity)
        : Clip(level, generator, clipValues, immunity) {

        this->caption = sensingEvent->getCaption();
        this->sensingEvent = sensingEvent;

    }

    std::pair<int, KUKADU_SHARED_PTR<Clip> > IntermediateEventClip::jumpNextRandom() {

        pair<int, KUKADU_SHARED_PTR<Clip> > retVal;
        visitedSubNode = retVal.first = sensingEvent->performClassification();
        retVal.second = getSubClipByIdx(retVal.first);
        return retVal;

    }

    std::string IntermediateEventClip::toString() const {
        return sensingEvent->getCaption();
    }

    KUKADU_SHARED_PTR<SensingController> IntermediateEventClip::getSensingController() {
        return sensingEvent;
    }

}
