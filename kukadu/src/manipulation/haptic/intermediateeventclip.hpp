#ifndef KUKADU_INTERMEDIATEEVENTCLIP_H
#define KUKADU_INTERMEDIATEEVENTCLIP_H

#include "../sensingcontroller.hpp"
#include "../../types/kukadutypes.hpp"
#include "../../learning/projective_simulation/core/clip.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace kukadu {

    class IntermediateEventClip : public Clip {

    private:

        int hapticMode;

        std::string caption;

        KUKADU_SHARED_PTR<SensingController> sensingEvent;

    public:

        IntermediateEventClip(KUKADU_SHARED_PTR<SensingController> sensingEvent,
                              int level, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, KUKADU_SHARED_PTR<std::vector<int> > clipValues, int immunity);

        std::string toString() const;
        std::pair<int, KUKADU_SHARED_PTR<Clip> > jumpNextRandom();

        KUKADU_SHARED_PTR<SensingController> getSensingController();

    };

}

#endif // INTERMEDIATEEVENTCLIP_H
