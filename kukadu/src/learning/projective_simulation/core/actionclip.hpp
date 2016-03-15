#ifndef KUKADU_ACTIONCLIP_H
#define KUKADU_ACTIONCLIP_H

#include <string>

#include "clip.hpp"
#include "../../../types/kukadutypes.hpp"

namespace kukadu {

    class ActionClip : public Clip {

    private:

        int actionId;
        std::string label;

    public:

        ActionClip(int actionId, int perceptDimensionality, std::string label, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator);

        int getActionId();

        std::string getLabel();
        std::string toString() const;

        std::pair<int, KUKADU_SHARED_PTR<Clip> > jumpNextRandom();

    };

}

#endif
