#ifndef RLINTERFACE_HPP
#define RLINTERFACE_HPP

#include "policy.hpp"

#include "../../../utils/types.hpp"
#include "../../../types/kukadutypes.hpp"

namespace kukadu {

    class RLInterface
    {
        protected:
        KUKADU_SHARED_PTR<Policy> policy;
        public:
            RLInterface(KUKADU_SHARED_PTR<Policy> policy);
            virtual void performRollout() = 0;
            virtual void performPolicy() = 0;
            virtual void requestReward() =0;
            virtual void reset() =0;
    };

}

#endif // RLINTERFACE_HPP
