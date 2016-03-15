#include "rlinterface.hpp"

namespace kukadu{

    RLInterface::RLInterface(KUKADU_SHARED_PTR<Policy> policy)
    {
        this->policy=policy;
    }

}
