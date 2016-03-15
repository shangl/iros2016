#ifndef STATE_HPP
#define STATE_HPP

#include "../../../utils/types.hpp"
#include "../../../types/kukadutypes.hpp"

namespace kukadu {

    class State
    {
    private:
    public:
        State();
        virtual bool equals(KUKADU_SHARED_PTR<State> other) = 0;
    };

}
#endif // STATE_HPP
