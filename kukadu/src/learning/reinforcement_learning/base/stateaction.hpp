#ifndef STATEACTION_HPP
#define STATEACTION_HPP

#include <utility>
#include <map>
#include <list>
#include "state.hpp"
#include "action.hpp"

#include "../../../utils/types.hpp"
#include "../../../types/kukadutypes.hpp"

namespace kukadu {


    class StateAction
    {
    private:
        KUKADU_SHARED_PTR<State> state;
        KUKADU_SHARED_PTR<Action> action;

    public:
        StateAction(KUKADU_SHARED_PTR<State> state, KUKADU_SHARED_PTR<Action> action);
        bool equals(KUKADU_SHARED_PTR<StateAction> other);
        KUKADU_SHARED_PTR<State> getState();
        KUKADU_SHARED_PTR<Action> getAction();

    };

}
#endif // STATEACTION_HPP
