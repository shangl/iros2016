#include "stateaction.hpp"

using namespace std;

namespace kukadu {

    StateAction::StateAction(KUKADU_SHARED_PTR<State> state, KUKADU_SHARED_PTR<Action> action)
    {
        this->state=state;
        this->action=action;
    }
    bool StateAction::equals(KUKADU_SHARED_PTR<StateAction> other)
    {
        return (this->state->equals(other->getState())) && (this->action->equals(other->getAction()));
    }

    KUKADU_SHARED_PTR<State> StateAction::getState()
    {
        return this->state;
    }

    KUKADU_SHARED_PTR<Action> StateAction::getAction()
    {
        return this->action;
    }
}
