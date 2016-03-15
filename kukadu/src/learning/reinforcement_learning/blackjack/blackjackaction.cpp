#include "blackjackaction.hpp"

namespace kukadu {

    BlackJackAction::BlackJackAction(bool action) : Action()
    {
        this->action=action;
    }

    bool BlackJackAction::getAction()
    {
        return this->action;
    }

    bool BlackJackAction::equals(KUKADU_SHARED_PTR<Action> other)
    {
        KUKADU_SHARED_PTR<BlackJackAction> o  = KUKADU_STATIC_POINTER_CAST<BlackJackAction>(other);
        return this->getAction() == o->getAction();
    }

}
