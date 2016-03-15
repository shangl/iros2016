#include "blackjackstate.hpp"

namespace kukadu {


    BlackJackState::BlackJackState(int myValue, int dealerValue, bool usableAce) : State()
    {
        this->myValue=myValue;
        this->dealerValue=dealerValue;
        this->usableAce = usableAce;
    }

    int BlackJackState::getMyValue()
    {
        return this->myValue;
    }

    int BlackJackState::getDealerValue()
    {
        return this->dealerValue;
    }

    bool BlackJackState::getUsableAce()
    {
        return this->usableAce;
    }

    bool BlackJackState::equals(KUKADU_SHARED_PTR<State> other)
    {
        KUKADU_SHARED_PTR<BlackJackState> o  = KUKADU_STATIC_POINTER_CAST<BlackJackState>(other);
        return (this->getMyValue()==o->getMyValue() && this->getDealerValue() == o->getDealerValue() && this->getUsableAce() == o->getUsableAce());
    }

}

