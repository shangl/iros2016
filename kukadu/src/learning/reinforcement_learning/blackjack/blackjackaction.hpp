#ifndef BLACKJACKACTION_HPP
#define BLACKJACKACTION_HPP

#include "../base/action.hpp"

namespace kukadu {

    class BlackJackAction : public Action
    {

    private:
        bool action;
    public:
        BlackJackAction(bool action);
        static const bool HIT = true;
        static const bool STAY = false;

        bool getAction();
        bool equals(KUKADU_SHARED_PTR<Action> other);
        bool equals(KUKADU_SHARED_PTR<BlackJackAction> other);
    };

}
#endif // BLACKJACKACTION_HPP
