#ifndef BLACKJACKSTATE_HPP
#define BLACKJACKSTATE_HPP

#include "../base/state.hpp"

namespace kukadu {

    class BlackJackState : public State
    {
    private:
        int myValue;
        int dealerValue;
        bool usableAce;

    public:
        BlackJackState(int myValue, int dealerValue, bool usableAce);

        int getMyValue();
        int getDealerValue();
        bool getUsableAce();
        bool equals(KUKADU_SHARED_PTR<State> other);
        bool equals(KUKADU_SHARED_PTR<BlackJackState> other);

    };


}
#endif // BLACKJACKSTATE_HPP
