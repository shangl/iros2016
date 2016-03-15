#ifndef BLACKJACKRL_HPP
#define BLACKJACKRL_HPP

#include "../base/rlinterface.hpp"
#include "../base/state.hpp"
#include "../base/stateaction.hpp"
#include <string>
#include <stdio.h>
#include <list>
#include "blackjackpolicy.hpp"

/*
 *
 * SCHAU DIR PROJECTIVE SIMULATION (learning/reinforcement_learning/projective_simulation) INTERFACES AN
 *
 *
 */

namespace kukadu {

    enum Card {EMPTY, ACE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, FACE };

    class BlackJackRL : public RLInterface
    {
    private:
        int gameCounter;
        bool myStatus;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<StateAction> > > moveList;
        bool usableAce;
        bool dealerUsableAce;
        int playerCount;
        int dealerCount;
        int playerSum;
        int dealerSum;
        KUKADU_SHARED_PTR<std::vector<Card> > deck;
        Card revealed;

    public:

        BlackJackRL(int gameCounter);

        void performRollout();
        void performPolicy();
        void requestReward();
        void reset();

        void loop();
        void deal(bool player);
        int result();
    };

}
#endif // BLACKJACKRL_HPP
