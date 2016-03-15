#ifndef BLACKJACKPOLICY_HPP
#define BLACKJACKPOLICY_HPP

#include "../base/policy.hpp"

namespace kukadu {

    class BlackJackPolicy : public Policy
    {
    private:
        KUKADU_SHARED_PTR<std::vector<std::pair<KUKADU_SHARED_PTR<StateAction>, int> > > visit;
    public:
        BlackJackPolicy();
    void updatePolicy(KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<StateAction> > > sa, int result);
    void dumpPolicy();
    void init();
    };

}
#endif // BLACKJACKPOLICY_HPP
