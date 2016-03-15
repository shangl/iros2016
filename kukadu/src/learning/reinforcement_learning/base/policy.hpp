#ifndef POLICY_HPP
#define POLICY_HPP

#include <utility>
#include <map>
#include <vector>
#include "state.hpp"
#include "action.hpp"
#include "stateaction.hpp"

#include "../../../utils/types.hpp"
#include "../../../types/kukadutypes.hpp"

namespace kukadu {

    class Policy{
    protected:
        KUKADU_SHARED_PTR<std::vector<std::pair<KUKADU_SHARED_PTR<StateAction>, double> > > qVal;
        KUKADU_SHARED_PTR<std::vector<std::pair<KUKADU_SHARED_PTR<State>,bool> > > policy;

    public:
        Policy(KUKADU_SHARED_PTR<std::vector<std::pair<KUKADU_SHARED_PTR<StateAction>, double> > > qVal, KUKADU_SHARED_PTR<std::vector<std::pair<KUKADU_SHARED_PTR<State>,bool> > >  policy);
        virtual void updatePolicy(KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<StateAction> > > sa, int result)=0;
        bool getPolicy(KUKADU_SHARED_PTR<State> state);
        virtual void dumpPolicy() =0;

    };

}
#endif
