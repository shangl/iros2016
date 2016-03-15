#include "policy.hpp"

using namespace std;

namespace kukadu{

    Policy::Policy(KUKADU_SHARED_PTR<std::vector<std::pair<KUKADU_SHARED_PTR<StateAction>, double> > > qVal, KUKADU_SHARED_PTR<std::vector<std::pair<KUKADU_SHARED_PTR<State>,bool> > >  policy) {
        this->qVal=qVal;
        this->policy=policy;
    }

    bool Policy::getPolicy(KUKADU_SHARED_PTR<State> state)
    {

        for(vector<pair<KUKADU_SHARED_PTR<State>, bool> >::iterator sa = policy->begin(); sa != policy->end(); ++sa)
        {
            if((*sa).first->equals(state)){
                return (*sa).second;
            }
        }
            cout<<"error: ";
        return true;
    }
}

