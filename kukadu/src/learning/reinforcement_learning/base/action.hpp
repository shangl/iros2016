#ifndef ACTION_HPP
#define ACTION_HPP

#include "../../../utils/types.hpp"
#include "../../../types/kukadutypes.hpp"

namespace kukadu {


    class Action
    {
    private:
    public:
        Action();
        virtual bool equals(KUKADU_SHARED_PTR<Action> other) =0;
    };

}
#endif // ACTION_HPP
