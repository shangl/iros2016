#ifndef KUKADU_TERMINALCOSTCOMPUTER
#define KUKADU_TERMINALCOSTCOMPUTER

#include <iostream>
#include <armadillo>
#include <vector>

#include "costcomputer.hpp"
#include "../../utils/types.hpp"

namespace kukadu {

    /** \brief The TerminalCostComputer implements the CostComputer interface
     *
     * This class implements the CostComputer in a simple way. The cost of the last rollout is inserted manually by the user to the console.
     * This method can be used for very low dimensional reinforcement learning as there a low number of rollouts is needed.
     * \ingroup ControlPolicyFramework
     */
    class TerminalCostComputer : public CostComputer {

    private:

    public:

        double computeCost(KUKADU_SHARED_PTR<ControllerResult> results);

    };

}

#endif
