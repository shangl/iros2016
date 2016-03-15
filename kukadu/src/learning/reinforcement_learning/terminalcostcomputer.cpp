#include "terminalcostcomputer.hpp"

using namespace std;

namespace kukadu {

    double TerminalCostComputer::computeCost(KUKADU_SHARED_PTR<ControllerResult> results) {

        double delta = 0.0;
        cout << "(TerminalCostComputer) Enter the deviation in query space (also be aware of the sign; e.g. qmeasured +/- cost = qdesired)...";
        cin >> delta;

        return delta;

    }

}
