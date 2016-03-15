#include "controlleractionclip.hpp"

using namespace std;

namespace kukadu {

    ControllerActionClip::ControllerActionClip(int actionId, KUKADU_SHARED_PTR<Controller> actionController,
                          KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator) : ActionClip(actionId, 1, actionController->getCaption(), generator) {
        this->actionController = actionController;
    }

    void ControllerActionClip::performAction() {

        int executeIt = 0;
        cout << "(ControllerActionClip) selected preparation action is \"" << actionController->getCaption() << "\"; want to execute it? (0 = no / 1 = yes)" << endl;
        cin >> executeIt;

        if(executeIt == 1) {
            actionController->performAction();
        } else {
            cout << "(ControllerActionClip) you decided not to perform the action; continue" << endl;
        }
    }

    KUKADU_SHARED_PTR<Controller> ControllerActionClip::getActionController() {
        return actionController;
    }

}
