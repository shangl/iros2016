#ifndef KUKADU_CONTROLLERACTIONCLIP_H
#define KUKADU_CONTROLLERACTIONCLIP_H

#include "../controller.hpp"
#include "../../learning/projective_simulation/core/actionclip.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace kukadu {

    class ControllerActionClip : public ActionClip {

    private:

        std::string caption;

        KUKADU_SHARED_PTR<Controller> actionController;

    public:

        ControllerActionClip(int actionId, KUKADU_SHARED_PTR<Controller> actionController,
                              KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator);

        void performAction();

        KUKADU_SHARED_PTR<Controller> getActionController();

    };

}

#endif // CONTROLLERACTIONCLIP_H
