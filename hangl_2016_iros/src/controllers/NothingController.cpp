#include "NothingController.hpp"

using namespace std;
using namespace kukadu;

NothingController::NothingController()
    : Controller("do nothing", 0) {

}

KUKADU_SHARED_PTR<ControllerResult> NothingController::performAction() {

}

