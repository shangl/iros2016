#include "discretepush.hpp"

using namespace std;
using namespace arma;
using namespace kukadu;

DiscretePush::DiscretePush(KUKADU_SHARED_PTR<kukadu::Controller> rot90deg, int multiple) : Controller(computeCaption(multiple), 0) {

    this->rot90deg = rot90deg;
    this->multiple = multiple;

}

std::string DiscretePush::computeCaption(int multiple) {
    stringstream s;
    s << "rot" << (multiple * 90) << "deg";
    return s.str();
}

KUKADU_SHARED_PTR<ControllerResult> DiscretePush::performAction() {

    for(int i = 0; i < multiple; ++i)
        rot90deg->performAction();

}
