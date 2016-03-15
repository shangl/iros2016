#include "controllerresult.hpp"

namespace kukadu {

    ControllerResult::ControllerResult(arma::vec t, std::vector<arma::vec> ys) {
        construct(t, ys, true, false);
    }

    ControllerResult::ControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success, bool bored) {
        construct(t, ys, success, bored);
    }

    arma::vec ControllerResult::getTimes() {
        return t;
    }

    std::vector<arma::vec> ControllerResult::getYs() {
        return y;
    }

    void ControllerResult::construct(arma::vec t, std::vector<arma::vec> ys, bool success, bool bored) {
        this->t = t;
        this->y = ys;
        this->bored = bored;
        this->success = success;
    }

    void ControllerResult::setSuccess(bool success) {
        this->success = success;
    }

    bool ControllerResult::getSuccess() {
        return success;
    }

    bool ControllerResult::wasBored() {
        return bored;
    }

}
