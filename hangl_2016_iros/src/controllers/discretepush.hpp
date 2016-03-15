#ifndef DISCRETEPUSH_H
#define DISCRETEPUSH_H

#include <armadillo>
#include <kukadu/kukadu.h>
#include <hangl_pushing_rect/pushingrotcontroller.hpp>

class DiscretePush : public kukadu::Controller {

private:

    int multiple;

    KUKADU_SHARED_PTR<kukadu::Controller> rot90deg;

    static std::string computeCaption(int multiple);

public:

    DiscretePush(KUKADU_SHARED_PTR<kukadu::Controller> rot90deg, int multiple);

    virtual KUKADU_SHARED_PTR<kukadu::ControllerResult> performAction();

};

#endif // NOTHINGCONTROLLER_H
