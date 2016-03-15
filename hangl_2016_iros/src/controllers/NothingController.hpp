#ifndef NOTHINGCONTROLLER_H
#define NOTHINGCONTROLLER_H

#include <armadillo>
#include <kukadu/kukadu.h>

class NothingController : public kukadu::Controller {

private:

public:

    NothingController();

    virtual KUKADU_SHARED_PTR<kukadu::ControllerResult> performAction();



};

#endif // NOTHINGCONTROLLER_H
