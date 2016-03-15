#ifndef PRESSSENSINGCONTROLLER_H
#define PRESSSENSINGCONTROLLER_H

#include <kukadu/kukadu.h>
#include <hangl_pushing_rect/pushingrotcontroller.hpp>

class PressSensingController : public kukadu::SensingController {

private:

    KUKADU_SHARED_PTR<kukadu::RosSchunk> leftHand;
    KUKADU_SHARED_PTR<kukadu::RosSchunk> rightHand;

    KUKADU_SHARED_PTR<kukadu::ControlQueue> leftQueue;
    KUKADU_SHARED_PTR<kukadu::ControlQueue> rightQueue;

    KUKADU_SHARED_PTR<kukadu::Controller> finPush;
    KUKADU_SHARED_PTR<kukadu::Controller> rot90Deg;

public:

    PressSensingController(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, std::string dataBasePath, KUKADU_SHARED_PTR<kukadu::ControlQueue> leftQueue,
                           KUKADU_SHARED_PTR<kukadu::ControlQueue> rightQueue, KUKADU_SHARED_PTR<kukadu::RosSchunk> leftHand, KUKADU_SHARED_PTR<kukadu::RosSchunk> rightHand,
                           std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFun,
                           KUKADU_SHARED_PTR<kukadu::Controller> rot90Deg,
                           KUKADU_SHARED_PTR<kukadu::Controller> finPush);

    virtual void prepare();
    virtual void cleanUp();
    virtual void performCore();
    virtual void prepareNextState();

    virtual int getSensingCatCount();

};

#endif // PRESSSENSINGCONTROLLER_H
