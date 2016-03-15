#ifndef SLIDESENSINGCONTROLLER_H
#define SLIDESENSINGCONTROLLER_H

#include <kukadu/kukadu.h>
#include <hangl_pushing_rect/utils.hpp>
#include <hangl_pushing_rect/FinalPush.hpp>
#include <hangl_pushing_rect/pushingrotcontroller.hpp>
#include <hangl_pushing_rect/pushforwardcontroller.hpp>
#include <hangl_pushing_rect/pushingtranscontroller.hpp>

class SlideSensingController : public kukadu::SensingController {

private:

    std::vector<double> handJoints;

    KUKADU_SHARED_PTR<kukadu::GenericHand> leftHand;
    KUKADU_SHARED_PTR<kukadu::GenericHand> rightHand;

    KUKADU_SHARED_PTR<kukadu::ControlQueue> leftQueue;
    KUKADU_SHARED_PTR<kukadu::ControlQueue> rightQueue;

    KUKADU_SHARED_PTR<kukadu::Controller> rot90Deg;
    KUKADU_SHARED_PTR<pushing::FinalPushController> finPush;
    KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward;
    KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushController;

public:

    SlideSensingController(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, std::string dataBasePath,
                           std::vector<KUKADU_SHARED_PTR<kukadu::ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<kukadu::GenericHand> > hands,
                           KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushController,
                           std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFun,
                           KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward,
                           KUKADU_SHARED_PTR<pushing::FinalPushController> finPush,
                           KUKADU_SHARED_PTR<kukadu::Controller> rot90Deg);

    virtual void prepare();
    virtual void cleanUp();
    virtual void performCore();
    virtual void prepareNextState();

    virtual int getSensingCatCount();

};

#endif // SLIDESENSINGCONTROLLER_H
