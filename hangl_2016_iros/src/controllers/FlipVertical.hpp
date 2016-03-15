#ifndef FLIPVERTICALDUMMY_H
#define FLIPVERTICALDUMMY_H

#include <armadillo>
#include <kukadu/kukadu.h>
#include <hangl_pushing_rect/utils.hpp>
#include <hangl_pushing_rect/FinalPush.hpp>
#include <hangl_pushing_rect/pushforwardcontroller.hpp>
#include <hangl_pushing_rect/pushingtranscontroller.hpp>

class FlipVertical : public kukadu::Controller {

private:

    KUKADU_SHARED_PTR<pushing::FinalPushController> finPush;
    KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward;
    KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushOrigin;

    KUKADU_SHARED_PTR<kukadu::ControlQueue> leftSimArm;
    KUKADU_SHARED_PTR<kukadu::ControlQueue> leftRealArm;

    KUKADU_SHARED_PTR<kukadu::GenericHand> leftSimHand;
    KUKADU_SHARED_PTR<kukadu::GenericHand> leftRealHand;

    KUKADU_SHARED_PTR<kukadu::GenericHand> rightRealHand;
    KUKADU_SHARED_PTR<kukadu::ControlQueue> rightRealArm;

    KUKADU_SHARED_PTR<kukadu::KomoPlanner> komoPlanner;
    ros::ServiceClient recoServ;

    KUKADU_SHARED_PTR<kukadu::Controller> rot90deg;

public:

    FlipVertical(ros::NodeHandle node,
                 KUKADU_SHARED_PTR<kukadu::Controller> rot90deg,
                 KUKADU_SHARED_PTR<pushing::FinalPushController> finPush,
                 KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward,
                 KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushOrigin,
                 KUKADU_SHARED_PTR<kukadu::KomoPlanner> komoPlanner,
                 KUKADU_SHARED_PTR<kukadu::ControlQueue> leftSimArm, KUKADU_SHARED_PTR<kukadu::GenericHand> leftSimHand,
                 KUKADU_SHARED_PTR<kukadu::ControlQueue> leftRealArm, KUKADU_SHARED_PTR<kukadu::GenericHand> leftRealHand,
                 KUKADU_SHARED_PTR<kukadu::ControlQueue> rightRealArm,
                 KUKADU_SHARED_PTR<kukadu::GenericHand> rightRealHand);

    virtual KUKADU_SHARED_PTR<kukadu::ControllerResult> performAction();

};

#endif // NOTHINGCONTROLLER_H
