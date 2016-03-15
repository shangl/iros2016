#ifndef POKESENSINGCONTROLLER_H
#define POKESENSINGCONTROLLER_H

#include <kukadu/kukadu.h>
#include <hangl_pushing_rect/FinalPush.hpp>
#include <hangl_pushing_rect/pushingrotcontroller.hpp>
#include <hangl_pushing_rect/pushforwardcontroller.hpp>
#include <hangl_pushing_rect/pushingtranscontroller.hpp>

class PokeSensingController : public kukadu::SensingController {

private:

    KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushController;

    KUKADU_SHARED_PTR<kukadu::RosSchunk> leftHand;
    KUKADU_SHARED_PTR<kukadu::RosSchunk> rightHand;

    KUKADU_SHARED_PTR<kukadu::ControlQueue> leftQueue;
    KUKADU_SHARED_PTR<kukadu::ControlQueue> rightQueue;

    KUKADU_SHARED_PTR<pushing::FinalPushController> finPush;
    KUKADU_SHARED_PTR<kukadu::Controller> rot90Deg;

public:

    PokeSensingController(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, std::string dataBasePath, KUKADU_SHARED_PTR<kukadu::ControlQueue> leftQueue,
                          KUKADU_SHARED_PTR<kukadu::ControlQueue> rightQueue,
                          KUKADU_SHARED_PTR<kukadu::RosSchunk> leftHand, KUKADU_SHARED_PTR<kukadu::RosSchunk> rightHand,
                           KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushController,
                           std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFun,
                          KUKADU_SHARED_PTR<pushing::FinalPushController> finPush, KUKADU_SHARED_PTR<kukadu::Controller> rot90Deg);

    virtual void prepare();
    virtual void cleanUp();
    virtual void performCore();
    virtual void prepareNextState();

    virtual int getSensingCatCount();

};

#endif // POKESENSINGCONTROLLER_H
