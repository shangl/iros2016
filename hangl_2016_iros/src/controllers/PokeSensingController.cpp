#include "PokeSensingController.hpp"

#include <hangl_pushing_rect/utils.hpp>

using namespace std;
using namespace kukadu;
using namespace pushing;

PokeSensingController::PokeSensingController(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, string dataBasePath,
                                             KUKADU_SHARED_PTR<ControlQueue> leftQueue, KUKADU_SHARED_PTR<ControlQueue> rightQueue,
                                             KUKADU_SHARED_PTR<RosSchunk> leftHand, KUKADU_SHARED_PTR<RosSchunk> rightHand,
                                               KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushController,
                                               std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFun,
                                             KUKADU_SHARED_PTR<FinalPushController> finPush, KUKADU_SHARED_PTR<kukadu::Controller> rot90Deg)
    : SensingController(generator, hapticMode, "poking", dataBasePath, createQueueList(leftQueue, rightQueue), createHandList(leftHand), tmpPath, classifierPath, classifierFile, classifierFun, 95) {

    this->leftHand = leftHand;
    this->rightHand = rightHand;
    this->leftQueue = leftQueue;
    this->rightQueue = rightQueue;

    this->finPush = finPush;
    this->rot90Deg = rot90Deg;

}

void PokeSensingController::prepareNextState() {
    rot90Deg->performAction();
}

int PokeSensingController::getSensingCatCount() {
    return 4;
}

void PokeSensingController::prepare() {

    if(rightQueue->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        rightQueue->stopCurrentMode();
        setStandardStiffness(rightQueue);
        rightQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }
    setStandardStiffness(rightQueue);

    goToStartPos(leftQueue);
    goToPushHandPos(leftHand);

    goToBlockingPos(rightQueue);
    goToRightHandBlocking(rightHand);

    //simpleJointPtp(rightQueue, stdToArmadilloVec({-2.2548043727874756, 1.5036139488220215, -2.1158950328826904, 1.7510524988174438, 2.1929965019226074, -1.2221717834472656, -0.7233961820602417}));
    //simpleJointPtp(rightQueue, stdToArmadilloVec({-2.3100, 1.5146, -2.1888, 1.8090, 2.1556, -1.3247, -0.7234}));
    //goToBlockingPos(rightQueue);

    finPush->prepare();

}

void PokeSensingController::cleanUp() {

    finPush->cleanUp();

    goToBlockingPos(rightQueue);
    goToStartPos(leftQueue);

}

void PokeSensingController::performCore() {

    sleep(1);

    finPush->performCore();

}
