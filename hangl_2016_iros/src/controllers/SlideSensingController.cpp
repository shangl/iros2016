#include "SlideSensingController.hpp"

using namespace std;
using namespace kukadu;
using namespace pushing;

SlideSensingController::SlideSensingController(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, string dataBasePath,
                                               std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands,
                                               KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushController,
                                               std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFun,
                                               KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward,
                                               KUKADU_SHARED_PTR<pushing::FinalPushController> finPush,
                                               KUKADU_SHARED_PTR<kukadu::Controller> rot90Deg)
    : SensingController(generator, hapticMode, "sliding", dataBasePath, queues, hands, tmpPath, classifierPath, classifierFile, classifierFun, 95) {

    handJoints = {0.0, -1.5, -1.0, 0.0, -0.2, -1.5, -1.0};

    this->leftHand = hands.at(0);
    this->rightHand = hands.at(1);
    this->leftQueue = queues.at(0);
    this->rightQueue = queues.at(1);

    this->finPush = finPush;
    this->rot90Deg = rot90Deg;
    this->pushForward = pushForward;
    this->pushController = pushController;

}

void SlideSensingController::prepareNextState() {
    rot90Deg->performAction();
}

int SlideSensingController::getSensingCatCount() {
    return 4;
}

void SlideSensingController::prepare() {

    if(rightQueue->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        rightQueue->stopCurrentMode();
        setStandardStiffness(rightQueue);
        rightQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    if(leftQueue->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        leftQueue->stopCurrentMode();
        setStandardStiffness(leftQueue);
        leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    setStandardStiffness(rightQueue);
    setStandardStiffness(leftQueue);

    goToStartPos(leftQueue);
    goToPushHandPos(leftHand);

    goToBlockingPos(rightQueue);
    goToRightHandBlocking(rightHand);

    pushController->performAction();
    finPush->performAction();

    // move finger a bit forward
    leftHand->moveJoints(stdToArmadilloVec(handJoints));
    simpleJointPtp(leftQueue, {-0.40275293588638306, 1.7016545534133911, 1.8671916723251343, -0.6587858200073242, 0.0556875579059124, 1.1993221044540405, -1.9818705320358276});
    leftHand->moveJoints(stdToArmadilloVec(createJointsVector(7, 0.0, -1.5, -1.0, 0.1, -0.1, -1.5, -1)));

    // push forward will stop as soon as it observes high forces
    pushForward->setGoBackToBlockingPos(false);
    pushForward->performAction();

    leftHand->moveJoints(stdToArmadilloVec(handJoints));

    setHardStiffness(rightQueue);

}

void SlideSensingController::cleanUp() {

    setStandardStiffness(rightQueue);
    goToBlockingPos(rightQueue);
    moveHome(leftQueue, KUKADU_SHARED_PTR<ControlQueue>(), false);
    goToPushHandPos(leftHand);

    pushForward->setGoBackToBlockingPos(true);
    pushForward->performAction();

}

void SlideSensingController::performCore() {

    sleep(1);

    vector<double> newHandJoints = {SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0.4, 1.2, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->moveJoints(stdToArmadilloVec(newHandJoints));

    sleep(4);

}
