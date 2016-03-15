#include "PressSensingController.hpp"

#include <hangl_pushing_rect/utils.hpp>

using namespace std;
using namespace kukadu;

PressSensingController::PressSensingController(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, string dataBasePath,
                                               KUKADU_SHARED_PTR<ControlQueue> leftQueue, KUKADU_SHARED_PTR<ControlQueue> rightQueue,
                                               KUKADU_SHARED_PTR<RosSchunk> leftHand, KUKADU_SHARED_PTR<RosSchunk> rightHand,
                                               std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFun,
                                               KUKADU_SHARED_PTR<kukadu::Controller> rot90Deg,
                                               KUKADU_SHARED_PTR<kukadu::Controller> finPush)
    : SensingController(generator, hapticMode, "pressing", dataBasePath, createQueueList(leftQueue, rightQueue), createHandList(leftHand), tmpPath, classifierPath, classifierFile, classifierFun, 95) {

    this->leftHand = leftHand;
    this->rightHand = rightHand;
    this->leftQueue = leftQueue;
    this->rightQueue = rightQueue;

    this->finPush = finPush;
    this->rot90Deg = rot90Deg;

}

void PressSensingController::prepareNextState() {
    rot90Deg->performAction();
}

int PressSensingController::getSensingCatCount() {
    return 4;
}

void PressSensingController::prepare() {

    if(leftQueue->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        leftQueue->stopCurrentMode();
        setStandardStiffness(leftQueue);
        leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    setStandardStiffness(leftQueue);

    goToStartPos(leftQueue);
    goToPushHandPos(leftHand);

    goToBlockingPos(rightQueue);
    goToRightHandBlocking(rightHand);

    finPush->performAction();

    moveHome(leftQueue, KUKADU_SHARED_PTR<ControlQueue>(), false);
    simpleJointPtp(leftQueue, stdToArmadilloVec({-0.9841417670249939, 1.8732956647872925, 2.116145133972168, -1.1639715433120728, -0.2766830325126648, 1.2711533308029175, -1.6481853723526}));
    leftHand->moveJoints(stdToArmadilloVec(createJointsVector(7, 7.718142037627508e-05, -1.4937496117203046, -0.9756279597988899, 0.3323829438032553, 0.28556264104787193, -1.4902725886010972, -0.9776301908854388)));
    simpleJointPtp(leftQueue, stdToArmadilloVec({-0.770733118057251, 2.0831286907196045, 2.1464719772338867, -1.287307620048523, -0.5006093978881836, 1.1945735216140747, -1.738010048866272}));

}

void PressSensingController::cleanUp() {

    setStandardStiffness(leftQueue);
    moveHome(leftQueue, KUKADU_SHARED_PTR<ControlQueue>(), false);

}

void PressSensingController::performCore() {

    sleep(1);

    simpleJointPtp(leftQueue, {-0.4976746082305908, 1.9, 2.1710205078125, -1.2850295305252075, -0.5185016989707947, 1.0343561172485352, -1.7773977518081665}, 8);

    sleep(2);

    simpleJointPtp(leftQueue, {-0.770733118057251, 2.0831286907196045, 2.1464719772338867, -1.287307620048523, -0.5006093978881836, 1.1945735216140747, -1.738010048866272});

}
