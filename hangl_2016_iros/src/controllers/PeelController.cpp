#include "PeelController.hpp"

using namespace std;
using namespace kukadu;
using namespace pushing;

PeelController::PeelController(std::string caption,
                               KUKADU_SHARED_PTR<ControlQueue> leftQueue, KUKADU_SHARED_PTR<ControlQueue> rightQueue, KUKADU_SHARED_PTR<RosSchunk> leftHand, KUKADU_SHARED_PTR<RosSchunk> rightHand,
                               std::string trajectoryPath, KUKADU_SHARED_PTR<Controller> pushForward,
                               std::vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers, std::vector<KUKADU_SHARED_PTR<Controller> > preparationControllers,
                               std::string corrPSPath, string rewardHistoryPath, bool storeReward, double senseStretch, double boredom, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                               int stdReward, int punishReward, double gamma, int stdPrepWeight, bool collectPrevRewards,
                               std::string pushForwardTrajPath, double ac, double tolAbsErr, double tolRelErr, double az, double bz, double simuFailProb,
                               KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushOrigin)
                : ComplexController(caption, sensingControllers, preparationControllers, corrPSPath, rewardHistoryPath, storeReward, senseStretch, boredom, generator, stdReward, punishReward, gamma, stdPrepWeight, collectPrevRewards, simuFailProb),
                  pushForward(rightQueue, rightHand, pushForwardTrajPath, az, bz, ac, tolAbsErr, tolRelErr, false){

    this->ac = ac;
    this->tolAbsErr = tolAbsErr;
    this->tolRelErr = tolRelErr;
    this->dmpStepSize = dmpStepSize;

    this->leftQueue = leftQueue;
    this->rightQueue = rightQueue;
    this->leftHand = leftHand;
    this->rightHand = rightHand;

    this->trajectoryPath = trajectoryPath;
    this->pushOrigin = pushOrigin;

}

double PeelController::getSimulatedReward(KUKADU_SHARED_PTR<SensingController> usedSensingController, KUKADU_SHARED_PTR<kukadu::PerceptClip> providedPercept, KUKADU_SHARED_PTR<kukadu::Controller> takenAction, int sensingClassIdx, int prepContIdx) {

    /*
     * percept
     * 0: binding (closed) side
     * 1: open side
     * 2: top
     * 3: bottom
     */


    // thats the old semantic meaning, but for simulation it doesnt make a difference
    /*
     * percept
     * 0: bottom
     * 1: binding (closed) side
     * 2: open side
     * 3: top
     */

    /*
     * action
     * 0: push 99
     * 1: push 180
     * 2: push 270
     * 3: flip vertically
     * 4: do nothing
     */

    if(sensingClassIdx == 0 && prepContIdx == 2)
        return getStdReward();
    else if(sensingClassIdx == 1 && prepContIdx == 4)
        return getStdReward();
    else if(sensingClassIdx == 2 && prepContIdx == 1)
        return getStdReward();
    else if(sensingClassIdx == 3 && prepContIdx == 0)
        return getStdReward();

    return getPunishReward();

}

void PeelController::executeComplexAction() {

    if(rightQueue->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        rightQueue->stopCurrentMode();
        setStandardStiffness(rightQueue);
        rightQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }
    setStandardStiffness(rightQueue);

    if(leftQueue->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        leftQueue->stopCurrentMode();
        setStandardStiffness(leftQueue);
        leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }

    vector<double> leftHandJoints = createJointsVector(7, 0.0, -0.2, 0.0, -0.2, -1.2, -0.2, 0.0);

    goToStartPos(leftQueue);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    goToBlockingPos(rightQueue);
    goToRightHandBlocking(rightHand);

    // push forward will stop as soon as it observes high forces
    simpleJointPtp(leftQueue, stdToArmadilloVec(createJointsVector(7, -0.3506065309047699, 1.6568267345428467, 1.9634923934936523, -0.5357810258865356, -0.12806689739227295, 1.395688533782959, 1.2073975801467896)));
    pushForward.performAction();

    setHardStiffness(rightQueue);
    setHardStiffness(leftQueue);
    simpleJointPtp(leftQueue, stdToArmadilloVec(createJointsVector(7, -0.3506065309047699, 1.6568267345428467, 1.9634923934936523, -0.5357810258865356, -0.12806689739227295, 1.395688533782959, 1.2073975801467896)));

    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    leftHand->setWaitForReached(false);
    leftHandJoints = createJointsVector(7, 0.0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.3303327436948519, 1.285967300722126);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    sleep(1);

    // second finger follows
    leftHandJoints = createJointsVector(7, 0.0, -0.9303327436948519, 0.8185967300722126, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    sleep(0.5);

    leftHandJoints = createJointsVector(7, 0.0, -0.8303327436948519, 0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    sleep(0.5);

    leftHandJoints = createJointsVector(7, 0.0, 0, 0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    sleep(0.5);

    leftHandJoints = createJointsVector(7, 0.0, 0, 0.5, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    sleep(1);

    // first finger again
    leftHandJoints = createJointsVector(7, 0.0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.9303327436948519, 0.8185967300722126);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    sleep(1);

    leftHandJoints = createJointsVector(7, 0.0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.8303327436948519, 0);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    sleep(0.5);

    leftHandJoints = createJointsVector(7, 0.0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0, 0);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    sleep(0.5);

    leftHand->setWaitForReached(true);

    leftHandJoints = createJointsVector(7, 0.0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0, 0.8);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    // finally grasp it
    leftHandJoints = createJointsVector(7, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0, 0.9, SDH_IGNORE_JOINT);
    leftHand->moveJoints(stdToArmadilloVec(leftHandJoints));

    sleep(2);

    simpleJointPtp(leftQueue, stdToArmadilloVec(createJointsVector(7, -0.9291561841964722, 1.9066647291183472, 1.9648972749710083, -0.949062168598175, -0.10840536653995514, 1.199838638305664, 1.1655352115631104)));
    leftHand->setWaitForReached(true);

    simpleJointPtp(leftQueue, stdToArmadilloVec({-0.5903574228286743, 1.585422396659851, 2.48711895942688, -0.9282639622688293, -0.5973115563392639, 1.199511170387268, 1.8194438219070435}));

    goToPushHandPos(leftHand);

    pushOrigin->performAction();

}
