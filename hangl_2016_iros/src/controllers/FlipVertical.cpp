#include "FlipVertical.hpp"

using namespace std;
using namespace arma;
using namespace kukadu;

FlipVertical::FlipVertical(ros::NodeHandle node,
                           KUKADU_SHARED_PTR<kukadu::Controller> rot90deg,
                           KUKADU_SHARED_PTR<pushing::FinalPushController> finPush,
                           KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward,
                           KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushOrigin,
                           KUKADU_SHARED_PTR<kukadu::KomoPlanner> komoPlanner,
                           KUKADU_SHARED_PTR<kukadu::ControlQueue> leftSimArm, KUKADU_SHARED_PTR<kukadu::GenericHand> leftSimHand,
                           KUKADU_SHARED_PTR<kukadu::ControlQueue> leftRealArm, KUKADU_SHARED_PTR<kukadu::GenericHand> leftRealHand,
                           KUKADU_SHARED_PTR<kukadu::ControlQueue> rightRealArm,
                           KUKADU_SHARED_PTR<kukadu::GenericHand> rightRealHand) : Controller("flip vertical", 0) {

    this->finPush = finPush;
    this->pushForward = pushForward;
    this->pushOrigin = pushOrigin;

    this->leftRealArm = leftRealArm;
    this->leftSimArm = leftSimArm;

    this->leftSimHand = leftSimHand;
    this->leftRealHand = leftRealHand;

    this->rightRealHand = rightRealHand;
    this->rightRealArm = rightRealArm;

    this->komoPlanner = komoPlanner;

    this->rot90deg = rot90deg;

    recoServ = node.serviceClient<hangl_vision::visionservice>("recognize_book");
    while(!recoServ.exists()) {
        ROS_WARN("Waiting for planning service to come up...");
        recoServ.waitForExistence(ros::Duration(10));
    }

}

KUKADU_SHARED_PTR<ControllerResult> FlipVertical::performAction() {

    hangl_vision::visionserviceRequest bookRequest;
    hangl_vision::visionserviceResponse bookResponse;

    while(!recoServ.call(bookRequest, bookResponse)) { // communication failure
        ROS_ERROR("Error on planning request!");
        return KUKADU_SHARED_PTR<kukadu::ControllerResult>();
    }

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf::Quaternion quat(bookResponse.coordinates.at(6), bookResponse.coordinates.at(7), bookResponse.coordinates.at(8), bookResponse.coordinates.at(9));
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    double bookOrientation = computeBookOrientation(yaw);

    // if horizontal, rotate by 90 degrees
    if(bookOrientation > -0.3 && bookOrientation < 0.3)
        rot90deg->performAction();

    if(leftSimArm->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        leftSimArm->stopCurrentMode();
        setStandardStiffness(leftSimArm);
        leftSimArm->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }
    setStandardStiffness(leftSimArm);

    if(leftRealArm->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
        leftRealArm->stopCurrentMode();
        setStandardStiffness(leftRealArm);
        leftRealArm->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    }
    setStandardStiffness(leftRealArm);

    pushForward->setGoBackToBlockingPos(false);
    pushForward->performAction();

    goToPushHandPos(leftSimHand);
    goToPushHandPos(leftRealHand);

    finPush->prepare();
    finPush->performCore();

    geometry_msgs::Pose currPose = leftRealArm->getCurrentCartesianPose();
    vec realPos = leftRealArm->getCurrentJoints().joints;
    simpleJointPtp(leftSimArm, realPos);

    vec lastJoints = realPos;
    vector<vec> completePlan;

    geometry_msgs::Pose nextPose = currPose;

    // lift the book
    for(double i = 0.0; i < M_PI / 2.0 + 0.2; i += 0.1) {
        nextPose = currPose;
        nextPose.position.z += 0.18 * sin(i);
        nextPose.position.x -= 0.18 * (1.0 - cos(i));
        vector<geometry_msgs::Pose> poses;
        poses.push_back(nextPose);
        vector<vec> jointPlan = komoPlanner->planCartesianTrajectory(lastJoints, poses);
        for(vec joints : jointPlan) {
            completePlan.push_back(joints);
            lastJoints = joints;
        }

    }

    completePlan = smoothJointPlan(completePlan);

    goToFlipHandPos(leftSimHand);
    for(vec jp : completePlan)
        leftSimArm->addJointsPosToQueue(jp);
    leftSimArm->synchronizeToControlQueue(1);

    // cout << "(FlipVertical) press key to execute on real robot" << endl; getchar();
    goToFlipHandPos(leftRealHand);
    for(vec jp : completePlan)
        leftRealArm->addJointsPosToQueue(jp);
    leftRealArm->synchronizeToControlQueue(1);

    // move fingers of right hand to move book forward
    rightRealHand->moveJoints(stdToArmadilloVec({0.0, 0.14, 0.8, 1.06, 1.54, 0.18, 0.8})); sleep(0.5);
    rightRealHand->moveJoints(stdToArmadilloVec({0.0, 0.14, 0.0, 1.06, 1.54, 0.14, 0.0})); sleep(0.5);
    rightRealHand->moveJoints(stdToArmadilloVec({0.0, 0.4, 0, 1.06, 1.54, 0.4, 0})); sleep(0.5);

    vector<double> flipHandPos = createJointsVector(7, 0.0, -1.57, -0.9, -1.57, -0.9, -1.57, -0.9);
    leftRealHand->moveJoints(stdToArmadilloVec(flipHandPos));

    goToBlockingPos(rightRealArm);

    nextPose.position.x -= 0.05;
    leftRealArm->cartesianPtp(nextPose);

    // get fingers away from bock
    nextPose.position.z += 0.2;
    leftRealArm->cartesianPtp(nextPose);

    nextPose.position.x += 0.2;
    leftRealArm->cartesianPtp(nextPose);

    moveHome(leftRealArm, leftSimArm, false);

    pushForward->setMaxForce(40);
    pushForward->setGoBackToBlockingPos(true);
    pushForward->performAction();
    pushForward->setMaxForce(8);

    pushOrigin->performAction();

    finPush->performAction();

}
