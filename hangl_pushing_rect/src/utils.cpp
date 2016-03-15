#include <hangl_pushing_rect/utils.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

double computeMaxJointDistance(arma::vec joints1, arma::vec joints2) {

    double maxDist = 0.0;
    for(int i = 0; i < joints1.n_elem; ++i) {
        double currDist = abs(joints1(i) - joints2(i));
        maxDist = max(currDist, maxDist);
    }
    return maxDist;

}

std::vector<arma::vec> smoothJointPlan(std::vector<arma::vec> jointPlan) {

    vector<vec> smoothedPlan;

    if(jointPlan.size()) {
        vec lastUsedJoints = jointPlan.at(0);
        smoothedPlan.push_back(lastUsedJoints);
        for(vec joints : jointPlan) {
            if(computeMaxJointDistance(lastUsedJoints, joints) > 0.001) {
                lastUsedJoints = joints;
                smoothedPlan.push_back(lastUsedJoints);
            }
        }
    }

    return smoothedPlan;

}

void pushForward(KUKADU_SHARED_PTR<ControlQueue> rightQueue, double maxForce) {

    goToBlockingPos(rightQueue);

    geometry_msgs::Pose targetPose;
    targetPose.position.x = -0.17; targetPose.position.y = 0.71; targetPose.position.z = 0.29;
    tf::Quaternion rot = rpyToQuat(M_PI, M_PI, -M_PI / 2.0);
    targetPose.orientation.x = rot.getX();
    targetPose.orientation.y = rot.getY();
    targetPose.orientation.z = rot.getZ();
    targetPose.orientation.w = rot.getW();

    ros::Rate r(3);
    double observedForce = 0.0;
    for(; targetPose.position.x < 0.2 && (observedForce = rightQueue->getAbsoluteCartForce()) < maxForce; targetPose.position.x += 0.02) {
        rightQueue->cartesianPtp(targetPose, maxForce);
        r.sleep();
    }

    cout << "max force exceeded with " << observedForce << endl;

}

void simpleJointPtp(KUKADU_SHARED_PTR<ControlQueue> rightQueue, arma::vec joints, double maxForce) {

    SimplePlanner sp(rightQueue, KUKADU_SHARED_PTR<Kinematics>());
    rightQueue->startJointRollBackMode(1.5);
    vector<vec> desiredJoints; desiredJoints.push_back(rightQueue->getCurrentJoints().joints); desiredJoints.push_back(joints);
    vector<vec> jointPlan = sp.planJointTrajectory(desiredJoints);

    for(vec joint : jointPlan) {
        if(rightQueue->getAbsoluteCartForce() > maxForce) {
            rightQueue->rollBack(1.0);
            break;
        } else {
            rightQueue->addJointsPosToQueue(joint);
            rightQueue->synchronizeToControlQueue(1);
        }
    }

    rightQueue->stopJointRollBackMode();


}

// normalize it such that yaw is 0 when the book is horizontal (rotation count clockwise from the horicontal position seen from the robot)
double computeBookOrientation(double angle) {

    double yaw = angle;

    yaw = fmod(yaw + M_PI / 2.0, M_PI);
    cout << "yaw after first calculation: " << yaw << endl;
    if(yaw > M_PI / 2.0)
        yaw = yaw - M_PI;

    return yaw;

}

void goToBlockingPos(KUKADU_SHARED_PTR<ControlQueue> rightQueue, bool waitForConfirmation) {

    //vec moveTo = stdToArmadilloVec(createJointsVector(7, -2.3800294399261475, 1.5282957553863525, -2.280046224594116, 1.884490966796875, 2.1091063022613525, -1.4556314945220947, -0.7266652584075928));
    vec moveTo = stdToArmadilloVec(createJointsVector(7, -2.4132, 1.62996, -2.22251, 2.01567, 2.18936, -1.65823, -0.956807));
    vec currentState = rightQueue->getCurrentJoints().joints;
    for(int i = 0; i < moveTo.n_elem; ++i)
        if(abs(currentState(i) - moveTo(i)) > 0.01) {
            simpleJointPtp(rightQueue, moveTo);
            break;
        }

    /*
    if(waitForConfirmation)
        cout << "press enter to move " << rightQueue->getRobotName() << " arm" << endl; getchar();

    // old joint position (not reachable anymore with komo - disabled collision checking for this case)
    //rightQueue->jointPtp(stdToArmadilloVec(createJointsVector(7, -2.66, 1.43, 0.97, -1.90, -1.79, -0.92, -0.25)));
    rightQueue->jointPtp(stdToArmadilloVec(createJointsVector(7, -2.51, 1.5, 0.89, -1.87, -1.63, -1.0, -0.45)));

    geometry_msgs::Pose targetPose;
    targetPose.position.x = -0.17; targetPose.position.y = 0.71; targetPose.position.z = 0.29;

    tf::Quaternion rot = rpyToQuat(M_PI, M_PI, -M_PI / 2.0);
    targetPose.orientation.x = rot.getX();
    targetPose.orientation.y = rot.getY();
    targetPose.orientation.z = rot.getZ();
    targetPose.orientation.w = rot.getW();
    rightQueue->cartesianPtp(targetPose);
    */

}

void goToStartPos(KUKADU_SHARED_PTR<ControlQueue> leftQueue) {

    //cout << "press enter to move " << leftQueue->getRobotName() << " arm" << endl; getchar();
    moveHome(leftQueue, KUKADU_SHARED_PTR<ControlQueue>(), false);

}

void goToPushHandPos(KUKADU_SHARED_PTR<GenericHand> leftHand) {

    //cout << "press enter to move " << leftHand->getHandName() << " hand" << endl; getchar();
    vector<double> pushHandPos = {0.0, -1.57, 0.0, -1.57, 0.0, -1.57, 0.0};
    leftHand->moveJoints(stdToArmadilloVec(pushHandPos));

}

void goToFlipHandPos(KUKADU_SHARED_PTR<GenericHand> leftHand) {
    vector<double> flipHandPos = createJointsVector(7, 0.0, -1.57, -0.6, -1.57, -0.6, -1.57, -0.6);
    leftHand->moveJoints(stdToArmadilloVec(flipHandPos));
}

void setHardStiffness(KUKADU_SHARED_PTR<ControlQueue> queue) {

    queue->setStiffness(500, 200, 0.5, 15000, 150, 1500);

}

void setStandardStiffness(KUKADU_SHARED_PTR<ControlQueue> queue) {

    queue->setStiffness(KukieControlQueue::KUKA_STD_XYZ_STIFF, KukieControlQueue::KUKA_STD_ABC_STIFF, KukieControlQueue::KUKA_STD_CPDAMPING, 15000, 150, 1500);

}

void setWeakStiffness(KUKADU_SHARED_PTR<ControlQueue> queue) {

    queue->setStiffness(50, 1, 0.05, 15000, 150, 1500);

}

void goToRightHandBlocking(KUKADU_SHARED_PTR<GenericHand> rightHand) {

    //vector<double> rightHandJoints = createJointsVector(7, 0.0039448284307002125, 0.13004066632283443, -0.8298513154179146, 1.0651348732446628, 1.5405204462584399, 0.18563213174028576, -0.8880457128729173);
    vector<double> rightHandJoints = createJointsVector(7, 0.0039448284307002125, 0.13004066632283443, -0.6598513154179146, 1.0651348732446628, 1.5405204462584399, 0.18563213174028576, -0.888045712872917);
    rightHand->moveJoints(stdToArmadilloVec(rightHandJoints));

}

std::vector<KUKADU_SHARED_PTR<GenericHand> > createHandList(KUKADU_SHARED_PTR<GenericHand> hand1) {
    std::vector<KUKADU_SHARED_PTR<GenericHand> > ret;
    ret.push_back(hand1);
    return ret;
}

std::vector<KUKADU_SHARED_PTR<ControlQueue> > createQueueList(KUKADU_SHARED_PTR<ControlQueue> queue1, KUKADU_SHARED_PTR<ControlQueue> queue2) {
    std::vector<KUKADU_SHARED_PTR<ControlQueue> > ret;
    ret.push_back(queue1); ret.push_back(queue2);
    return ret;
}

void ptpWithSimulation(geometry_msgs::Pose pose,
                                                     KUKADU_SHARED_PTR<kukadu::ControlQueue>  simulationQueue,
                                                     KUKADU_SHARED_PTR<kukadu::ControlQueue>  executionQueue,
                                                     bool useReal) {

    vector<mes_result> jointPlan = simulationQueue->cartesianPtp(pose);

    if(useReal) {
        cout << "press key to execute plan on the real robot" << endl;
    //    getchar();
        for(mes_result jp : jointPlan) {
            executionQueue->addJointsPosToQueue(jp.joints);
        }
        executionQueue->synchronizeToControlQueue(1);
    }

}

void jointPtpWithSimulation(arma::vec joints,
                                                     KUKADU_SHARED_PTR<kukadu::ControlQueue>  simulationQueue,
                                                     KUKADU_SHARED_PTR<kukadu::ControlQueue>  executionQueue,
                                                     bool useReal) {


    vector<mes_result> jointPlan = simulationQueue->jointPtp(joints);

    if(useReal) {
        cout << "press key to execute plan on the real robot" << endl;
    //    getchar();
        for(mes_result jp : jointPlan) {
            executionQueue->addJointsPosToQueue(jp.joints);
        }
        executionQueue->synchronizeToControlQueue(1);
    }

}

void moveHome(KUKADU_SHARED_PTR<kukadu::ControlQueue> simQueue, KUKADU_SHARED_PTR<kukadu::ControlQueue> realQueue, bool useReal) {

    if(useReal)
        simQueue->jointPtp(realQueue->getCurrentJoints().joints);

    vec startVec = stdToArmadilloVec(createJointsVector(7, -1.4971023797988892, 1.5505013465881348, 2.334461212158203, -1.7423652410507202, -1.8476978540420532, 1.268733024597168, 0.7063689231872559));

    jointPtpWithSimulation(startVec, simQueue, realQueue, useReal);

}
