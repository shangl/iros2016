#include "controlqueue.hpp"

#include "../utils/utils.hpp"
#include "../types/kukadutypes.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    KUKADU_SHARED_PTR<kukadu_thread> ControlQueue::startQueueThread() {
        setInitValues();
        thr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::run, this));
        while(!this->isInitialized());
        startQueueThreadHook();
        return thr;
    }

    ControlQueue::ControlQueue(int degOfFreedom, double desiredCycleTime) {

        jointPtpRunning = false;
        cartesianPtpRunning = false;

        currentTime = 0.0;
        this->degOfFreedom = degOfFreedom;
        this->desiredCycleTime = desiredCycleTime;
        this->sleepTime = desiredCycleTime;

        continueCollecting = false;

    }

    int ControlQueue::getMovementDegreesOfFreedom() {
        return degOfFreedom;
    }

    double ControlQueue::getAbsoluteCartForce() {

        mes_result m = getCurrentCartesianFrcTrq();
        vec forces = m.joints.subvec(0, 2);
        vec prod = forces.t() * forces;
        return sqrt(prod(0));

    }

    double ControlQueue::getTimeStep() {
        return desiredCycleTime;
    }

    arma::vec ControlQueue::getStartingJoints() {
        return startingJoints;
    }

    void ControlQueue::setStartingJoints(arma::vec joints) {
        startingJoints = joints;
    }

    void ControlQueue::shutUp() {
        isShutUpFlag = true;
    }

    void ControlQueue::startTalking() {
        isShutUpFlag = false;
    }

    bool ControlQueue::isShutUp() {
        return isShutUpFlag;
    }

    void ControlQueue::addCartesianPosToQueue(geometry_msgs::Pose pose) {
        cartesianMovementQueue.push(pose);
    }

    void ControlQueue::switchMode(int mode) {
        if(ros::ok) {
            if(!isShutUp()) {
                ROS_INFO("(ControlQueue) switching control mode");
            }
            setCurrentControlTypeInternal(mode);
            currentControlType = mode;
        } else {
            if(!isShutUp())
                ROS_INFO("(ControlQueue) ros error");
        }
    }

    void ControlQueue::setInitValuesInternal() {

        isInit = false;
        finish = 0;

        currentTime = 0.0;

        rollbackMode = false;
        rollBackQueueSize = 0;

        currentJoints = arma::vec(1);

        while(!movementQueue.empty()) movementQueue.pop();
        while(!cartesianMovementQueue.empty()) cartesianMovementQueue.pop();

        setInitValues();

    }

    double ControlQueue::getCurrentTime() {
        return currentTime;
    }

    void ControlQueue::setFinish() {
        finish = 1;
        startingJoints = arma::vec(1);
    }

    bool ControlQueue::isInitialized() {
        return isInit;
    }

    void ControlQueue::addJointsPosToQueue(arma::vec joints) {
        movementQueue.push(joints);
    }

    mes_result ControlQueue::getCurrentCartesianPos() {

        mes_result ret;
        ret.joints = vec(7);
        ret.joints(0) = currentCartPose.position.x;
        ret.joints(1) = currentCartPose.position.y;
        ret.joints(2) = currentCartPose.position.z;
        ret.joints(3) = currentCartPose.orientation.x;
        ret.joints(4) = currentCartPose.orientation.y;
        ret.joints(5) = currentCartPose.orientation.z;
        ret.joints(6) = currentCartPose.orientation.w;
        ret.time = getCurrentTime();
        return ret;

    }

    bool ControlQueue::getQueueRunning() {
        return !finish;
    }

    void ControlQueue::synchronizeToControlQueue(int maxNumJointsInQueue) {
        if(currentControlType == CONTROLQUEUE_JNT_IMP_MODE || currentControlType == CONTROLQUEUE_JNT_POS_MODE) {
            while(movementQueue.size() > maxNumJointsInQueue);
        } else if(currentControlType == CONTROLQUEUE_CART_IMP_MODE) {
            while(cartesianMovementQueue.size() > maxNumJointsInQueue);
        }
    }

    void ControlQueue::stopCurrentMode() {
        switchMode(CONTROLQUEUE_STOP_MODE);
        switchMode(CONTROLQUEUE_JNT_POS_MODE);
        switchMode(CONTROLQUEUE_STOP_MODE);
    }

    void ControlQueue::internalJointPtpCaller() {
        jointPtp(internalJointPasser);
    }

    KUKADU_SHARED_PTR<kukadu_thread> ControlQueue::jointPtpNb(arma::vec joints) {

        internalJointPasser = joints;
        jointPtpThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::internalJointPtpCaller, this));
        return jointPtpThr;

    }

    void ControlQueue::internalCartPtpCaller() {
        cartesianPtp(internalPosePasser);
    }

    KUKADU_SHARED_PTR<kukadu_thread> ControlQueue::cartesianPtpNb(geometry_msgs::Pose pos) {

        internalPosePasser = pos;
        cartPtpThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::internalJointPtpCaller, this));
        return jointPtpThr;

    }

    double ControlQueue::getMeasuredTimeStep() {
        return lastDuration;
    }

    void ControlQueue::run() {

        setInitValuesInternal();
        ros::Rate sleepRate(1.0 / sleepTime);

        arma::vec movement = arma::vec(1);
        geometry_msgs::Pose movementPose;

        if(getStartingJoints().n_elem > 1) {

            if(!isShutUp())
                ROS_INFO("(ControlQueue) start moving to start position");
            jointPtp(getStartingJoints());

            if(!isShutUp())
                ROS_INFO("(ControlQueue) finished moving to start position");

        }

        isInit = true;

        lastDuration = 0.0;
        double toleratedMaxDuration = 1.1 * desiredCycleTime;
        double toleratedMinDuration = 0.9 * desiredCycleTime;

        movement = getCurrentJoints().joints;

        t.tic("r");

        while(!finish && ros::ok) {

            t.tic("d");

            /*
            // switched off adaptive cycle time behaviour for now - probably requires a PID controller for that
            if(toleratedMinDuration < lastDuration || lastDuration > toleratedMaxDuration) {

                sleepTime = max(0.000000001, sleepTime + 0.3 * (desiredCycleTime -  lastDuration));
                sleepRate = ros::Rate(1.0 / sleepTime);

            }
            */

            currentTime = t.toc("r");

            currentJoints = getCurrentJoints().joints;
            currentCartPose = getCurrentCartesianPose();

            if(rollbackMode) {

                rollBackQueue.push_front(currentJoints);

                // if queue is full --> go back
                while(rollBackQueue.size() > rollBackQueueSize)
                    rollBackQueue.pop_back();

            }

            if(!stopQueueWhilePtp() || !jointPtpRunning && !cartesianPtpRunning) {

                if(currentControlType == CONTROLQUEUE_JNT_IMP_MODE || currentControlType == CONTROLQUEUE_JNT_POS_MODE) {

                    if(movementQueue.size() > 0) {

                        // move to position in queue
                        movement = movementQueue.front();
                        movementQueue.pop();

                    } else {

                        if(stopQueueWhilePtp())
                            movement = getCurrentJoints().joints;

                    }

                    submitNextJointMove(movement);

                } else if(currentControlType == CONTROLQUEUE_CART_IMP_MODE) {

                    if(cartesianMovementQueue.size() > 0) {

                        // move to position in queue
                        movementPose = cartesianMovementQueue.front();
                        cartesianMovementQueue.pop();

                    } else {

                        movementPose = getCurrentCartesianPose();

                    }

                    submitNextCartMove(movementPose);

                }

            }

            sleepRate.sleep();

            ros::spinOnce();

            lastDuration = t.toc("d");

        }

        if(!isShutUp())
            cout << "thread finished" << endl;

    }

    void ControlQueue::jointsCollector() {

        double lastTime = DBL_MIN;
        collectedJoints.clear();
        ros::Rate r((int) (1.0 / getTimeStep() + 10.0));
        while(continueCollecting) {
            mes_result lastResult = getCurrentJoints();
            if(lastResult.time != lastTime)
                collectedJoints.push_back(lastResult);
            r.sleep();
        }

    }

    std::vector<mes_result> ControlQueue::jointPtp(arma::vec joints) {
        jointPtpRunning = true;

        if(!continueCollecting) {

            continueCollecting = true;
            jointsColletorThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::jointsCollector, this));
            jointPtpInternal(joints);
            continueCollecting = false;
            jointsColletorThr->join();

        } else {

            throw KukaduException("(ControlQueue) only one ptp at a time can be executed");

        }

        jointPtpRunning = false;
        return collectedJoints;

    }

    std::vector<mes_result> ControlQueue::cartesianPtp(geometry_msgs::Pose pos, double maxForce) {

        cartesianPtpRunning = true;

        if(!continueCollecting) {

            continueCollecting = true;
            jointsColletorThr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&ControlQueue::jointsCollector, this));
            cartPtpInternal(pos, maxForce);
            continueCollecting = false;
            jointsColletorThr->join();

        } else {

            throw KukaduException("(ControlQueue) only one ptp at a time can be executed");

        }

        cartesianPtpRunning = false;
        return collectedJoints;

    }

    std::vector<mes_result> ControlQueue::cartesianPtp(geometry_msgs::Pose pos) {

        return cartesianPtp(pos, DBL_MAX);

    }

    void ControlQueue::startJointRollBackMode(double possibleTime) {

        rollBackQueue.clear();
        rollbackMode = true;
        rollBackTime = possibleTime;
        // buffer of 1.0 more second
        rollBackQueueSize = (int) ((possibleTime + 1.0) / desiredCycleTime);

    }

    void ControlQueue::stopJointRollBackMode() {

        rollbackMode = false;
        rollBackQueue.clear();

    }

    void ControlQueue::rollBack(double time) {

        rollbackMode = false;
        int rollBackCount = (int) (time / desiredCycleTime);

        int stretchFactor = ceil((double) rollBackCount / (double) rollBackQueue.size());
        stretchFactor = max((double) stretchFactor, 1.0);

        vec lastCommand(getMovementDegreesOfFreedom());
        if(rollBackQueue.size())
            lastCommand = rollBackQueue.front();

        int newRollBackCount = ceil((double) rollBackCount / (double) stretchFactor);

        // fill command queue with last commands (backwards)
        for(int i = 0; i < newRollBackCount && rollBackQueue.size(); ++i) {

            vec nextCommand = rollBackQueue.front();

            // interpolate to stretch the trajectory in case there are not enough measured packets (happens in usage with simulator)
            vec diffUnit = (nextCommand - lastCommand) / (double) stretchFactor;
            for(int j = 0; j < stretchFactor; ++j) {
                addJointsPosToQueue(lastCommand + j * diffUnit);
            }

            lastCommand = nextCommand;
            rollBackQueue.pop_front();

        }

        // wait until everything has been executed
        synchronizeToControlQueue(1);
        rollBackQueue.clear();

    }

}
