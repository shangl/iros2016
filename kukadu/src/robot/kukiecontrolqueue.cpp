#include "kukiecontrolqueue.hpp"
#include "kinematics/komoplanner.hpp"
#include "../utils/kukadutokenizer.hpp"
#include "kinematics/moveitkinematics.hpp"

#include <tf/tf.h>
#include <stdexcept>

using namespace std;
using namespace arma;

namespace kukadu {

    void KukieControlQueue::constructQueue(double sleepTime, std::string commandTopic, std::string retPosTopic, std::string switchModeTopic, std::string retCartPosTopic,
                        std::string cartStiffnessTopic, std::string jntStiffnessTopic, std::string ptpTopic,
                        std::string commandStateTopic, std::string ptpReachedTopic, std::string addLoadTopic, std::string jntFrcTrqTopic, std::string cartFrcTrqTopic,
                        std::string cartPtpTopic, std::string cartPtpReachedTopic, std::string cartMoveRfQueueTopic, std::string cartMoveWfQueueTopic, std::string cartPoseRfTopic, std::string jntSetPtpThreshTopic, bool acceptCollisions, ros::NodeHandle node
                    ) {

        set_ctrlc_exit_handler();

        this->acceptCollisions = acceptCollisions;

        this->ptpTopic = ptpTopic;
        this->addLoadTopic = addLoadTopic;
        this->commandTopic = commandTopic;
        this->cartPtpTopic = cartPtpTopic;
        this->retJointPosTopic = retPosTopic;
        this->jntFrcTrqTopic = jntFrcTrqTopic;
        this->switchModeTopic = switchModeTopic;
        this->retCartPosTopic = retCartPosTopic;
        this->ptpReachedTopic = ptpReachedTopic;
        this->cartFrcTrqTopic = cartFrcTrqTopic;
        this->stiffnessTopic = cartStiffnessTopic;
        this->jntStiffnessTopic = jntStiffnessTopic;
        this->commandStateTopic = commandStateTopic;
        this->cartPtpReachedTopic = cartPtpReachedTopic;
        this->cartMoveRfQueueTopic = cartMoveRfQueueTopic;
        this->cartMoveWfQueueTopic = cartMoveWfQueueTopic;
        this->jntSetPtpThreshTopic = jntSetPtpThreshTopic;

        loop_rate = new ros::Rate(1.0 / sleepTime);

        cartesianPtpReached = 0;

        setInitValues();
        this->node = node;

        if(sleepTime == 0.0) {
            ROS_ERROR("(KukieControlQueue) the sleep time you provided is 0. note that it is required in seconds");
            throw KukaduException("(KukieControlQueue) the sleep time you provided is 0. note that it is required in seconds");
        }

        subJntPos = node.subscribe(retPosTopic, 2, &KukieControlQueue::robotJointPosCallback, this);
        subComState = node.subscribe(commandStateTopic, 2, &KukieControlQueue::commandStateCallback, this);
        subPtpReached = node.subscribe(ptpReachedTopic, 2, &KukieControlQueue::ptpReachedCallback, this);
        subjntFrcTrq = node.subscribe(jntFrcTrqTopic, 2, &KukieControlQueue::jntFrcTrqCallback, this);
        subCartFrqTrq = node.subscribe(cartFrcTrqTopic, 2, &KukieControlQueue::cartFrcTrqCallback, this);
        subCartPtpReached = node.subscribe(cartPtpReachedTopic, 2, &KukieControlQueue::cartPtpReachedCallback, this);
        subCartPoseRf = node.subscribe(cartPoseRfTopic, 2, &KukieControlQueue::cartPosRfCallback, this);

        pub_set_cart_stiffness = node.advertise<iis_robot_dep::CartesianImpedance>(stiffnessTopic, 1);
        pub_set_joint_stiffness = node.advertise<iis_robot_dep::FriJointImpedance>(jntStiffnessTopic, 1);
        pubCartPtp = node.advertise<geometry_msgs::Pose>(cartPtpTopic, 1);
        pubCartMoveRfQueue = node.advertise<geometry_msgs::Pose>(cartMoveRfQueueTopic, 1);
        pubCartMoveWfQueue = node.advertise<geometry_msgs::Pose>(cartMoveWfQueueTopic, 1);
        pub_set_ptp_thresh = node.advertise<std_msgs::Float64>(jntSetPtpThreshTopic, 1);

        pubCommand = node.advertise<std_msgs::Float64MultiArray>(commandTopic, 10);
        pubSwitchMode = node.advertise<std_msgs::Int32>(switchModeTopic, 1);
        pubPtp = node.advertise<std_msgs::Float64MultiArray>(ptpTopic, 10);

        isRealRobot = (getRobotDeviceType().compare("real")) ? false : true;

        // this is required because shared_from_this can't be called in constructor (initializiation happens by lazy loading)
        plannerInitialized = false;
        kinematicsInitialized = false;

        ros::Rate r(10);
        while(!firstJointsReceived || !firstModeReceived)
            r.sleep();

        currentControlType = impMode;

        currentCartFrqTrq = vec(6);
        currentCartFrqTrq.fill(0.0);

        currentJntFrqTrq = vec(getMovementDegreesOfFreedom());
        currentJntFrqTrq.fill(0.0);

        usleep(1e6);

    }

    void KukieControlQueue::startQueueThreadHook() {
        cartPoseThr = kukadu_thread(&KukieControlQueue::computeCurrentCartPose, this);
    }

    KukieControlQueue::KukieControlQueue(double sleepTime, std::string deviceType, std::string armPrefix, ros::NodeHandle node, bool acceptCollisions) : ControlQueue(LBR_MNJ, sleepTime) {

        commandTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/move";
        retJointPosTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/get_state";
        switchModeTopic = "/" + deviceType + "/" + armPrefix + "/settings/switch_mode";
        retCartPosTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/get_pose_quat_wf";
        stiffnessTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/set_impedance";
        jntStiffnessTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/set_impedance";
        ptpTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/ptp";
        commandStateTopic = "/" + deviceType + "/" + armPrefix + "/settings/get_command_state";
        ptpReachedTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/ptp_reached";
        jntFrcTrqTopic = "/" + deviceType + "/" + armPrefix + "/sensoring/est_ext_jnt_trq";
        cartFrcTrqTopic = "/" + deviceType + "/" + armPrefix + "/sensoring/cartesian_wrench";
        cartPtpTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/ptpQuaternion";
        cartPtpReachedTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/ptp_reached";
        cartMoveRfQueueTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/move_rf";
        cartMoveWfQueueTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/move_wf";
        cartPoseRfTopic = "/" + deviceType + "/" + armPrefix + "/cartesian_control/get_pose_quat_rf";
        jntSetPtpThreshTopic = "/" + deviceType + "/" + armPrefix + "/joint_control/set_ptp_thresh";
        addLoadTopic = "not supported yet";

        this->deviceType = deviceType;
        this->armPrefix = armPrefix;

        constructQueue(sleepTime, commandTopic, retJointPosTopic, switchModeTopic, retCartPosTopic, stiffnessTopic,
                       jntStiffnessTopic, ptpTopic, commandStateTopic, ptpReachedTopic, addLoadTopic, jntFrcTrqTopic, cartFrcTrqTopic,
                       cartPtpTopic, cartPtpReachedTopic, cartMoveRfQueueTopic, cartMoveWfQueueTopic, cartPoseRfTopic, jntSetPtpThreshTopic, acceptCollisions, node);

    }

    void KukieControlQueue::submitNextJointMove(arma::vec joints) {

        std_msgs::Float64MultiArray nextCommand;
        for(int i = 0; i < getMovementDegreesOfFreedom(); ++i)
            nextCommand.data.push_back(joints[i]);
        pubCommand.publish(nextCommand);

    }

    void KukieControlQueue::submitNextCartMove(geometry_msgs::Pose pose) {

        pubCartMoveWfQueue.publish(pose);

    }

    std::string KukieControlQueue::getRobotSidePrefix() {

        KukaduTokenizer tok(armPrefix, "_");
        return tok.next();

    }

    void KukieControlQueue::cartPosRfCallback(const geometry_msgs::Pose msg) {
        currentCartPoseRf = msg;
    }

    geometry_msgs::Pose KukieControlQueue::getCurrentCartesianPoseRf() {
        return currentCartPoseRf;
    }

    void KukieControlQueue::setJntPtpThresh(double thresh) {
        std_msgs::Float64 th;
        th.data = thresh;
        pub_set_ptp_thresh.publish(th);
    }

    // relative pos in worldframe
    geometry_msgs::Pose KukieControlQueue::moveCartesianRelativeWf(geometry_msgs::Pose basePoseRf, geometry_msgs::Pose offset) {

        double newTargetWorldPos[4] = {1, 1, 1, 1};

        // add relative coordinates
        newTargetWorldPos[0] = basePoseRf.position.x + offset.position.x;
        newTargetWorldPos[1] = basePoseRf.position.y + offset.position.y;
        newTargetWorldPos[2] = basePoseRf.position.z + offset.position.z;

        // store it back to current pose
        basePoseRf.position.x = newTargetWorldPos[0];
        basePoseRf.position.y = newTargetWorldPos[1];
        basePoseRf.position.z = newTargetWorldPos[2];

        // publish robot frame pose to move
        addCartesianPosToQueue(basePoseRf);

        return basePoseRf;

    }

    std::string KukieControlQueue::getRobotDeviceType() {
        return deviceType;
    }

    std::string KukieControlQueue::getRobotFileName() {
        return string("kuka_lwr_") + deviceType + string("_") + armPrefix;
    }

    std::string KukieControlQueue::getRobotName() {
        return string("KUKA LWR (") + deviceType + string(" ") + armPrefix + string(")");
    }

    std::vector<std::string> KukieControlQueue::getJointNames() {
        vector<string> retVal;
        retVal.push_back("A1"); retVal.push_back("A2"); retVal.push_back("E1"); retVal.push_back("A3"); retVal.push_back("A4"); retVal.push_back("A5"); retVal.push_back("A6");
        return retVal;
    }

    void KukieControlQueue::jntMoveCallback(const std_msgs::Float64MultiArray& msg) {

    }

    void KukieControlQueue::cartFrcTrqCallback(const geometry_msgs::Wrench& msg) {

        cartFrcTrqMutex.lock();
            currentCartFrqTrq = vec(6);
            if(isRealRobot) {
                currentCartFrqTrq(0) = msg.force.x;
                currentCartFrqTrq(1) = msg.force.y;
                currentCartFrqTrq(2) = msg.force.z;
                currentCartFrqTrq(3) = msg.torque.x;
                currentCartFrqTrq(4) = msg.torque.y;
                currentCartFrqTrq(5) = msg.torque.z;
            } else {
                currentCartFrqTrq.fill(0.0);
            }
        cartFrcTrqMutex.unlock();

    }

    void KukieControlQueue::jntFrcTrqCallback(const std_msgs::Float64MultiArray& msg) {

        if(isRealRobot)
            currentJntFrqTrq = stdToArmadilloVec(msg.data);
        else {
            currentJntFrqTrq = vec(getMovementDegreesOfFreedom());
            currentJntFrqTrq.fill(0.0);
        }

    }

    void KukieControlQueue::robotJointPosCallback(const sensor_msgs::JointState& msg) {

        currentJointsMutex.lock();

            currJoints = arma::vec(msg.position.size());
            for(int i = 0; i < msg.position.size(); ++i) currJoints(i) = msg.position.at(i);

            firstJointsReceived = true;

        currentJointsMutex.unlock();

    }

    arma::vec KukieControlQueue::getFrcTrqCart(){
        return currentCartFrqTrq;
    }

    void KukieControlQueue::computeCurrentCartPose() {

        if(!kinematicsInitialized) {

            kin = KUKADU_SHARED_PTR<Kinematics>(new KomoPlanner(shared_from_this(),
                                                                     resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"),
                                                                     resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"),
                                                                     getRobotSidePrefix(), acceptCollisions));
            kinematicsInitialized = true;

        }

        while(getQueueRunning()) {

            forwadKinMutex.lock();
                currCarts = kin->computeFk(armadilloToStdVec(getCurrentJoints().joints));
            forwadKinMutex.unlock();

        }

    }

    geometry_msgs::Pose KukieControlQueue::getCurrentCartesianPose() {

        return currCarts;

    }

    void KukieControlQueue::commandStateCallback(const std_msgs::Float32MultiArray& msg) {
        impMode = msg.data[1];
        firstModeReceived = true;
    }

    void KukieControlQueue::ptpReachedCallback(const std_msgs::Int32MultiArray& msg) {
        ptpReached = msg.data[0];
    }

    void KukieControlQueue::cartPtpReachedCallback(const std_msgs::Int32MultiArray& msg) {
        cartesianPtpReached = msg.data[0];
    }

    void KukieControlQueue::setInitValues() {
        impMode = -1;
        ptpReached = false;
        firstJointsReceived = false;
        firstModeReceived = false;
    }

    int KukieControlQueue::getCurrentControlType() {
        return impMode;
    }

    mes_result KukieControlQueue::getCurrentCartesianFrcTrq() {

        mes_result ret;

        cartFrcTrqMutex.lock();
            ret.joints = currentCartFrqTrq;
        cartFrcTrqMutex.unlock();

        ret.time = getCurrentTime();

        return ret;

    }

    bool KukieControlQueue::stopQueueWhilePtp() {
        return false;
    }

    mes_result KukieControlQueue::getCurrentJntFrcTrq() {

        mes_result ret;

        ret.joints = currentJntFrqTrq;
        ret.time = getCurrentTime();

        return ret;

    }

    void KukieControlQueue::setCurrentControlTypeInternal(int controlType) {

        std_msgs::Int32 newMode;
        newMode.data = controlType;
        if(ros::ok()) {
            pubSwitchMode.publish(newMode);
            ros::spinOnce();
        }
        while(impMode != controlType) {
            loop_rate->sleep();
            ros::spinOnce();
        }

    }

    void KukieControlQueue::cartPtpInternal(geometry_msgs::Pose pos, double maxForce) {

        cartesianPtpReached = false;

        komoMutex.lock();
            if(!plannerInitialized) {

                planner = KUKADU_SHARED_PTR<PathPlanner>(new KomoPlanner(shared_from_this(),
                                                                         resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"),
                                                                         resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"),
                                                                         getRobotSidePrefix(), acceptCollisions));
                plannerInitialized = true;
            }

            vector<geometry_msgs::Pose> desiredPlan;
            desiredPlan.push_back(getCurrentCartesianPose());
            desiredPlan.push_back(pos);
            vector<vec> desiredJointPlan = planner->planCartesianTrajectory(desiredPlan, false, true);

        komoMutex.unlock();

        bool maxForceExceeded = false;
        if(desiredJointPlan.size() > 0) {

            for(int i = 0; i < desiredJointPlan.size() && !maxForceExceeded; ++i) {

                if(getAbsoluteCartForce() > maxForce)
                    maxForceExceeded = true;
                else
                    addJointsPosToQueue(desiredJointPlan.at(i));
                synchronizeToControlQueue(1);

            }

        } else {
            ROS_ERROR("(KukieControlQueue) Cartesian position not reachable");
        }

    }

    void KukieControlQueue::jointPtpInternal(arma::vec joints) {

        ptpReached = false;

        komoMutex.lock();
        if(!plannerInitialized) {

            planner = KUKADU_SHARED_PTR<PathPlanner>(new KomoPlanner(shared_from_this(),
                                                                     resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"),
                                                                     resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"),
                                                                     getRobotSidePrefix(), acceptCollisions));
            plannerInitialized = true;

        }

        bool performPtp = false;
        vec currentState = getCurrentJoints().joints;
        for(int i = 0; i < joints.n_elem; ++i)
            if(abs(currentState(i) - joints(i)) > 0.01) {
                performPtp = true;
                break;
            }


        vector<vec> desiredJointPlan;
        if(performPtp) {

            vector<arma::vec> desiredPlan;
            desiredPlan.push_back(getCurrentJoints().joints);
            desiredPlan.push_back(joints);
            desiredJointPlan = planner->planJointTrajectory(desiredPlan);

        }
        komoMutex.unlock();

        if(performPtp) {
            if(desiredJointPlan.size() > 0) {

                for(int i = 0; i < desiredJointPlan.size(); ++i)
                    addJointsPosToQueue(desiredJointPlan.at(i));

                synchronizeToControlQueue(1);

            } else {
                ROS_ERROR("(KukieControlQueue) Joint plan not reachable");
            }
        }

    }

    double KukieControlQueue::computeDistance(float* a1, float* a2, int size) {
        double ret = 0.0;
        for(int i = 0 ; i < size; ++i) {
            ret = pow(a1[i] - a2[i], 2);
        }

        return ret;
    }

    void KukieControlQueue::setAdditionalLoad(float loadMass, float loadPos) {

        if(isRealRobot) {

            std_msgs::Float32MultiArray msg;
            msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            msg.layout.dim[0].size = 2;
            msg.layout.dim[0].stride = 0;
            msg.layout.dim[0].label = "Load";
            msg.data.push_back(loadMass);
            msg.data.push_back(loadPos);

            pubAddLoad.publish(msg);

            int tmpMode = getCurrentControlType();
            stopCurrentMode();
            switchMode(tmpMode);

        } else {
            if(!isShutUp())
                ROS_INFO("(setAdditionalLoad) this functionality is not available in simulator - ignored");
        }

    }

    void KukieControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {

        if(isRealRobot) {
            iis_robot_dep::CartesianImpedance imp;

            imp.stiffness.linear.x = imp.stiffness.linear.y = imp.stiffness.linear.z = cpstiffnessxyz;
            imp.damping.linear.x = imp.damping.linear.y = imp.damping.linear.z = cpdamping;
            imp.stiffness.angular.x = imp.stiffness.angular.y = imp.stiffness.angular.z = cpstiffnessabc;
            imp.damping.angular.x = imp.damping.angular.y = imp.damping.angular.z = cpdamping;
            imp.cpmaxdelta = cpmaxdelta;
            imp.axismaxdeltatrq = axismaxdeltatrq;

            iis_robot_dep::FriJointImpedance newImpedance;
            for (int j = 0; j < 7; j++){
                newImpedance.stiffness[j] = cpstiffnessxyz;
                newImpedance.damping[j] = cpdamping;
            }

            pub_set_cart_stiffness.publish(imp);
            pub_set_joint_stiffness.publish(newImpedance);
            ros::spinOnce();
        } else {
            if(!isShutUp())
                ROS_INFO("(setStiffness) this functionality is not available in simulator - ignored");
        }

    }

    mes_result KukieControlQueue::getCurrentJoints() {
        mes_result res;
        res.time = getCurrentTime();
        res.joints = currJoints;
        return res;
    }

    void KukieControlQueue::safelyDestroy() {
    }

}
