#include "rosschunk.hpp"

using namespace std;
using namespace arma;
using namespace iis_robot_dep;

namespace kukadu {

    RosSchunk::RosSchunk(std::string type, std::string hand) {

    }

    RosSchunk::RosSchunk(ros::NodeHandle node, std::string type, std::string hand) {

        this->node = node;
        waitForReached = true;
        trajPub = node.advertise<std_msgs::Float64MultiArray>(string("/") + type + string("/") + hand + "_sdh/joint_control/move", 1);

        this->hand = hand;

        stateSub = node.subscribe(string("/") + type + string("/") + hand + "_sdh/joint_control/get_state", 1, &RosSchunk::stateCallback, this);
        tactileSub = node.subscribe(string("/") + type + string("/") + hand + "_sdh/sensoring/tactile", 1, &RosSchunk::tactileCallback, this);
        previousCurrentPosQueueSize = 10;
        isFirstCallback = true;

        ros::Rate r(5);
        while(isFirstCallback) {
            ros::spinOnce();
            r.sleep();
        }

        currentCommandedPos = currentPos;
        currentGraspId = eGID_PARALLEL;
        moveJoints(stdToArmadilloVec(currentPos));

    }

    std::string RosSchunk::getHandName() {
        return string("schunk_") + hand;
    }

    void RosSchunk::setWaitForReached(bool waitForReached) {
        this->waitForReached = waitForReached;
    }

    void RosSchunk::tactileCallback(const iis_robot_dep::TactileSensor& state) {

        tactileMutex.lock();

            currentTactileReadings.clear();

            for(int i = 0; i < state.tactile_matrix.size(); ++i) {

                 TactileMatrix tactMat = state.tactile_matrix.at(i);
                 int xSize = tactMat.cells_x;
                 int ySize = tactMat.cells_y;
                 mat currentMat(xSize, ySize);

                 for(int j = 0, run = 0; j < xSize; ++j)
                     for(int k = 0; k < ySize; ++k, ++run)
                         currentMat(j, k) = tactMat.tactile_array.at(run);

                 currentTactileReadings.push_back(currentMat.t());

            }

        tactileMutex.unlock();

    }

    void RosSchunk::stateCallback(const sensor_msgs::JointState& state) {

        currentPosMutex.lock();

            if(!isFirstCallback) {

                previousCurrentPosQueue.erase(previousCurrentPosQueue.begin(), previousCurrentPosQueue.begin() + 1);
                previousCurrentPosQueue.push_back(currentPos);
                currentPos = state.position;

            } else {

                previousCurrentPosQueue.clear();
                for(int i = 0; i < previousCurrentPosQueueSize; ++i)
                    previousCurrentPosQueue.push_back(state.position);

                currentPos = state.position;
                isFirstCallback = false;
                currentCommandedPos = state.position;

            }

            vector<double> doubleCommandedPos;
            for(int i = 0; i < currentCommandedPos.size(); ++i)
                doubleCommandedPos.push_back(currentCommandedPos.at(i));

            targetReached = !vectorsDeviate(state.position, doubleCommandedPos, 0.03);

            if(!targetReached) {

                bool stillMoving = false;
                for(int i = 0; i < previousCurrentPosQueue.size(); ++i) {
                    vector<double> previousCurrentPos = previousCurrentPosQueue.at(i);
                    bool deviates = vectorsDeviate(previousCurrentPos, currentPos, 0.01);
                    if(deviates) {
                        stillMoving = true;
                        break;
                    }
                }

                if(!movementStarted && stillMoving) {
                    movementStarted = true;
                } else if(movementStarted && !stillMoving) {
                    targetReached = true;
                }

            }

        currentPosMutex.unlock();

    }

    bool RosSchunk::vectorsDeviate(const std::vector<double> v1, const std::vector<double> v2, double tolerance) {

        bool tmpReached = true;

        for(int i = 0; i < v1.size(); ++i) {
            if(abs(v1.at(i) - v2.at(i)) > tolerance)
                tmpReached = false;
        }

        return !tmpReached;

    }

    std::vector<double> RosSchunk::generateCylindricalPose(double percentage) {

        vector<double> hand_pose;

        hand_pose.push_back(0);
        hand_pose.push_back((-30 + percentage * 30) * M_PI / 180.0);
        hand_pose.push_back((30 + percentage * 35) * M_PI / 180.0);
        hand_pose.push_back((-30 + percentage * 30) * M_PI / 180.0);
        hand_pose.push_back((30 + percentage * 35) * M_PI / 180.0);
        hand_pose.push_back((-30 + percentage * 30) * M_PI / 180.0);
        hand_pose.push_back((30 + percentage * 35) * M_PI / 180.0);

        return hand_pose;

    }

    std::vector<double> RosSchunk::generateParallelPose(double percentage) {

        vector<double> hand_pose;

        // finger orientation
        hand_pose.push_back(0);

        // finger 1
        hand_pose.push_back((-75. + percentage * 82.) * M_PI / 180.0);


        hand_pose.push_back((75. - percentage * 82.) * M_PI / 180.0);

        // finger 2, joint 1
        hand_pose.push_back((-75. + percentage * 82.) * M_PI / 180.0);

        // finger 2, joint 2
        hand_pose.push_back((90. - percentage * 82.) * M_PI / 180.0);

        hand_pose.push_back((-75. + percentage * 82.) * M_PI / 180.0);
        hand_pose.push_back((90. - percentage * 82.) * M_PI / 180.0);

        return hand_pose;

    }

    std::vector<double> RosSchunk::generateCentricalPose(double percentage) {

        vector<double> hand_pose;

        hand_pose.push_back((60) * M_PI/180.0);
        hand_pose.push_back((-75 + percentage * 82) * M_PI / 180.0);
        hand_pose.push_back((75 - percentage * 82) * M_PI / 180.0);
        hand_pose.push_back((-75 + percentage * 82) * M_PI / 180.0);
        hand_pose.push_back((75 - percentage * 82) * M_PI / 180.0);
        hand_pose.push_back((-75 + percentage * 82) * M_PI / 180.0);
        hand_pose.push_back((75 - percentage * 82) * M_PI / 180.0);

        return hand_pose;

    }

    std::vector<double> RosSchunk::generateSphericalPose(double percentage) {

        vector<double> hand_pose;

        hand_pose.push_back((60) * M_PI / 180.0);
        hand_pose.push_back((-40 + percentage * 25) * M_PI / 180.0);
        hand_pose.push_back((40 + percentage * 15) * M_PI / 180.0);
        hand_pose.push_back((-40 + percentage * 25) * M_PI / 180.0);
        hand_pose.push_back((40 + percentage * 15) * M_PI / 180.0);
        hand_pose.push_back((-40 + percentage * 25) * M_PI / 180.0);
        hand_pose.push_back((40 + percentage * 15) * M_PI / 180.0);

        return hand_pose;

    }

    void RosSchunk::connectHand() {
        // nothing to do when using ros
    }

    void RosSchunk::closeHand(double percentage, double velocity) {

        if(percentage >= 0.0 && percentage <= 1.1) {

            vector<double> hand_pose;

            switch(currentGraspId) {
            case eGID_CENTRICAL:
                hand_pose = generateCentricalPose(percentage);
                break;
            case eGID_CYLINDRICAL:
                hand_pose = generateCylindricalPose(percentage);
                break;
            case eGID_PARALLEL:
                hand_pose = generateParallelPose(percentage);
                break;
            case eGID_SPHERICAL:
                hand_pose = generateSphericalPose(percentage);
                break;
            default:
                string msg = "(RosSchunk) grasp type not supported";
                cerr << msg << endl;
                throw KukaduException(msg.c_str());

            }

            moveJoints(stdToArmadilloVec(hand_pose));

        } else {
            string msg = "(RosSchunk) grasp percentage out of range";
            cerr << msg << endl;
            throw KukaduException(msg.c_str());
        }

    }

    void RosSchunk::disconnectHand() {

        // nothing to do when using ros

    }

    void RosSchunk::setGrasp(kukadu_grasps grasp) {

        currentGraspId = grasp;

    }

    void RosSchunk::safelyDestroy() {

    }

    void RosSchunk::publishSingleJoint(int idx, double pos) {

        vector<double> command;
        for(int i = 0; i < currentCommandedPos.size(); ++i)
            if(i == idx)
                command.push_back(pos);
            else
                command.push_back(SDH_IGNORE_JOINT);

        moveJoints(stdToArmadilloVec(command));

    }

    void RosSchunk::moveJoints(arma::vec positions) {

        std::vector<double> stdPos = armadilloToStdVec(positions);
        std_msgs::Float64MultiArray newJoints;
        for(int i = 0; i < stdPos.size(); ++i) {
            if(stdPos.at(i) != SDH_IGNORE_JOINT)
                newJoints.data.push_back(stdPos.at(i));
            else
                newJoints.data.push_back(currentCommandedPos.at(i));
        }

        currentCommandedPos = newJoints.data;

        // get current position
        ros::spinOnce();

        targetReached = false;
        movementStarted = false;

        initCurrentPos = currentPos;
        trajPub.publish(newJoints);

        if(waitForReached) {

            while(!targetReached)
                ros::spinOnce();

        }

    }

    std::vector<arma::mat> RosSchunk::getTactileSensing() {

        std::vector<arma::mat> ret;
        tactileMutex.lock();
            ret = currentTactileReadings;
        tactileMutex.unlock();

        return ret;

    }

}
