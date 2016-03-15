#include <hangl_pushing_rect/pushingtranscontroller.hpp>

#include <sstream>
#include <armadillo>
#include <kukadu/kukadu.h>
#include <hangl_pushing_rect/utils.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

namespace pushing {

    PushingTranslationController::PushingTranslationController(arma::vec pushTo,
                                                         KUKADU_SHARED_PTR<kukadu::ControlQueue> simulationQueue, KUKADU_SHARED_PTR<kukadu::ControlQueue> executionQueue,
                                                         KUKADU_SHARED_PTR<RosSchunk> schunkSim, KUKADU_SHARED_PTR<RosSchunk> schunkReal,
                                                         KUKADU_SHARED_PTR<kukadu::KomoPlanner> komoPlanner,
                                                         bool useReal,
                                                         KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward,
                                                         ros::NodeHandle node) : Controller(createCaption(pushTo), 0) {

        this->useReal = useReal;

        this->pushTo = pushTo;

        this->schunkSim = schunkSim;
        this->schunkReal = schunkReal;

        this->simulationQueue = simulationQueue;
        this->executionQueue = executionQueue;

        this->pushForward = pushForward;

        recoServ = node.serviceClient<hangl_vision::visionservice>("recognize_book");
        while(!recoServ.exists()) {
            ROS_WARN("Waiting for planning service to come up...");
            recoServ.waitForExistence(ros::Duration(10));
        }

        pushHandPos = {0.0, -1.57, 0.0, -1.57, 0.0, -1.57, 0.0};

        this->komoPlanner = komoPlanner;

    }

    void PushingTranslationController::setPushTo(arma::vec pushTo) {
        this->pushTo = pushTo;
    }

    KUKADU_SHARED_PTR<kukadu::ControllerResult> PushingTranslationController::performAction() {

        if(pushForward)
            pushForward->performAction();

        hangl_vision::visionserviceRequest bookRequest;
        hangl_vision::visionserviceResponse bookResponse;

        moveHome(simulationQueue, executionQueue, useReal);

        // schunkSim->moveJoints(stdToArmadilloVec(pushHandPos));
        goToPushHandPos(schunkSim);

        if(useReal)
            // schunkReal->moveJoints(stdToArmadilloVec(pushHandPos));
            goToPushHandPos(schunkReal);

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

        double length = bookResponse.coordinates.at(0);
        double width = bookResponse.coordinates.at(2);
        double height = bookResponse.coordinates.at(1);

        cout << "book with dimensions (length = " << length << ", width = " << width << ", height = " << height << ") at position (" <<
                bookResponse.coordinates.at(3) << ", " << bookResponse.coordinates.at(4) << ", " << bookResponse.coordinates.at(5) << ") with rotation of " <<
                bookOrientation << " rad found" << endl;

        struct rectAlignment rectAligned = checkOrientation(bookOrientation, ROTATION_TOLERANCE);

        vec bookLocation = stdToArmadilloVec(createJointsVector(3, bookResponse.coordinates.at(3), bookResponse.coordinates.at(4), bookResponse.coordinates.at(5)));

        double distance = 0.0;
        bool finishedPushing = false;

        while(!finishedPushing) {

            double horicontalDistance = bookLocation(1) - pushTo(1);
            double verticalDistance = bookLocation(0) - pushTo(0);

            int pushDirection;
            if(abs(horicontalDistance) > Y_TRANSLATION_TOLERANCE)
                pushDirection = (horicontalDistance > 0) ? PUSH_RIGHT : PUSH_LEFT;
            else if(abs(verticalDistance) > X_TRANSLATION_TOLERANCE) {

                pushDirection = (verticalDistance > 0) ? PUSH_DOWN : PUSH_UP;

                // down ignored for now (that is done by the final push controller)
                if(pushDirection == PUSH_DOWN)
                    finishedPushing = true;

            } else
                finishedPushing = true;

            if(!finishedPushing) {

                geometry_msgs::Pose startPoseWithOffset = pushStartPose(pushDirection, bookLocation, length, width, height,
                                                              rectAligned.alignmentType, 0.09, 0.149 + 0.07, 0.07);

                geometry_msgs::Pose startPoseWoHeightOffset = pushStartPose(pushDirection, bookLocation, length, width, height,
                                                              rectAligned.alignmentType, 0.09, 0.149, 0.07);

                geometry_msgs::Pose startPose = pushStartPose(pushDirection, bookLocation, length, width, height,
                                                              rectAligned.alignmentType, 0.09, 0.149, 0.0);

                ptpWithSimulation(startPoseWithOffset, simulationQueue, executionQueue, useReal);
                ptpWithSimulation(startPoseWoHeightOffset, simulationQueue, executionQueue, useReal);

                geometry_msgs::Pose lastPose = startPose;

                // first set the appropriate distance, then push and then update location (no success checking here)
                switch(pushDirection) {
                case PUSH_RIGHT:
                    cout << "pushing right" << endl;
                    distance = abs(horicontalDistance) - 0.02;
                    lastPose = pushRight(startPose, distance);
                    bookLocation(1) = pushTo(1);
                    break;
                case PUSH_LEFT:
                    cout << "pushing left" << endl;
                    // 0.03 is the correction for the finger width
                    distance = abs(horicontalDistance);
                    lastPose = pushLeft(startPose, distance);
                    bookLocation(1) = pushTo(1);
                    break;
                case PUSH_UP:
                    cout << "pushing up" << endl;
                    distance = abs(verticalDistance);
                    lastPose = pushUp(startPose, distance);
                    bookLocation(0) = pushTo(0);
                    break;
                case PUSH_DOWN:
                    cout << "pushing down" << endl;
                    // 0.03 is the correction for the finger width
                    distance = abs(verticalDistance) - 0.03;
                    lastPose = pushDown(startPose, distance);
                    bookLocation(0) = pushTo(0);
                    break;
                }

                lastPose.position.z += 0.1;
                ptpWithSimulation(lastPose, simulationQueue, executionQueue, useReal);

            }

        }

        return KUKADU_SHARED_PTR<kukadu::ControllerResult>();

    }

    geometry_msgs::Pose PushingTranslationController::pushDown(geometry_msgs::Pose startPose, double distance) {

        double jumpingStep = 0.03;

        vector<vec> pushJointPlan;

        ptpWithSimulation(startPose, simulationQueue, executionQueue, useReal);

        vec lastJointPos = simulationQueue->getCurrentJoints().joints;

        geometry_msgs::Pose lastCart = startPose;

        for(double i = 0; i < distance; i += jumpingStep) {

            vector<geometry_msgs::Pose> pushCartIntermedPlan;
            startPose.position.x -= jumpingStep;
            pushCartIntermedPlan.push_back(startPose);
            vector<vec> pushIntermediateJointPlan = komoPlanner->planCartesianTrajectory(lastJointPos, pushCartIntermedPlan);

            lastCart = startPose;

            // drop first joint config as it is already contained in the last plan
            for(int k = 1; k < pushIntermediateJointPlan.size(); ++k) {
                pushJointPlan.push_back(pushIntermediateJointPlan.at(k));
                if(k == pushIntermediateJointPlan.size() - 1)
                    lastJointPos = pushIntermediateJointPlan.at(k);
            }

        }

        pushJointPlan = smoothJointPlan(pushJointPlan);

        for(vec jp : pushJointPlan)
            simulationQueue->addJointsPosToQueue(jp);
        simulationQueue->synchronizeToControlQueue(1);

        if(useReal) {
        //    cout << "press key to execute pushing on real robot" << endl; getchar();
            for(vec jp : pushJointPlan)
                executionQueue->addJointsPosToQueue(jp);
            executionQueue->synchronizeToControlQueue(1);
        }

        return lastCart;

    }

    geometry_msgs::Pose PushingTranslationController::pushUp(geometry_msgs::Pose startPose, double distance) {

        double jumpingStep = 0.03;

        vector<vec> pushJointPlan;

        ptpWithSimulation(startPose, simulationQueue, executionQueue, useReal);

        vec lastJointPos = simulationQueue->getCurrentJoints().joints;

        geometry_msgs::Pose lastCart = startPose;

        for(double i = 0; i < distance; i += jumpingStep) {

            vector<geometry_msgs::Pose> pushCartIntermedPlan;
            startPose.position.x += jumpingStep;
            pushCartIntermedPlan.push_back(startPose);
            vector<vec> pushIntermediateJointPlan = komoPlanner->planCartesianTrajectory(lastJointPos, pushCartIntermedPlan);

            lastCart = startPose;

            // drop first joint config as it is already contained in the last plan
            for(int k = 1; k < pushIntermediateJointPlan.size(); ++k) {
                pushJointPlan.push_back(pushIntermediateJointPlan.at(k));
                if(k == pushIntermediateJointPlan.size() - 1)
                    lastJointPos = pushIntermediateJointPlan.at(k);
            }

        }

        pushJointPlan = smoothJointPlan(pushJointPlan);

        for(vec jp : pushJointPlan)
            simulationQueue->addJointsPosToQueue(jp);
        simulationQueue->synchronizeToControlQueue(1);

        if(useReal) {
        //    cout << "press key to execute pushing on real robot" << endl; getchar();
            for(vec jp : pushJointPlan)
                executionQueue->addJointsPosToQueue(jp);
            executionQueue->synchronizeToControlQueue(1);
        }

        return lastCart;

    }

    geometry_msgs::Pose PushingTranslationController::pushLeft(geometry_msgs::Pose startPose, double distance) {

        double jumpingStep = 0.03;

        vector<vec> pushJointPlan;

        ptpWithSimulation(startPose, simulationQueue, executionQueue, useReal);

        vec lastJointPos = simulationQueue->getCurrentJoints().joints;

        geometry_msgs::Pose lastCart = startPose;

        for(double i = 0; i < distance; i += jumpingStep) {

            vector<geometry_msgs::Pose> pushCartIntermedPlan;
            startPose.position.y += jumpingStep;
            pushCartIntermedPlan.push_back(startPose);
            vector<vec> pushIntermediateJointPlan = komoPlanner->planCartesianTrajectory(lastJointPos, pushCartIntermedPlan);

            lastCart = startPose;

            // drop first joint config as it is already contained in the last plan
            for(int k = 1; k < pushIntermediateJointPlan.size(); ++k) {
                pushJointPlan.push_back(pushIntermediateJointPlan.at(k));
                if(k == pushIntermediateJointPlan.size() - 1)
                    lastJointPos = pushIntermediateJointPlan.at(k);
            }

        }

        pushJointPlan = smoothJointPlan(pushJointPlan);

        for(vec jp : pushJointPlan)
            simulationQueue->addJointsPosToQueue(jp);
        simulationQueue->synchronizeToControlQueue(1);

        if(useReal) {
            cout << "press key to execute pushing on real robot" << endl; // getchar();
            for(vec jp : pushJointPlan)
                executionQueue->addJointsPosToQueue(jp);
            executionQueue->synchronizeToControlQueue(1);
        }

        return lastCart;

    }

    geometry_msgs::Pose PushingTranslationController::pushRight(geometry_msgs::Pose startPose, double distance) {

        double jumpingStep = 0.03;

        vector<vec> pushJointPlan;

        ptpWithSimulation(startPose, simulationQueue, executionQueue, useReal);

        vec lastJointPos = simulationQueue->getCurrentJoints().joints;

        geometry_msgs::Pose lastCart = startPose;

        for(double i = 0; i < distance; i += jumpingStep) {

            vector<geometry_msgs::Pose> pushCartIntermedPlan;
            startPose.position.y -= jumpingStep;
            pushCartIntermedPlan.push_back(startPose);
            vector<vec> pushIntermediateJointPlan = komoPlanner->planCartesianTrajectory(lastJointPos, pushCartIntermedPlan);

            lastCart = startPose;

            // drop first joint config as it is already contained in the last plan
            for(int k = 1; k < pushIntermediateJointPlan.size(); ++k) {
                pushJointPlan.push_back(pushIntermediateJointPlan.at(k));
                if(k == pushIntermediateJointPlan.size() - 1)
                    lastJointPos = pushIntermediateJointPlan.at(k);
            }

        }

        pushJointPlan = smoothJointPlan(pushJointPlan);

        for(vec jp : pushJointPlan)
            simulationQueue->addJointsPosToQueue(jp);
        simulationQueue->synchronizeToControlQueue(1);

        if(useReal) {
            cout << "press key to execute pushing on real robot" << endl; // getchar();
            for(vec jp : pushJointPlan)
                executionQueue->addJointsPosToQueue(jp);
            executionQueue->synchronizeToControlQueue(1);
        }

        return lastCart;

    }

    geometry_msgs::Pose PushingTranslationController::pushStartPose(int pushDirection, arma::vec bookLocation, double bookLength, double bookWidth, double bookHeight, int bookAlignment,
                                                                    double fingerPalmOffset, double zOffset, double desiredBookDistance) {

        geometry_msgs::Pose handPose;

        vec bookOffset;

        // not very efficient written, but should work out
        if(bookAlignment == HORICONTAL_ALIGNMENT) {

            if(pushDirection == PUSH_LEFT) {

                // standard offset for pushing to the right (seen from the robot)
                bookOffset = stdToArmadilloVec(createJointsVector(3, 0.0, -bookLength / 2.0 + fingerPalmOffset - desiredBookDistance, zOffset));

                tf::Quaternion rot = rpyToQuat(-M_PI / 2.0, M_PI / 2.0, 0.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else if(pushDirection == PUSH_RIGHT) {

                // standard offset for pushing to the right (seen from the robot)
                bookOffset = stdToArmadilloVec(createJointsVector(3, 0.0, bookLength / 2.0 + fingerPalmOffset + desiredBookDistance, zOffset));

                tf::Quaternion rot = rpyToQuat(-M_PI / 2.0, M_PI / 2.0, 0.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else if(pushDirection == PUSH_DOWN) {

                // standard offset for pushing towards the robot body (down)
                bookOffset = stdToArmadilloVec(createJointsVector(3, bookWidth / 2.0 - fingerPalmOffset + desiredBookDistance, 0.0, zOffset));

                //tf::Quaternion rot = rpyToQuat(M_PI, 0.0, -M_PI / 2.0);
                tf::Quaternion rot = rpyToQuat(M_PI, M_PI, -M_PI / 2.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else if(pushDirection == PUSH_UP) {

                // standard offset for pushing away from the robot body (up)
                bookOffset = stdToArmadilloVec(createJointsVector(3, -bookWidth / 2.0 + fingerPalmOffset - desiredBookDistance, 0.0, zOffset));

                tf::Quaternion rot = rpyToQuat(M_PI, 0.0, -M_PI / 2.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else {
                throw "(PushingTranslationController) push direction not known";
            }

        } else {

            if(pushDirection == PUSH_LEFT || pushDirection == PUSH_RIGHT) {

                // standard offset for pushing to the right (seen from the robot)
                if(pushDirection == PUSH_LEFT)
                    bookOffset = stdToArmadilloVec(createJointsVector(3, 0.0, -bookWidth / 2.0 + fingerPalmOffset - desiredBookDistance, zOffset));
                else
                    bookOffset = stdToArmadilloVec(createJointsVector(3, 0.0, bookWidth / 2.0 + fingerPalmOffset + desiredBookDistance, zOffset));

                tf::Quaternion rot = rpyToQuat(-M_PI / 2.0, M_PI / 2.0, 0.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else if(pushDirection == PUSH_DOWN) {

                // standard offset for pushing towards the robot body (down)
                bookOffset = stdToArmadilloVec(createJointsVector(3, bookLength / 2.0 - fingerPalmOffset + desiredBookDistance, 0.0, zOffset));

                //tf::Quaternion rot = rpyToQuat(M_PI, 0.0, -M_PI / 2.0);
                tf::Quaternion rot = rpyToQuat(M_PI, M_PI, -M_PI / 2.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else if(pushDirection == PUSH_UP) {

                // standard offset for pushing away from the robot body (up)
                bookOffset = stdToArmadilloVec(createJointsVector(3, -bookLength / 2.0 + fingerPalmOffset - desiredBookDistance, 0.0, zOffset));

                tf::Quaternion rot = rpyToQuat(M_PI, 0.0, -M_PI / 2.0);
                handPose.orientation.x = rot.getX();
                handPose.orientation.y = rot.getY();
                handPose.orientation.z = rot.getZ();
                handPose.orientation.w = rot.getW();

            } else {
                throw "(PushingTranslationController) push direction not known";
            }


        }

        vec finalPalmPosition = bookLocation + bookOffset;
        handPose.position.x = finalPalmPosition(0); handPose.position.y = finalPalmPosition(1); handPose.position.z = finalPalmPosition(2);

        return handPose;

    }

    std::string PushingTranslationController::createCaption(arma::vec pushTo) {
        stringstream s;
        s << "translate" << pushTo(0) << "_" << pushTo(1);
        return s.str();
    }

    // returns (valid orientation, orientation id, to rotate); orientation id = 0 --> horicontal, orientation id = 1 --> vertical; to rotate: how many radians the book has to be rotated
    // to be perfectly aligned
    struct rectAlignment PushingTranslationController::checkOrientation(double rotation, double tolerance) {

        struct rectAlignment retStruct;

        retStruct.aligned = false;
        retStruct.alignmentType = INT_MAX;
        retStruct.rotationError = 0.0;

        double rotateToHoricontal = -rotation;
        double rotateToVertical1 = M_PI / 2 - rotation;
        double rotateToVertical2 = -M_PI / 2 + rotation;

        if(abs(rotateToHoricontal) <= abs(rotateToVertical1) && abs(rotateToHoricontal) <= abs(rotateToVertical2)) {
            retStruct.alignmentType = HORICONTAL_ALIGNMENT;
            retStruct.rotationError = rotateToHoricontal;
            if(abs(rotateToHoricontal) < tolerance)
                retStruct.aligned = true;
        } else if(abs(rotateToVertical1) <= abs(rotateToHoricontal) && abs(rotateToVertical1) <= abs(rotateToVertical2)) {
            retStruct.alignmentType = VERTICAL_ALIGNMENT;
            retStruct.rotationError = rotateToVertical1;
            if(abs(rotateToVertical1) < tolerance)
                retStruct.aligned = true;
        } else if(abs(rotateToVertical2) <= abs(rotateToHoricontal) && abs(rotateToVertical2) <= abs(rotateToVertical1)) {
            retStruct.alignmentType = VERTICAL_ALIGNMENT;
            retStruct.rotationError = rotateToVertical2;
            if(abs(rotateToVertical2) < tolerance)
                retStruct.aligned = true;
        } else {
            throw "(PushingTranslationController) cannot estimate orientation";
        }

        return retStruct;

    }

}
