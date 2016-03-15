#include <hangl_pushing_rect/pushingrotcontroller.hpp>

#include <sstream>

using namespace std;
using namespace arma;
using namespace kukadu;

namespace pushing {

    PushingRotationController::PushingRotationController(double rotation, KUKADU_SHARED_PTR<kukadu::ControlQueue> simulationQueue, KUKADU_SHARED_PTR<kukadu::ControlQueue> executionQueue,
                                                         KUKADU_SHARED_PTR<RosSchunk> schunkSim, KUKADU_SHARED_PTR<RosSchunk> schunkReal,
                                                         KUKADU_SHARED_PTR<kukadu::KomoPlanner> komoPlanner,
                                                         bool useReal,
                                                         KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward,
                                                         KUKADU_SHARED_PTR<pushing::PushingTranslationController> originTrans,
                                                         KUKADU_SHARED_PTR<pushing::FinalPushController> finPush,
                                                         ros::NodeHandle node) : Controller(createCaption(rotation), 0) {

        this->useReal = useReal;

        this->rotation = rotation;

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

        pushHandPos = {0.0, 1.1, 0.0, -1.57, 0.0, -1.57, 0.0};

        palmOffset = 0.09;
        pushHeightOffset = 0.155;

        this->komoPlanner = komoPlanner;
        this->originTrans = originTrans;
        this->finPush = finPush;

    }

    double PushingRotationController::getPalmOffset() {
        return palmOffset;
    }

    double PushingRotationController::getPushHeightOffset() {
        return pushHeightOffset;
    }

    KUKADU_SHARED_PTR<kukadu::ControllerResult> PushingRotationController::performAction() {

        if(pushForward) {
            pushForward->performAction();
            pushForward->getHand()->moveJoints(stdToArmadilloVec({0.0, 0.14, 0.8, 1.06, 1.54, 0.18, 0.8}));
            simpleJointPtp(pushForward->getQueue(), stdToArmadilloVec({-2.1735239028930664, 0.7832490801811218, -1.2088751792907715, 1.7411022186279297, 1.7530142068862915, -1.5967271327972412, -1.0228410959243774}));
        }

        double fingerToHandOffset = 0.01;
        double handOffset = 0.1;
        double zSafetyOffset = 0.1;
        double touchingPoint = 0.7;

        hangl_vision::visionserviceRequest bookRequest;
        hangl_vision::visionserviceResponse bookResponse;

        schunkSim->moveJoints(stdToArmadilloVec(pushHandPos));

        if(useReal)
            schunkReal->moveJoints(stdToArmadilloVec(pushHandPos));

        moveHome(simulationQueue, executionQueue, useReal);

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

        geometry_msgs::Pose p = computePushingPos(bookOrientation, length, width, height, bookResponse.coordinates.at(3), bookResponse.coordinates.at(4), bookResponse.coordinates.at(5),
                                                  palmOffset, handOffset, fingerToHandOffset, zSafetyOffset, touchingPoint);

        cout << "pos x: " << p.position.x << "; pos y: " << p.position.y << "; pos z: " << p.position.z << endl;


        cout << "press key to move to book position" << endl;
        //getchar();

        // move to book with security margin on top of the book
        ptpWithSimulation(p, simulationQueue, executionQueue, useReal);

        // move the finger down to the table
        p = computePushingPos(bookOrientation, length, width, height, bookResponse.coordinates.at(3), bookResponse.coordinates.at(4), bookResponse.coordinates.at(5),
                                                  palmOffset, handOffset, fingerToHandOffset, 0.0, touchingPoint);

        ptpWithSimulation(p, simulationQueue, executionQueue, useReal);

        cout << "press key to simulate pushing" << endl;
        vector<vec> pushJointPlan;
        pushJointPlan.push_back(simulationQueue->getCurrentJoints().joints);

        vec lastJointPos = simulationQueue->getCurrentJoints().joints;

        for(double i = 0; i < rotation; i += 0.1) {

            p = computePushingPos(bookOrientation - i, length, width, height, bookResponse.coordinates.at(3), bookResponse.coordinates.at(4), bookResponse.coordinates.at(5),
                                                      palmOffset, 0.0, fingerToHandOffset, 0.0, touchingPoint);

            vector<geometry_msgs::Pose> pushCartIntermedPlan;
            pushCartIntermedPlan.push_back(p);
            vector<vec> pushIntermediateJointPlan = komoPlanner->planCartesianTrajectory(lastJointPos, pushCartIntermedPlan);

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
            cout << "press key to execute pushing on real robot" << endl;// getchar();
            for(vec jp : pushJointPlan)
                executionQueue->addJointsPosToQueue(jp);
            executionQueue->synchronizeToControlQueue(1);
        }

        if(originTrans) {
            cout << "(PushingRotController) translate back to origin" << endl;
            originTrans->performAction();
        } else {
            cout << "(PushingRotController) no translation controller given" << endl;
        }

        if(pushForward) {
            moveHome(simulationQueue, executionQueue, useReal);
            goToBlockingPos(pushForward->getQueue(), false);
            goToRightHandBlocking(pushForward->getHand());
            finPush->performAction();
        }

        return KUKADU_SHARED_PTR<kukadu::ControllerResult>();

    }

    std::string PushingRotationController::createCaption(double rotation) {
        stringstream s;
        s << "rotation" << rotation;
        return s.str();
    }

    geometry_msgs::Pose PushingRotationController::computePushingPos(double bookOrientation, double length, double width, double height, double xPos, double yPos, double zPos,
                                          double palmOffset, double handOffset, double fingerToHandOffset, double zOffset, double touchingPoint) {

        geometry_msgs::Pose p;

        tf::Quaternion rot = rpyToQuat(-M_PI / 2.0, M_PI / 2.0, 0.0);
        p.orientation.x = rot.getX();
        p.orientation.y = rot.getY();
        p.orientation.z = rot.getZ();
        p.orientation.w = rot.getW();

        // normal rotation matrix
        mat bookRot(2, 2); bookRot(0, 0) = cos(bookOrientation); bookRot(0, 1) = -sin(bookOrientation); bookRot(1, 0) = sin(bookOrientation); bookRot(1, 1) = cos(bookOrientation);
        vec pPos(2); pPos(0) = -width / 2.0 - handOffset; pPos(1) = touchingPoint * length / 2.0;
        vec bookPos(2); bookPos(0) = xPos; bookPos(1) = yPos;
        vec pushPos = bookPos + bookRot * pPos;

        // correct for the error that palm is some distance behind the fingers (it is assumed that palm always is counter-aligned with the y axis)
        p.position.x = pushPos(0) + fingerToHandOffset; p.position.y = pushPos(1) + palmOffset;
        p.position.z = zPos + pushHeightOffset + zOffset;

        return p;

    }

}
