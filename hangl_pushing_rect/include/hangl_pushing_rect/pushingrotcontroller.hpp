#ifndef HANGL_PUSHING_ROT_CONTROLLER
#define HANGL_PUSHING_ROT_CONTROLLER

#include <queue>
#include <string>
#include <ros/ros.h>
#include <armadillo>
#include <ros/ros.h>
#include <kukadu/kukadu.h>
#include <geometry_msgs/Pose.h>
#include <hangl_vision/visionservice.h>

#include <hangl_pushing_rect/utils.hpp>
#include <hangl_pushing_rect/FinalPush.hpp>
#include <hangl_pushing_rect/pushforwardcontroller.hpp>
#include <hangl_pushing_rect/pushingtranscontroller.hpp>

namespace pushing {

    class PushingRotationController : public kukadu::Controller {

    private:

        bool useReal;

        double rotation;
        double palmOffset;
        double pushHeightOffset;

        std::vector<double> pushHandPos;

        ros::ServiceClient recoServ;

        KUKADU_SHARED_PTR<kukadu::RosSchunk> schunkSim;
        KUKADU_SHARED_PTR<kukadu::RosSchunk> schunkReal;

        KUKADU_SHARED_PTR<kukadu::ControlQueue> simulationQueue;
        KUKADU_SHARED_PTR<kukadu::ControlQueue> executionQueue;

        KUKADU_SHARED_PTR<kukadu::KomoPlanner> komoPlanner;

        KUKADU_SHARED_PTR<pushing::FinalPushController> finPush;
        KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward;
        KUKADU_SHARED_PTR<pushing::PushingTranslationController> originTrans;

        static std::string createCaption(double rotation);

        geometry_msgs::Pose computePushingPos(double bookOrientation, double length, double width, double height, double xPos, double yPos, double zPos,
                                              double palmOffset, double handOffset, double fingerToHandOffset, double zOffset, double touchingPoint);
        void planAndExecuteRotation();

    public:

        PushingRotationController(double rotation,
                                  KUKADU_SHARED_PTR<kukadu::ControlQueue> simulationQueue, KUKADU_SHARED_PTR<kukadu::ControlQueue> executionQueue,
                                  KUKADU_SHARED_PTR<kukadu::RosSchunk> schunkSim, KUKADU_SHARED_PTR<kukadu::RosSchunk> schunkReal,
                                  KUKADU_SHARED_PTR<kukadu::KomoPlanner> komoPlanner,
                                  bool useReal,
                                  KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward,
                                  KUKADU_SHARED_PTR<pushing::PushingTranslationController> originTrans,
                                  KUKADU_SHARED_PTR<pushing::FinalPushController> finPush,
                                  ros::NodeHandle node);

        KUKADU_SHARED_PTR<kukadu::ControllerResult> performAction();

        double getPalmOffset();
        double getPushHeightOffset();

    };

}

#endif
