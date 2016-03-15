#ifndef HANGL_PUSHING_TRANS_CONTROLLER
#define HANGL_PUSHING_TRANS_CONTROLLER

#include <tuple>
#include <utility>
#include <queue>
#include <string>
#include <ros/ros.h>
#include <armadillo>
#include <ros/ros.h>
#include <kukadu/kukadu.h>
#include <geometry_msgs/Pose.h>
#include <hangl_vision/visionservice.h>

#include <hangl_pushing_rect/utils.hpp>
#include <hangl_pushing_rect/pushforwardcontroller.hpp>

namespace pushing {

    struct rectAlignment {
        bool aligned;
        int alignmentType;
        double rotationError;
    };

    class PushingTranslationController : public kukadu::Controller {

    private:

        bool useReal;

        arma::vec pushTo;

        std::vector<double> pushHandPos;

        ros::ServiceClient recoServ;

        KUKADU_SHARED_PTR<kukadu::RosSchunk> schunkSim;
        KUKADU_SHARED_PTR<kukadu::RosSchunk> schunkReal;

        KUKADU_SHARED_PTR<kukadu::ControlQueue> simulationQueue;
        KUKADU_SHARED_PTR<kukadu::ControlQueue> executionQueue;

        KUKADU_SHARED_PTR<kukadu::KomoPlanner> komoPlanner;

        KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward;

        static std::string createCaption(arma::vec pushTo);

        rectAlignment checkOrientation(double rotation, double tolerance);

        geometry_msgs::Pose pushStartPose(int pushDirection, arma::vec bookLocation, double bookLength, double bookWidth, double bookHeight, int bookAlignment,
                                          double fingerPalmOffset, double zOffset, double desiredBookDistance);

        geometry_msgs::Pose pushDown(geometry_msgs::Pose startPose, double distance);
        geometry_msgs::Pose pushUp(geometry_msgs::Pose startPose, double distance);
        geometry_msgs::Pose pushLeft(geometry_msgs::Pose startPose, double distance);
        geometry_msgs::Pose pushRight(geometry_msgs::Pose startPose, double distance);

    public:

        PushingTranslationController(arma::vec pushTo,
                                  KUKADU_SHARED_PTR<kukadu::ControlQueue> simulationQueue, KUKADU_SHARED_PTR<kukadu::ControlQueue> executionQueue,
                                  KUKADU_SHARED_PTR<kukadu::RosSchunk> schunkSim, KUKADU_SHARED_PTR<kukadu::RosSchunk> schunkReal,
                                  KUKADU_SHARED_PTR<kukadu::KomoPlanner> komoPlanner,
                                  bool useReal,
                                  KUKADU_SHARED_PTR<pushing::PushForwardController> pushForward,
                                  ros::NodeHandle node);

        KUKADU_SHARED_PTR<kukadu::ControllerResult> performAction();

        void setPushTo(arma::vec pushTo);

        static constexpr int HORICONTAL_ALIGNMENT = 0;
        static constexpr int VERTICAL_ALIGNMENT = 1;

        static constexpr double ROTATION_TOLERANCE = 0.3;
        static constexpr double Y_TRANSLATION_TOLERANCE = 0.02;
        static constexpr double X_TRANSLATION_TOLERANCE = 0.05;

        static constexpr int PUSH_UP = 0;
        static constexpr int PUSH_DOWN = 1;
        static constexpr int PUSH_LEFT = 2;
        static constexpr int PUSH_RIGHT = 3;

    };

}

#endif
