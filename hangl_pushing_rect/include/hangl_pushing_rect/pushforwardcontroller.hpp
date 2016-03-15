#ifndef HANGL_PUSHING_FORWARD_CONTROLLER
#define HANGL_PUSHING_FORWARD_CONTROLLER

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

namespace pushing {

    class PushForwardController : public kukadu::Controller {

    private:

        bool goBackToBlockingPos;

        double ac;
        double maxForce;
        double tolAbsErr;
        double tolRelErr;
        double dmpStepSize;

        arma::vec timesFinalPush;

        std::shared_ptr<kukadu::Dmp> dmpFinalPush;
        std::shared_ptr<kukadu::SensorData> dataFinalPush;
        std::shared_ptr<kukadu::JointDMPLearner> learnerFinalPush;

        std::shared_ptr<kukadu::GenericHand> rightHand;
        std::shared_ptr<kukadu::ControlQueue> rightQueue;

    public:

        PushForwardController(std::shared_ptr<kukadu::ControlQueue> rightQueue, std::shared_ptr<kukadu::GenericHand> rightHand, std::string trajectoryPath,
                              double az, double bz, double ac, double tolAbsErr, double tolRelErr, bool goBackToBlockingPos);

        std::shared_ptr<kukadu::ControllerResult> performAction();
        std::shared_ptr<kukadu::ControllerResult> performAction(double tmax);

        KUKADU_SHARED_PTR<kukadu::GenericHand> getHand();
        KUKADU_SHARED_PTR<kukadu::ControlQueue> getQueue();

        void setMaxForce(double maxForce);
        void setGoBackToBlockingPos(bool goToBlockingPos);

    };

}

#endif
