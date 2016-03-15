#ifndef KUKADU_PLOTTINGHAND_H
#define KUKADU_PLOTTINGHAND_H

#include <vector>
#include <string>
#include <limits>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iis_robot_dep/TactileMatrix.h>
#include <iis_robot_dep/TactileSensor.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

#include "rosschunk.hpp"
#include "../../utils/utils.hpp"
#include "../../types/kukadutypes.hpp"

namespace kukadu {

    /** \brief Provides control capabilities for the Schunk SDH robotic hand with ROS binding
     * Implements the GenericHand interface for the Schunk SDH robotic hand. Note that using this class the programm has to be executed with root rights
     * \ingroup RobotFramework
     */
    class PlottingHand : public RosSchunk {

    private:

    public:

        PlottingHand(std::string type, std::string hand);

        void connectHand();
        void safelyDestroy();
        void disconnectHand();
        void setGrasp(kukadu_grasps grasp);
        void publishSingleJoint(int idx, double pos);
        void closeHand(double percentage, double velocity);
        void publishSdhJoints(std::vector<double> positions);
        void moveJoints(arma::vec joints);

        std::vector<arma::mat> getTactileSensing();

    };

}

#endif
