#ifndef KUKADU_ROSSCHUNK_H
#define KUKADU_ROSSCHUNK_H

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

#include "generichand.hpp"
#include "../../utils/utils.hpp"
#include "../../types/kukadutypes.hpp"

namespace kukadu {

    #define SDH_IGNORE_JOINT std::numeric_limits<double>::min()

    enum kukadu_grasps {eGID_CENTRICAL, eGID_CYLINDRICAL, eGID_PARALLEL, eGID_SPHERICAL};

    /** \brief Provides control capabilities for the Schunk SDH robotic hand with ROS binding
     * Implements the GenericHand interface for the Schunk SDH robotic hand. Note that using this class the programm has to be executed with root rights
     * \ingroup RobotFramework
     */
    class RosSchunk : public GenericHand {

    private:

        kukadu_grasps currentGraspId;

        bool waitForReached;
        int previousCurrentPosQueueSize;

        ros::NodeHandle node;
        ros::Publisher trajPub;
        ros::Subscriber stateSub;
        ros::Subscriber tactileSub;

        std::vector<std::string> joint_names_str;
        std::vector<arma::mat> currentTactileReadings;

        std::string hand;

        std::vector<double> generateCylindricalPose(double percentage);
        std::vector<double> generateParallelPose(double percentage);
        std::vector<double> generateCentricalPose(double percentage);
        std::vector<double> generateSphericalPose(double percentage);

        bool targetReached;
        bool isFirstCallback;
        bool movementStarted;

        bool vectorsDeviate(const std::vector<double> v1, const std::vector<double> v2, double tolerance);
        std::vector<double> currentCommandedPos;
        std::vector<double> initCurrentPos;
        std::vector<double> currentPos;
        std::vector<std::vector<double> > previousCurrentPosQueue;

        kukadu_mutex currentPosMutex;
        kukadu_mutex tactileMutex;

        void stateCallback(const sensor_msgs::JointState& state);
        void tactileCallback(const iis_robot_dep::TactileSensor& state);

    protected:

        RosSchunk(std::string type, std::string hand);

    public:

        RosSchunk(ros::NodeHandle node, std::string type, std::string hand);

        virtual void connectHand();
        virtual void safelyDestroy();
        virtual void disconnectHand();
        virtual void setGrasp(kukadu_grasps grasp);
        virtual void publishSingleJoint(int idx, double pos);
        virtual void closeHand(double percentage, double velocity);

        void moveJoints(arma::vec joints);
        void setWaitForReached(bool waitForReached);

        virtual std::string getHandName();

        virtual std::vector<arma::mat> getTactileSensing();

    };

}

#endif
