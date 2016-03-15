#ifndef KUKADU_PLOTTINGCONTROLQUEUE_H
#define KUKADU_PLOTTINGCONTROLQUEUE_H

#include <queue>
#include <time.h>
#include <math.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <iis_robot_dep/KukieError.h>
#include <iis_robot_dep/FriRobotData.h>
#include <iis_robot_dep/FriJointState.h>
#include <iis_robot_dep/FriJointCommand.h>
#include <iis_robot_dep/FriRobotJntData.h>
#include <iis_robot_dep/FriJointImpedance.h>
#include <iis_robot_dep/CartesianImpedance.h>

// Custom librairies
#include "../utils/types.hpp"
#include "../utils/utils.hpp"
#include "../types/kukadutypes.hpp"
#include "../robot/controlqueue.hpp"
#include "../utils/destroyableobject.hpp"
#include "robotDriver/src/kuka/friRemote.h"

namespace kukadu {

    /** \brief The OrocosControlQueue provides control capabilities for the Kuka LWR 4+ robotic arm
     *
     * This class implements the abstract ControlQueue class for the usage with the iisorocos system. It provides basic functionalities such as command mode control
     * in joint space as well as point to point movement in cartesian and joint space. To use it, the additionally provided KRL script has to be selected on the robot
     * controller side. For further information how to use it, please see the sample programs and the kuka documentation
     * \ingroup RobotFramework
     */
    class PlottingControlQueue : public ControlQueue {

    private:

        int impMode;
        int ptpReached;
        int monComMode;
        int currentMode;

        double currTime;

        arma::vec currJoints;
        arma::vec startJoints;

        geometry_msgs::Pose fakeCurrentPose;

        std::vector<std::string> jointNames;

        void construct(std::vector<std::string> jointNames, double timeStep);

    protected:

        virtual void startQueueThreadHook();
        virtual void submitNextJointMove(arma::vec joints);
        virtual void submitNextCartMove(geometry_msgs::Pose pose);
        virtual void setCurrentControlTypeInternal(int controlType);

        virtual bool stopQueueWhilePtp();

    public:

        /** \brief Constructor for KukaControlQueue
         * \param port port to listen for incoming fri connection
         * \param sleepTime cycle sleep time (time between to packets sent in command mode)
         * \param initMode control mode (e.g. command mode, monitor mode) with which the queue should be started
         */
        PlottingControlQueue(int degOfFreedom, double timeStep);

        PlottingControlQueue(std::vector<std::string> jointNames, double timeStep);

        void safelyDestroy();
        void setInitValues();
        void stopCurrentMode();
        void switchMode(int mode);
        void jointPtpInternal(arma::vec joints);
        void setJntPtpThresh(double thresh);
        void setStartingJoints(arma::vec joints);
        void addJointsPosToQueue(arma::vec joints);
        void cartPtpInternal(geometry_msgs::Pose pos, double maxForce);
        void addCartesianPosToQueue(geometry_msgs::Pose pose);
        void setAdditionalLoad(float loadMass, float loadPos);
        void synchronizeToControlQueue(int maxNumJointsInQueue);
        void setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq);

        virtual int getCurrentControlType();

        double getCurrentTime();

        std::string getRobotName();
        std::string getRobotFileName();

        arma::vec getStartingJoints();
        arma::vec retrieveJointsFromRobot();

        std::vector<std::string> getJointNames();

        mes_result getCurrentJoints();
        mes_result getCurrentJntFrcTrq();
        mes_result getCurrentCartesianPos();
        mes_result getCurrentCartesianFrcTrq();

        geometry_msgs::Pose getCurrentCartesianPose();

        virtual void rollBack(double time);
        virtual void stopJointRollBackMode();
        virtual void startJointRollBackMode(double possibleTime);

    };

}

#endif
