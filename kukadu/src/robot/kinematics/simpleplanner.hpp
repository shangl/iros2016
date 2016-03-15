#ifndef KUKADU_SIMPLEPLANNER_H
#define KUKADU_SIMPLEPLANNER_H

#include <vector>
#include <ros/ros.h>
#include <armadillo>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <geometry_msgs/Pose.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include "kinematics.hpp"
#include "pathplanner.hpp"
#include "../controlqueue.hpp"
#include "../../types/kukadutypes.hpp"

namespace kukadu {

    class SimplePlanner : public PathPlanner {

    private:

        int degOfFreedom;

        double cycleTime;

        KUKADU_SHARED_PTR<Kinematics> kin;
        KUKADU_SHARED_PTR<ControlQueue> queue;

        RMLPositionFlags refFlags;
        ReflexxesAPI* refApi;
        RMLPositionInputParameters* refInputParams;
        RMLPositionOutputParameters* refOutputParams;

        bool checkRestrictions(const std::vector<arma::vec>& plan);
        bool checkPlanSmoothness(const std::vector<arma::vec>& plan);

#ifdef CPP11SUPPORTED
        static constexpr int MAX_NUM_ATTEMPTS = 10;
        static constexpr double MAX_JNT_DIST = 0.2;
#else
        static const int MAX_NUM_ATTEMPTS = 10;
        static const double MAX_JNT_DIST = 0.2;
#endif

    public:

        SimplePlanner(KUKADU_SHARED_PTR<ControlQueue> queue, KUKADU_SHARED_PTR<Kinematics> kin);
        ~SimplePlanner();

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints);
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);

    };

}

#endif
