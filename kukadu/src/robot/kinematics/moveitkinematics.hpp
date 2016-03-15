#ifndef KUKADU_MOVEITKINEMATICS_H
#define KUKADU_MOVEITKINEMATICS_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <armadillo>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "kinematics.hpp"
#include "restriction/restriction.hpp"
#include "../../types/kukadutypes.hpp"

namespace kukadu {

    class MoveItKinematics : public Kinematics {

    private:

        bool avoidCollisions;

        int maxAttempts;
        int degOfFreedom;

        double timeOut;

        std::string tipLink;
        std::string moveGroupName;

        robot_model::RobotModelPtr robot_model_;
        robot_model_loader::RobotModelLoaderPtr rml_;
        planning_scene::PlanningScenePtr planning_scene_;

        KUKADU_SHARED_PTR<Restriction> modelRestriction;
        KUKADU_SHARED_PTR<robot_model::JointModelGroup> jnt_model_group;

        void construct(std::string moveGroupName, std::string tipLink, bool avoidCollisions, int maxAttempts, double timeOut);

        bool collisionCheckCallback(moveit::core::RobotState* state, const moveit::core::JointModelGroup* joint_group, const double* solution);

    public:

        MoveItKinematics(std::string moveGroupName, std::string tipLink);
        MoveItKinematics(std::string moveGroupName, std::string tipLink, bool avoidCollisions, int maxAttempts, double timeOut);

        virtual std::vector<arma::vec> computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal);

        virtual geometry_msgs::Pose computeFk(std::vector<double> jointState);

        bool isColliding(arma::vec jointState, geometry_msgs::Pose pose);

        KUKADU_SHARED_PTR<Restriction> getModelRestriction();

#ifdef CPP11SUPPORTED
        static constexpr int STD_MAX_ATTEMPTS = 5;
        static constexpr double STD_TIMEOUT = 0.1;
        static constexpr bool STD_AVOID_COLLISIONS = true;
#else
        static const int STD_MAX_ATTEMPTS = 5;
        static const double STD_TIMEOUT = 0.1;
        static const bool STD_AVOID_COLLISIONS = true;
#endif

    };

}

#endif
