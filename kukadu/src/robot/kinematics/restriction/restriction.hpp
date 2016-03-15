#ifndef KUKADU_RESTRICTION_H
#define KUKADU_RESTRICTION_H

#include <armadillo>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "../../../types/kukadutypes.hpp"

namespace kukadu {

    class Restriction {

    private:

    public:

        virtual std::string getRestrictionName() = 0;

        virtual bool stateOk(arma::vec joint, geometry_msgs::Pose cartPose) = 0;

    };

    class MoveItRestriction : public Restriction {

    private:

        robot_model::RobotModelPtr robotModel;
        planning_scene::PlanningScenePtr planningScene;
        KUKADU_SHARED_PTR<moveit::core::JointModelGroup> modelGroup;

    public:

        MoveItRestriction(robot_model::RobotModelPtr, planning_scene::PlanningScenePtr planningScene, KUKADU_SHARED_PTR<moveit::core::JointModelGroup> modelGroup);

        virtual std::string getRestrictionName();
        virtual bool stateOk(arma::vec joint, geometry_msgs::Pose cartPose);

    };

    class TableRestriction : public Restriction {

    public:

        virtual std::string getRestrictionName();
        virtual bool stateOk(arma::vec joint, geometry_msgs::Pose cartPose);

    };

}

#endif
