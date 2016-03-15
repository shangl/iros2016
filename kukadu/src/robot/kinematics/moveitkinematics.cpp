#include "moveitkinematics.hpp"

#include <eigen_conversions/eigen_msg.h>

#include "../../utils/utils.hpp"

using namespace ros;
using namespace std;
using namespace arma;

namespace kukadu {

    MoveItKinematics::MoveItKinematics(std::string moveGroupName, std::string tipLink) {

        construct(moveGroupName, tipLink, STD_AVOID_COLLISIONS, STD_MAX_ATTEMPTS, STD_TIMEOUT);

    }

    MoveItKinematics::MoveItKinematics(std::string moveGroupName, std::string tipLink, bool avoidCollisions, int maxAttempts, double timeOut) {

        construct(moveGroupName, tipLink, avoidCollisions, maxAttempts, timeOut);

    }

    void MoveItKinematics::construct(std::string moveGroupName, std::string tipLink, bool avoidCollisions, int maxAttempts, double timeOut) {

        this->moveGroupName = moveGroupName;
        this->tipLink = tipLink;
        this->avoidCollisions = avoidCollisions;
        this->maxAttempts = maxAttempts;
        this->timeOut = timeOut;

        /* Load the robot model */
        rml_.reset(new robot_model_loader::RobotModelLoader("robot_description"));

        robot_model_ = rml_->getModel();
        if(!robot_model_) {
            string error = "(MoveItKinematics) Unable to load robot model - make sure, that the robot_description is uploaded to the server";
            ROS_ERROR("(MoveItKinematics) Unable to load robot model - make sure, that the robot_description is uploaded to the server");
            throw KukaduException(error.c_str());
        }

        // create instance of planning scene
        planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

        jnt_model_group = KUKADU_SHARED_PTR<robot_model::JointModelGroup>(robot_model_->getJointModelGroup(moveGroupName));

        if(!jnt_model_group) {
            ROS_ERROR("(MoveItKinematics) unknown group name");
            return;
        }

        degOfFreedom = jnt_model_group->getJointModelNames().size();
        modelRestriction = KUKADU_SHARED_PTR<Restriction>(new MoveItRestriction(robot_model_, planning_scene_, jnt_model_group));

        if(avoidCollisions)
            addRestriction(modelRestriction);

    }

    geometry_msgs::Pose MoveItKinematics::computeFk(std::vector<double> jointState) {

        geometry_msgs::Pose retPose;

        moveit::core::RobotState state(robot_model_);
        state.setJointGroupPositions(jnt_model_group.get(), jointState);

        const Eigen::Affine3d &eef_state = state.getGlobalLinkTransform(tipLink);
        tf::poseEigenToMsg(eef_state, retPose);

        return retPose;

    }

    std::vector<arma::vec> MoveItKinematics::computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose &goal) {

        vector<vec> retVec;
        vector<double> solution;

        // create robot state instance and set to initial values
        moveit::core::RobotState state(robot_model_);
        state.setToDefaultValues();

        // check, if seed state was provided and is valid
        if(!currentJointState.empty() && currentJointState.size() != jnt_model_group->getJointModelNames().size()) {
            ROS_ERROR("If a seed state is provided, it has to contain exactly one value per joint!");
            return retVec;
        }

        // set seed state if necessary
        if(!currentJointState.empty()) {
            state.setJointGroupPositions(jnt_model_group.get(), currentJointState);
        }

        // compute result
        if(state.setFromIK(jnt_model_group.get(), goal, tipLink, maxAttempts, timeOut, boost::bind(&MoveItKinematics::collisionCheckCallback, this, _1, _2, _3))) {
            // store joint positions
            state.copyJointGroupPositions(jnt_model_group.get(), solution);
            retVec.push_back(stdToArmadilloVec(solution));
        }

        return retVec;

    }

    bool MoveItKinematics::collisionCheckCallback(moveit::core::RobotState *state, const moveit::core::JointModelGroup* joint_group, const double* solution) {

        if(avoidCollisions) {

            vec solVec(degOfFreedom);
            for(int i = 0; i < degOfFreedom; ++i)
                solVec(i) = solution[i];

            geometry_msgs::Pose pose = computeFk(armadilloToStdVec(solVec));

            return checkAllRestrictions(solVec, pose);

        } else
            return true;

    }

    bool MoveItKinematics::isColliding(arma::vec jointState, geometry_msgs::Pose pose) {
        return !modelRestriction->stateOk(jointState, pose);
    }

    KUKADU_SHARED_PTR<Restriction> MoveItKinematics::getModelRestriction() {
        return modelRestriction;
    }

}
