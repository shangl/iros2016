#include "conversions.hpp"

#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/ExecuteKnownTrajectoryRequest.h>

#include <uibk_planning_node/TrajectoryPlanner.h>

using namespace std;

namespace trajectory_planner_moveit {

TrajectoryPlanner::TrajectoryPlanner(ros::NodeHandle &nh, moveit::planning_interface::MoveGroup& group, const std::vector<string> jointNames, std::string kinematicPathTopic) : kin_helper_(nh), _group(group) {

    this->nh = nh;
    ROS_INFO("Connecting to planning service...");

    string topic = kinematicPathTopic;
    planning_client_ = nh.serviceClient<moveit_msgs::GetMotionPlan>(topic);

    this->groupName = group.getName();
    this->jointNames = jointNames;

    // set some default values
    planning_time_ = 5.0;
    planning_attempts_ = 5;
    max_traj_pts_ = 50;
    goal_joint_tolerance_ = 1e-4;
    goal_position_tolerance_ = 1e-4; // 0.1 mm
    goal_orientation_tolerance_ = 1e-3; // ~0.1 deg
    planner_id_ = "";

    execution_client = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");

}

TrajectoryPlanner::~TrajectoryPlanner() {

}

bool TrajectoryPlanner::executePlan(moveit_msgs::RobotTrajectory& trajectory, ros::ServiceClient& execution_client) {

    moveit_msgs::ExecuteKnownTrajectory msg;
    moveit_msgs::ExecuteKnownTrajectoryRequest &request = msg.request;
    request.wait_for_execution = true;
    request.trajectory = trajectory;
    bool success = execution_client.call(msg);
    if (success) {
        moveit_msgs::MoveItErrorCodes &code = msg.response.error_code;
        if (code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        //    ROS_INFO("Execution finished successfully.");
        } else {
            ROS_ERROR("Execution finished with error_code '%d'", code.val);
            return false;
        }
    } else {
        ROS_ERROR("Execution failed!");
        return false;
    }
    return true;

}

bool TrajectoryPlanner::executePlan(moveit_msgs::MotionPlanResponse& trajectory, ros::ServiceClient& execution_client) {
    return executePlan(trajectory.trajectory, execution_client);
}

bool TrajectoryPlanner::executePlan(moveit_msgs::RobotTrajectory& trajectory) {
    return executePlan(trajectory, execution_client);
}

bool TrajectoryPlanner::executePlan(moveit_msgs::MotionPlanResponse& trajectory) {
    return executePlan(trajectory, execution_client);
}

void TrajectoryPlanner::setPlannerId(const string &planner_id) {
    planner_id_ = planner_id;
}

string TrajectoryPlanner::getPlannerId() {
    return planner_id_;
}

void TrajectoryPlanner::setAllowedPlanningTime(double value) {
    planning_time_ = value;
}

double TrajectoryPlanner::getAllowedPlanningTime() {
    return planning_time_;
}

void TrajectoryPlanner::setPlanningAttempts(int value) {
    planning_attempts_ = value;
}

int TrajectoryPlanner::getPlanningAttempts() {
    return planning_attempts_;
}

void TrajectoryPlanner::setMaxTrajectoryPoints(int value) {
    max_traj_pts_ = value;
}

int TrajectoryPlanner::getMaxTrajectoryPoints() {
    return max_traj_pts_;
}

const string TrajectoryPlanner::getName() {
    return "TrajectoryPlanner";
}

bool TrajectoryPlanner::plan(std::vector<double> &jointPos, moveit_msgs::MotionPlanResponse &solution) {

    sensor_msgs::JointState start_state;
    start_state.position = _group.getCurrentJointValues();
    if (!planning_client_.exists()) {
        ROS_ERROR_STREAM("Unable to connect to planning service - ensure that MoveIt is launched!");

        return false;
    }

    moveit_msgs::GetMotionPlanRequest get_mp_request;
    moveit_msgs::MotionPlanRequest &request = get_mp_request.motion_plan_request;

    request.group_name = groupName;
    request.num_planning_attempts = planning_attempts_;
    request.allowed_planning_time = planning_time_;
    request.planner_id = planner_id_;
    request.start_state.joint_state = start_state;

    vector<string> joint_names = jointNames;

    moveit_msgs::Constraints c;
    c.joint_constraints.resize(joint_names.size());

    for (int j = 0; j < joint_names.size(); ++j) {
        moveit_msgs::JointConstraint &jc = c.joint_constraints[j];
        jc.joint_name = joint_names[j];
        jc.position = jointPos[j];
        jc.tolerance_above = 1e-4;
        jc.tolerance_below = 1e-4;
        jc.weight = 1.0;
    }
    request.goal_constraints.push_back(c);

    moveit_msgs::GetMotionPlanResponse get_mp_response;

    bool success = planning_client_.call(get_mp_request, get_mp_response);
    solution = get_mp_response.motion_plan_response;

    if(success) {
        int pts_count = (int)solution.trajectory.joint_trajectory.points.size();
        int error_code = solution.error_code.val;

        if(error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_DEBUG_STREAM("Planning failed with status code '" << solution.error_code.val << "'");
            return false;
        }

        if(pts_count > max_traj_pts_) {
            ROS_WARN("Valid solution found but contains to many points.");
            return false;
        }

        ROS_DEBUG("Solution found for planning problem .");
        return true;

    } else {
        ROS_DEBUG_STREAM("Planning failed with status code '" << solution.error_code.val << "'");
        return false;
    }

}

bool TrajectoryPlanner::plan(const geometry_msgs::PoseStamped &goal, moveit_msgs::MotionPlanResponse &solution) {

    sensor_msgs::JointState start_state;
    start_state.position = _group.getCurrentJointValues();
    return plan(goal, solution, start_state);

}

bool TrajectoryPlanner::plan(const geometry_msgs::PoseStamped &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state) {

	if (!planning_client_.exists()) {
		ROS_ERROR_STREAM("Unable to connect to planning service - ensure that MoveIt is launched!");

		return false;
	}

	moveit_msgs::GetMotionPlanRequest get_mp_request;
	moveit_msgs::MotionPlanRequest &request = get_mp_request.motion_plan_request;

    request.group_name = groupName;
	request.num_planning_attempts = planning_attempts_;
	request.allowed_planning_time = planning_time_;
	request.planner_id = planner_id_;
	request.start_state.joint_state = start_state;

    ROS_DEBUG("Computing possible IK solutions for goal pose");

    vector<string> joint_names = jointNames;

	// compute a set of ik solutions and construct goal constraint
	for (int i = 0; i < 5; ++i) {
		moveit_msgs::RobotState ik_solution;

        geometry_msgs::PoseStamped pose_goal = goal;
		pose_goal.header.stamp = ros::Time::now();

        if(kin_helper_.computeIK(groupName, pose_goal, start_state, ik_solution)) {
        //if(kin_helper_.computeIK(groupName, pose_goal, start_state, ik_solution, false, 10, 10.0)) {
			vector<double> values;
			getJointPositionsFromState(joint_names, ik_solution, values);

			moveit_msgs::Constraints c;
			c.joint_constraints.resize(joint_names.size());

			for (int j = 0; j < joint_names.size(); ++j) {
				moveit_msgs::JointConstraint &jc = c.joint_constraints[j];
				jc.joint_name = joint_names[j];
				jc.position = values[j];
				jc.tolerance_above = 1e-4;
				jc.tolerance_below = 1e-4;
				jc.weight = 1.0;
			}
			request.goal_constraints.push_back(c);
		}
	}

	if(request.goal_constraints.size() == 0) {
		ROS_WARN("No valid IK solution found for given pose goal - planning failed!");
		return false;
	}

	ROS_DEBUG("Found %d valid IK solutions for given pose goal", (int)request.goal_constraints.size());
	ROS_DEBUG("Calling planning service...");

	moveit_msgs::GetMotionPlanResponse get_mp_response;

	bool success = planning_client_.call(get_mp_request, get_mp_response);
	solution = get_mp_response.motion_plan_response;

	if(success) {
		int pts_count = (int)solution.trajectory.joint_trajectory.points.size();
		int error_code = solution.error_code.val;

		if(error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
			ROS_DEBUG_STREAM("Planning failed with status code '" << solution.error_code.val << "'");
			return false;
		}

		if(pts_count > max_traj_pts_) {
			ROS_WARN("Valid solution found but contains to many points.");
			return false;
		}

		ROS_DEBUG("Solution found for planning problem .");
		return true;

	} else {
		ROS_DEBUG_STREAM("Planning failed with status code '" << solution.error_code.val << "'");
		return false;
	}

}


}
