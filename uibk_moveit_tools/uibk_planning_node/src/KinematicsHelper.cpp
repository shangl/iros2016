#include "KinematicsHelper.h"

using namespace std;
using namespace ros;

namespace trajectory_planner_moveit {

KinematicsHelper::KinematicsHelper(NodeHandle &nh, string ikTopic, string fkTopic) {
    this->ikTopic = ikTopic;
    this->fkTopic = fkTopic;
    ik_client_ = nh.serviceClient<moveit_msgs::GetPositionIK>(ikTopic);
    fk_client_ = nh.serviceClient<moveit_msgs::GetPositionFK>(fkTopic);
}

KinematicsHelper::~KinematicsHelper() {

}

bool KinematicsHelper::computeIK(const string &groupName, const geometry_msgs::PoseStamped &goal, moveit_msgs::RobotState &solution, const bool avoid_collisions, const int attempts, const double timeout) {

	sensor_msgs::JointState state;
    return computeIK(groupName, goal, state, solution, avoid_collisions, attempts, timeout);

}

bool KinematicsHelper::computeIK(const string &groupName, const geometry_msgs::PoseStamped &goal, const sensor_msgs::JointState &seed_state, moveit_msgs::RobotState &solution, const bool avoid_collisions, const int attempts, const double timeout) {

    ROS_DEBUG_NAMED("KinematicsHelper", "IK request received for group '%s'", groupName.c_str());

	moveit_msgs::GetPositionIKRequest request;

    request.ik_request.group_name = groupName;
	request.ik_request.pose_stamped = goal;
	request.ik_request.attempts = attempts;
	request.ik_request.timeout = ros::Duration(timeout);
	request.ik_request.avoid_collisions = avoid_collisions;
    request.ik_request.ik_link_name = "link5";

	moveit_msgs::RobotState seed;
	seed.joint_state = seed_state;
	request.ik_request.robot_state = seed;

	return computeIKInternal(request, solution);
}

bool KinematicsHelper::computeIKInternal(const moveit_msgs::GetPositionIKRequest &request, moveit_msgs::RobotState &solution) {
	moveit_msgs::GetPositionIKResponse response;

	ROS_DEBUG_NAMED("KinematicsHelper", "Calling IK Service...");
	if(!ik_client_.exists()) {
		ROS_ERROR("The IK service client is not accessable - maybe MoveIt was not lauched correctly.");
		return false;
	}

	if(ik_client_.call(request, response)) {

		if(response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
			solution = response.solution;
			ROS_DEBUG_NAMED("KinematicsHelper", "IK solution successfully calculated");

			return true;
        } else {
			ROS_WARN_NAMED("KinematicsHelper", "IK calculation failed with error code '%d'", response.error_code.val);
			return false;
		}

	} else {
		ROS_ERROR("IK Service call failed! Maybe MoveIt was not launched properly.");
		return false;
	}
}

bool KinematicsHelper::computeFK(const moveit_msgs::RobotState &state, const string &link_name, geometry_msgs::Pose &solution, const string &frame_id) {
	ROS_DEBUG_NAMED("KinematicsHelper", "FK request received for link '%s'.", link_name.c_str());

	if(!fk_client_.exists()) {
		ROS_ERROR("The FK service client is not accessable - maybe MoveIt was not lauched correctly.");
		return false;
	}

	moveit_msgs::GetPositionFKRequest request;

	request.header.stamp = ros::Time::now();
	request.header.frame_id = frame_id;
	request.robot_state = state;
	request.fk_link_names.push_back(link_name);

	moveit_msgs::GetPositionFKResponse response;

	ROS_DEBUG_NAMED("KinematicsHelper", "Calling FK Service...");

	if(fk_client_.call(request, response)) {

		if(response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
			ROS_DEBUG_NAMED("KinematicsHelper", "Response contains %d results", (int)response.fk_link_names.size());
			ROS_DEBUG_NAMED("KinematicsHelper", "First link in result set: '%s'", response.fk_link_names[0].c_str());

			solution = response.pose_stamped[0].pose;

            ROS_DEBUG_NAMED("KinematicsHelper", "FK pose successfully calculated");

			return true;
		} else {
			ROS_WARN_NAMED("KinematicsHelper", "FK calculation failed with error code '%d'", response.error_code.val);
			return false;
		}

	} else {
		ROS_ERROR("IK Service call failed! Maybe MoveIt was not launched properly.");
		return false;
	}
}




}
