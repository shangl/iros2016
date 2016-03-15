/*
 * plan_execution.h
 *
 *  Created on: Mar 18, 2014
 *      Author: Martin Griesser
 */

#ifndef PLANNING_HELPER_H_
#define PLANNING_HELPER_H_

/*
 * PlanExecution.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: Martin Griesser
 */

#include <ros/ros.h>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/Grasp.h>


#define CAN_LOOK false
#define ALLOW_REPLAN false
#define SUPPORT_SURFACE "table_surface_link"
#define FRAME_ID "world_link"

namespace uibk_planning_node {

struct PlanningResult {
	int type;
	int status;
	std::string status_msg;
	std::string object_id;
	moveit_msgs::RobotState start_state;
	std::vector<moveit_msgs::RobotTrajectory> trajectory_stages;
	std::vector<std::string> trajectory_descriptions;
};

typedef boost::shared_ptr<PlanningResult> PlanningResultPtr;


class PlanningHelper {

public:

	static const int SUCCESS = 1;
	static const int FAILURE = 0;

	static const int PICK = 0;
	static const int PLACE = 1;
	static const int GOAL = 2;

	PlanningHelper(const std::string &arm);
    virtual ~PlanningHelper() {}

    PlanningResultPtr plan_pick(const std::string &object, const std::vector<moveit_msgs::Grasp> &grasps);
    PlanningResultPtr plan_place(const std::string &object, const std::vector<moveit_msgs::PlaceLocation> &locations);
	PlanningResultPtr plan(const geometry_msgs::Pose &pose_goal);

    bool execute(const PlanningResultPtr &plan);

	void setPlanningGroupName(const std::string &name);
    std::string getPlanningGroupName();

	void setPlannerId(const std::string &planner_id);
	std::string getPlannerId();

	void setAllowedPlanningTime(double value);
	double getAllowedPlanningTime();

	void setPlanningAttempts(int value);
	int getPlanningAttempts();

	void setSupportSurfaceName(const std::string name);
	std::string getSupportSurfaceName();

private:

    bool execute(const moveit_msgs::RobotTrajectory &trajectory);
	bool attachObject(const std::string &object);
	bool detachObject(const std::string &object);

	ros::Publisher pub_;

	std::string group_name_;
	std::string end_effector_;
	std::string end_effector_link_;
	double planning_time_;
	int planning_attempts_;
	double goal_joint_tolerance_;
	double goal_position_tolerance_;
	double goal_orientation_tolerance_;
	std::string planner_id_;
	std::string support_surface_;

	ros::NodeHandle node_handle_;

	ros::ServiceClient execution_client_;
	ros::Publisher attached_object_publisher_;

	boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction> > pick_action_client_;
	boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction> > place_action_client_;
	boost::scoped_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > move_action_client_;

	robot_model::RobotModelConstPtr robot_model_;
	const robot_model::JointModelGroup *joint_model_group_;
};

typedef boost::shared_ptr<PlanningHelper> PlanningHelperPtr;

}

#endif /* PLANNING_HELPER_H_ */
