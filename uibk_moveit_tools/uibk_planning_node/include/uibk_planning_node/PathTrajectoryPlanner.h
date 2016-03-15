#ifndef PATHTRAJECTORYPLANNER_H
#define PATHTRAJECTORYPLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetCartesianPath.h>

#include "TrajectoryPlanner.h"
#include "../../src/PlannerBase.h"

namespace trajectory_planner_moveit {


class PathTrajectoryPlanner : public PlannerBase {

private:

    double eef_step_;
    double jump_threshold_;

    std::string finalLinkName;

    TrajectoryPlanner trajectory_planner_;

    std::vector<std::string> jointNames;

    ros::ServiceClient planning_client_;


public:

    PathTrajectoryPlanner(ros::NodeHandle &nh, moveit::planning_interface::MoveGroup& group, std::vector<std::string> jointNames, std::string finalLinkName, std::string kinematicPathTopic = UIBK_STD_GROUP_NAME);
    ~PathTrajectoryPlanner() {}

    void setJumpThreshold(double value) { jump_threshold_ = value; }
    double setJumpThreshold() { return jump_threshold_; }

    void setEEFStep(double value) { eef_step_ = value; }
    double getEEFStep() { return eef_step_; }

    const std::string getName();

    virtual bool plan(const geometry_msgs::PoseStamped &goal, moveit_msgs::MotionPlanResponse &solution);
    virtual bool plan(const geometry_msgs::PoseStamped &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state);

};


}


#endif // PATHTRAJECTORYPLANNER_H
