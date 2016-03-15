#ifndef PLANNERBASE_H
#define PLANNERBASE_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/MotionPlanResponse.h>

namespace trajectory_planner_moveit {


class PlannerBase {

protected:

    std::string groupName;

public:

    virtual ~PlannerBase() {}

    virtual const std::string getName() = 0;
    virtual bool plan(const geometry_msgs::PoseStamped &goal, moveit_msgs::MotionPlanResponse &solution) = 0;
    virtual bool plan(const geometry_msgs::PoseStamped &goal, moveit_msgs::MotionPlanResponse &solution, const sensor_msgs::JointState &start_state) = 0;

    void setMoveGroup(const std::string& groupName) { this->groupName = groupName; }
    std::string getMoveGroup() { return groupName; }

};

}

#endif // PLANNERBASE_H
