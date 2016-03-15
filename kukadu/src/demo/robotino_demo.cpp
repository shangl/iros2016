#include <vector>
#include <iostream>
//#include <kukadu/kukadu.h>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectoryRequest.h>
#include <moveit_msgs/GetMotionPlan.h>

using namespace std;

string groupPrefix = "Arm";
ros::ServiceClient planning_client_;

bool plan(const geometry_msgs::Pose &goal,
                             moveit_msgs::MotionPlanResponse &solution,
                             const sensor_msgs::JointState &start_state) {

    if (!planning_client_.exists()) {
        ROS_ERROR_STREAM("Unable to connect to planning service - ensure that MoveIt is launched!");

        return false;
    }

    moveit_msgs::GetMotionPlanRequest get_mp_request;
    moveit_msgs::MotionPlanRequest &request = get_mp_request.motion_plan_request;

    request.group_name = groupPrefix;
    request.num_planning_attempts = 5;
    request.allowed_planning_time = 5.0;
    //request.planner_id = "LBKPIECEkConfigDefault";
    //request.planner_id = "PRMkConfigDefault";
    request.start_state.joint_state = start_state;

    ROS_INFO("Computing possible IK solutions for goal pose");

    moveit_msgs::Constraints c;

    /*
    moveit_msgs::PositionConstraint pc;

    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = 1;
    pc.link_name = "link5";
    pc.target_point_offset.z = -0.5;
    //pc.constraint_region.
    pc.weight = 1.0;

    c.position_constraints.push_back(pc);
    */

    // this is working
    for(int i = 0; i < 5; ++i) {
        stringstream s;
        s << "joint" << (i + 1);
        moveit_msgs::JointConstraint js;
        js.position = start_state.position.at(i);
        if(i == 0) js.position += 0.4;
        js.joint_name = s.str();
        js.tolerance_above = 1e-4;
        js.tolerance_below = 1e-4;
        js.weight = 1.0;
        c.joint_constraints.push_back(js);
    }

    request.goal_constraints.push_back(c);

    /*
    // compute a set of ik solutions and construct goal constraint
    for (int i = 0; i < 5; ++i) {
        moveit_msgs::RobotState ik_solution;

        geometry_msgs::PoseStamped pose_goal;
        pose_goal.header.stamp = ros::Time::now();
        pose_goal.header.frame_id = FRAME_ID;
        pose_goal.pose = goal;

        if(kin_helper_.computeIK(arm_, pose_goal, start_state, ik_solution)) {
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
    */

    if(request.goal_constraints.size() == 0) {
        ROS_WARN("No valid IK solution found for given pose goal - planning failed!");
        return false;
    }

    ROS_DEBUG("Found %d valid IK solutions for given pose goal", (int)request.goal_constraints.size());
    ROS_DEBUG("Calling planning service...");

    moveit_msgs::GetMotionPlanResponse get_mp_response;

    bool success = planning_client_.call(get_mp_request, get_mp_response);
    solution = get_mp_response.motion_plan_response;
    int error_code = solution.error_code.val;

    if(success) {

        int pts_count = (int) solution.trajectory.joint_trajectory.points.size();

        if(error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Planning failed");
            cout << " " << error_code;
            return false;
        }

        ROS_INFO("Solution found for planning problem .");
        return true;

    } else {
        ROS_INFO("Planning failed");
        cout << " " << error_code;
        return false;
    }

}

bool executePlan(moveit_msgs::RobotTrajectory& trajectory, ros::ServiceClient& execution_client) {

    moveit_msgs::ExecuteKnownTrajectory msg;
    moveit_msgs::ExecuteKnownTrajectoryRequest &request = msg.request;
    request.wait_for_execution = true;
    request.trajectory = trajectory;
    bool success = execution_client.call(msg);
    if (success) {
        moveit_msgs::MoveItErrorCodes &code = msg.response.error_code;
        if (code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Execution finished successfully.");
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


int main(int argc, char** args) {

    ros::init(argc, args, "robotino_demo"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Retrieving joint state...");
    moveit::planning_interface::MoveGroup group(groupPrefix);
    group.setEndEffectorLink("link5");

    vector<double> jointVals = group.getCurrentJointValues();
    for(int i = 0; i < jointVals.size(); ++i) {
        cout << jointVals.at(i) << " ";
    }

    ROS_INFO("Connecting to planning service...");

    string topic = "plan_kinematic_path";
    planning_client_ = node->serviceClient<moveit_msgs::GetMotionPlan>(topic);

    geometry_msgs::Pose newPose;
    // 0 0 0 0 position
    newPose.position.x = -0.00165411 + 0.1;
    newPose.position.y = -0.000955;
    newPose.position.z = 0.86 - 0.5;
    newPose.orientation.x = 0;
    newPose.orientation.y = 0;
    newPose.orientation.z = -0.5;
    newPose.orientation.w = 0.866025;

    robot_state::RobotState start_state(*group.getCurrentState());
    group.setStartState(start_state);

    /*
    moveit_msgs::RobotTrajectory trajectory;
    vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(newPose);
    double fraction = group.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

    */

    sensor_msgs::JointState js;
    js.position = jointVals;

    moveit_msgs::MotionPlanResponse p;
    ROS_INFO("Starting to plan...");
    plan(newPose, p, js);
    moveit_msgs::RobotTrajectory trajectory = p.trajectory;

    ros::ServiceClient execution_client = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");
    // cout << trajectory << endl;
    cout << "success: " << executePlan(trajectory, execution_client) << endl;

    getchar();

    return 0;

}
