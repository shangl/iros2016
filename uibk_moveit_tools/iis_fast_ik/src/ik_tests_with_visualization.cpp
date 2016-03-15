#include <visualization_msgs/Marker.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <../include/iis_fast_ik/kinematics.h>

using namespace std;

ros::Publisher state_pub_;
ros::Publisher marker_pub_;

void printSolution(const vector<double> &values) {

    for(size_t i = 0; i < values.size(); ++i) {
        stringstream ss;
        ss << "right_arm_" << i << "_joint";
        ROS_INFO("%s: %1.4f", ss.str().c_str(), values[i]);
    }
}

void displaySolution(const vector<double> &values)  {
    moveit_msgs::DisplayRobotState msg;

    msg.state.joint_state.name.push_back("right_arm_0_joint");
    msg.state.joint_state.name.push_back("right_arm_1_joint");
    msg.state.joint_state.name.push_back("right_arm_2_joint");
    msg.state.joint_state.name.push_back("right_arm_3_joint");
    msg.state.joint_state.name.push_back("right_arm_4_joint");
    msg.state.joint_state.name.push_back("right_arm_5_joint");
    msg.state.joint_state.name.push_back("right_arm_6_joint");

    msg.state.joint_state.position = values;

    state_pub_.publish(msg);

    ros::Duration(2).sleep();
}

void showMarker(const geometry_msgs::Pose pose) {
    visualization_msgs::Marker msg;

    msg.header.frame_id = "world_link";
    msg.header.stamp = ros::Time::now();

    msg.ns = "goal1";
    msg.id = 1;
    msg.type = visualization_msgs::Marker::SPHERE;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose = pose;

    msg.scale.x = 0.05;
    msg.scale.y = 0.05;
    msg.scale.z = 0.05;

//    msg.color.r = 1;
    msg.color.g = 1;
//    msg.color.b = 1;
    msg.color.a = 1;

    marker_pub_.publish(msg);
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "ik_tests");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    state_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>("uibk_robot_state", 1, true);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("markers", 1, true);

    ROS_INFO("Create test poses");

    geometry_msgs::Pose goal1;

    goal1.position.x = 0.186961;
    goal1.position.y = 0.424272;
    goal1.position.z = 0.543895;
    goal1.orientation.x = -0.230403;
    goal1.orientation.y = -0.673347;
    goal1.orientation.z = 0.484887;
    goal1.orientation.w = -0.508336;

    ROS_INFO("Display test pose 1");
    showMarker(goal1);

    // create an instance of our KinematicsHelper class
    // be sure that robot description and kinematics.yaml is on the parameter server
    uibk_kinematics::Kinematics kinematics;

    // this vector holds the solution in case of success.
    vector<double> solution;

    ROS_INFO("Computing test pose 1 with default tip link");
    // call the function in our helper class.
    if(kinematics.computeIK("right", goal1, solution, false)) {
        printSolution(solution);
        displaySolution(solution);
    } else {
        ROS_ERROR("Computation failed");
        return EXIT_FAILURE;
    }

    cout << endl;
    ROS_INFO("Computing test pose 1 for right_sdh_grasp_link as tip link");
    if(kinematics.computeIK("right", goal1, "right_sdh_grasp_link", solution,  solution, false)) {
        printSolution(solution);
        displaySolution(solution);
    } else {
        ROS_ERROR("Computation failed");
        return EXIT_FAILURE;
    }

    cout << endl;
    ROS_INFO("Computing test pose 1 for right_sdh_tip_link as tip link");
    if(kinematics.computeIK("right", goal1, "right_sdh_tip_link", solution, solution, false)) {
        printSolution(solution);
        displaySolution(solution);
    } else {
        ROS_ERROR("Computation failed");
        return EXIT_FAILURE;
    }

    ROS_INFO("All tests successfully executed.");

    return EXIT_SUCCESS;
}



