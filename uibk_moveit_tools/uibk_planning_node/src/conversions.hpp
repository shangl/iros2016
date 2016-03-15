#ifndef UIBK_PLANNING_CONVERSIONS_H
#define UIBK_PLANNING_CONVERSIONS_H

#include <vector>
#include <string>
#include <sstream>
#include <moveit_msgs/RobotState.h>

double getJointPositionFromState(const std::string &joint, const moveit_msgs::RobotState &state);
void getJointPositionsFromState(const std::vector<std::string> &joints, const moveit_msgs::RobotState &state, std::vector<double> &values);
void getArmJointNames(const std::string &arm, std::vector<std::string> &names);
void getArmValuesFromState(const std::string &arm, const moveit_msgs::RobotState &state, std::vector<double> &values);

#endif // CONVERSIONS_H
