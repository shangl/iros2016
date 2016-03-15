#ifndef TRAJECTORYTOOLS_H
#define TRAJECTORYTOOLS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/RobotTrajectory.h>

#include "KinematicsHelper.h"

namespace trajectory_planner_moveit {

class TrajectoryTools {

public:

    static double computeTrajectoryDistanceRatio(const std::string &arm,
                                          const moveit_msgs::RobotTrajectory &trajectory,
                                          KinematicsHelper &ki_helper);

    static double computeDistance(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);

};

}

#endif // TRAJECTORYTOOLS_H
