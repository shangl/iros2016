#ifndef KUKADU_PATHPLANNER_H
#define KUKADU_PATHPLANNER_H

#include <vector>
#include <armadillo>
#include <geometry_msgs/Pose.h>

#include "../../types/kukadutypes.hpp"

namespace kukadu {

    class PathPlanner {

    private:

        bool checkCollision;

    public:

        PathPlanner();

        void setCheckCollisions(bool collision);

        bool getCheckCollision();

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints) = 0;
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians, bool useCurrentRobotState) = 0;

#ifdef CPP11SUPPORTED
        static constexpr int RESULT_FAILED = 0;
        static constexpr int RESULT_SUCCESS = 1;
        static constexpr int RESULT_APPROXIMATE = 2;
#else
        static const int RESULT_FAILED = 0;
        static const int RESULT_SUCCESS = 1;
        static const int RESULT_APPROXIMATE = 2;
#endif

    };

}

#endif
