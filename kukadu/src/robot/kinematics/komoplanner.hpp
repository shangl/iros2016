#ifndef KUKADU_KOMOPLANNER_H
#define KUKADU_KOMOPLANNER_H

#include <vector>
#include <string>

#include <sensor_msgs/JointState.h>

#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Motion/motion.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <Optim/optimization.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>

#include "kinematics.hpp"
#include "pathplanner.hpp"
#include "simpleplanner.hpp"
#include "../controlqueue.hpp"
#include "../../types/kukadutypes.hpp"

namespace kukadu {

    struct PlanningResult {

        int status;

        double planning_time;

        std::string error_msg;

        ors::Vector pos_error;
        ors::Vector ang_error;

        ors::Transformation resulting_pose;
        trajectory_msgs::JointTrajectory path;

    };

    class KomoPlanner : public PathPlanner, public Kinematics {

    private:

        bool acceptCollision;

        double _positionPrecision;
        double _zeroVelocityPrecision;
        double _jointLimitPrecision;
        double _jointLimitMargin;
        double _jointStatePrecision;
        double _collisionPrecision;
        double _collisionMargin;
        double _alignmentPrecision;
        double _maxIterations;

        // required to ensure that only one instance of the class at the same time (this seems to be a problem for komo)
        static kukadu_mutex oneAtATimeMutex;

        std::string eef_link;
        std::string activeJointsPrefix;

        ors::Vector _pos_tolerance;
        ors::Vector _ang_tolerance;

        MT::String _world_link_name;
        MT::String _support_surface_name;

        ors::KinematicWorld* _world;
        std::vector<std::string> sJointNames;
        std::vector<ors::Joint*> _active_joints;

        KUKADU_SHARED_PTR<ControlQueue> queue;
        KUKADU_SHARED_PTR<SimplePlanner> simplePlanner;

        void display(bool block, const char* msg);
        void setState(const sensor_msgs::JointState &state);
        void ensureJointLimits(ors::KinematicWorld &w, arr &x);
        void setJointPosition(const std::string &name, const double pos);
        void pathToTrajectory(trajectory_msgs::JointTrajectory &traj, const arr &path);
        void setState(const std::vector<std::string>& jointNames, const arma::vec& joints);
        void computePositionError(const ors::Shape &eef, const ors::Shape &target, ors::Vector &error);
        void computePositionError(ors::KinematicWorld &w, const char *eef, const char *target, ors::Vector &error);
        void computeAlignmentError(ors::KinematicWorld &w, const char *eef, const char *target, ors::Vector &error);
        void computeAlignmentError(const ors::Shape &eef, const ors::Shape &target, ors::Vector &error, int axes = 7);

        bool check_goal_state(const arr &desired, const arr &actual);
        bool withinTolerance(ors::Vector &error, ors::Vector &tolerance);
        bool validateJointLimits(arr wp, arr limits, std::string &error_msg);
        bool withinTolerance(ors::Vector &error, ors::Vector &tolerance, int axes);
        bool validateJointLimits(ors::KinematicWorld &w, arr x, std::string &error_msg);
        bool validateCollisions(ors::KinematicWorld &w, const arr &x, std::string &error_msg);

        double keyframeOptimizer(arr& x, MotionProblem& MP, bool x_is_initialized, uint verbose);
        double optimizeEndpose(arr &xT, ors::KinematicWorld &w, const char *link, const char *target, bool allowCollision = true);

        void allowContact(const char* link, bool allow);

    public:

        KomoPlanner(KUKADU_SHARED_PTR<ControlQueue> queue, std::string configPath, std::string mtConfigPath, std::string activeJointsPrefix, bool acceptCollision = false);
        ~KomoPlanner();

        virtual std::vector<arma::vec> planJointTrajectory(std::vector<arma::vec> intermediateJoints);
        virtual std::vector<arma::vec> planCartesianTrajectory(std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);
        virtual std::vector<arma::vec> planCartesianTrajectory(arma::vec startJoints, std::vector<geometry_msgs::Pose> intermediatePoses, bool smoothCartesians = false, bool useCurrentRobotState = true);

        geometry_msgs::Pose computeFk(arma::vec joints);

        virtual geometry_msgs::Pose computeFk(std::vector<double> jointState);
        virtual std::vector<arma::vec> computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal);

    };

}

#endif
