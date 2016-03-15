#ifndef KUKADU_KINEMATICS_H
#define KUKADU_KINEMATICS_H

#include "vector"
#include <ros/ros.h>
#include <armadillo>
#include <geometry_msgs/Pose.h>

#include "restriction/restriction.hpp"
#include "../../types/kukadutypes.hpp"

namespace kukadu {

    class Kinematics {

    private:

        std::vector<KUKADU_SHARED_PTR<Restriction> > restrictions;

    public:

        Kinematics();

        void addRestriction(KUKADU_SHARED_PTR<Restriction> restriction);
        void removeRestriction(KUKADU_SHARED_PTR<Restriction> restriction);

        int getRestrictionsCount();
        int  getRestrictionIdx(KUKADU_SHARED_PTR<Restriction> restriction);

        KUKADU_SHARED_PTR<Restriction> getRestrictionByIdx(int idx);

        bool checkAllRestrictions(arma::vec currentState, geometry_msgs::Pose pose);

        virtual std::vector<arma::vec> computeIk(arma::vec currentJointState, const geometry_msgs::Pose& goal);
        virtual std::vector<arma::vec> computeIk(std::vector<double> currentJointState, const geometry_msgs::Pose& goal) = 0;

        virtual geometry_msgs::Pose computeFk(std::vector<double> jointState) = 0;

    };

}

#endif
