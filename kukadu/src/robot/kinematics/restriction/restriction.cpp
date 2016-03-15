#include "restriction.hpp"

#include "../../../utils/utils.hpp"

using namespace std;

namespace kukadu {

    MoveItRestriction::MoveItRestriction(robot_model::RobotModelPtr robotModel, planning_scene::PlanningScenePtr planningScene, KUKADU_SHARED_PTR<robot_model::JointModelGroup> modelGroup) {

        this->planningScene = planningScene;
        this->modelGroup = modelGroup;
        this->robotModel = robotModel;

    }

    bool MoveItRestriction::stateOk(arma::vec joint, geometry_msgs::Pose cartPose) {

        moveit::core::RobotState state(robotModel);
        double jointStateArray[joint.n_elem];
        for(int i = 0; i < joint.n_elem; ++i)
            jointStateArray[i] = joint[i];
        state.setJointGroupPositions(modelGroup.get(), jointStateArray);
        state.update();

        /*
        if(planningScene->isStateColliding(state, modelGroup->getName())) {
            vector<string> collidingLinks;
            collision_detection::CollisionRequest requ;
            collision_detection::CollisionResult res;
            requ.contacts = true;
            requ.max_contacts = 10000;
            planningScene->getCollidingLinks(collidingLinks, state);
            planningScene->checkCollision(requ, res, state);

            cout << cartPose << endl;

            for(collision_detection::CollisionResult::ContactMap::iterator it = res.contacts.begin(); it != res.contacts.end(); ++it) {
                cout << "collission: " << it->first.first << " " << it->first.second << endl;
            }

            cout << "colliding links: ";
            for(int i = 0; i < collidingLinks.size(); ++i) {
                cout << collidingLinks.at(i) << endl;
            }

        }
        */

        return !planningScene->isStateColliding(state, modelGroup->getName());

    }

    std::string MoveItRestriction::getRestrictionName() {
        return string("MoveItRestriction");
    }

    bool TableRestriction::stateOk(arma::vec joint, geometry_msgs::Pose cartPose) {

        if(cartPose.position.z < 0) {
            if(cartPose.position.x > 0) {
                return false;
            }
        }

        return true;

    }

    std::string TableRestriction::getRestrictionName() {
        return string("TableRestriction");
    }

}
