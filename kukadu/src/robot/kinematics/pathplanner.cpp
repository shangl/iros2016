#include "pathplanner.hpp"

namespace kukadu {

    PathPlanner::PathPlanner() {
    }

    void PathPlanner::setCheckCollisions(bool collision) {
        checkCollision = collision;
    }

    bool PathPlanner::getCheckCollision() {
        return checkCollision;
    }

}
