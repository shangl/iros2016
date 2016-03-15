#include "trajectoryexecutor.hpp"

namespace kukadu {

    TrajectoryExecutor::TrajectoryExecutor() : Controller("simple trajectory executor", 0.0) {

    }

    KUKADU_SHARED_PTR<ControllerResult> TrajectoryExecutor::performAction() {
        return executeTrajectory();
    }

}
