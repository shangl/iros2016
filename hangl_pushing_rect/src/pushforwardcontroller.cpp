#include <hangl_pushing_rect/pushforwardcontroller.hpp>

#include <sstream>
#include <armadillo>
#include <kukadu/kukadu.h>
#include <hangl_pushing_rect/utils.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

namespace pushing {

    PushForwardController::PushForwardController(std::shared_ptr<ControlQueue> rightQueue, std::shared_ptr<GenericHand> rightHand, std::string trajectoryPath,
                                                 double az, double bz, double ac, double tolAbsErr, double tolRelErr, bool goBackToBlockingPos)
                    : Controller("push forward", 0.01) {

        this->ac = ac;
        this->maxForce = 8;
        this->tolAbsErr = tolAbsErr;
        this->tolRelErr = tolRelErr;
        this->rightHand = rightHand;
        this->rightQueue = rightQueue;
        this->goBackToBlockingPos = goBackToBlockingPos;
        dataFinalPush = SensorStorage::readStorage(rightQueue, trajectoryPath);
        timesFinalPush = dataFinalPush->getTimes();
        learnerFinalPush = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesFinalPush, dataFinalPush->getJointPos())));
        dmpFinalPush = learnerFinalPush->fitTrajectories();

    }

    std::shared_ptr<ControllerResult> PushForwardController::performAction() {

        return performAction(3.2);

    }

    void PushForwardController::setMaxForce(double maxForce) {
        this->maxForce = maxForce;
    }

    std::shared_ptr<ControllerResult> PushForwardController::performAction(double tmax) {

        goToRightHandBlocking(rightHand);
        goToBlockingPos(rightQueue);

        vec startPos = dmpFinalPush->getY0();
        simpleJointPtp(rightQueue, startPos);

        DMPExecutor execTransPushForward(dmpFinalPush, rightQueue);
        execTransPushForward.doRollBackOnMaxForceEvent(false);
        execTransPushForward.enableMaxForceMode(maxForce, DMPExecutor::IGNORE_FORCE, DMPExecutor::IGNORE_FORCE, DMPExecutor::IGNORE_FORCE);
        execTransPushForward.executeTrajectory(ac, 0, tmax, tolAbsErr, tolRelErr);

        if(goBackToBlockingPos)
            goToBlockingPos(rightQueue);

        return nullptr;

    }

    void PushForwardController::setGoBackToBlockingPos(bool goToBlockingPos) {
        this->goBackToBlockingPos = goToBlockingPos;
    }

    KUKADU_SHARED_PTR<kukadu::ControlQueue> PushForwardController::getQueue() {
        return rightQueue;
    }

    KUKADU_SHARED_PTR<kukadu::GenericHand> PushForwardController::getHand() {
        return rightHand;
    }

}
