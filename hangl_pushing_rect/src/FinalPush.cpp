#include <hangl_pushing_rect/utils.hpp>
#include <hangl_pushing_rect/FinalPush.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

namespace pushing {

    FinalPushController::FinalPushController(std::shared_ptr<ControlQueue> leftQueue, std::string trajectoryPath, double az, double bz, double ac, double tolAbsErr, double tolRelErr)
                    : Controller("final push", 0.01) {

        this->ac = ac;
        this->tolAbsErr = tolAbsErr;
        this->tolRelErr = tolRelErr;
        this->leftQueue = leftQueue;
        dataFinalPush = SensorStorage::readStorage(leftQueue, trajectoryPath);
        timesFinalPush = dataFinalPush->getTimes();
        learnerFinalPush = shared_ptr<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesFinalPush, dataFinalPush->getJointPos())));
        dmpFinalPush = learnerFinalPush->fitTrajectories();

    }

    void FinalPushController::prepare() {

        if(leftQueue->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
            leftQueue->stopCurrentMode();
            setStandardStiffness(leftQueue);
            leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
        }

        setStandardStiffness(leftQueue);

        //moveHome(leftQueue, KUKADU_SHARED_PTR<ControlQueue>(), false);

        SimplePlanner simPlan(leftQueue, KUKADU_SHARED_PTR<kukadu::Kinematics>());
        vector<arma::vec> propPlan = {
            leftQueue->getCurrentJoints().joints,
            stdToArmadilloVec({-0.9692263007164001, 1.113829493522644, 1.1473214626312256, -1.444376826286316, -0.28663957118988037, -0.8957559466362, -0.2651996612548828}),
            stdToArmadilloVec({-0.109378, 1.55693, 1.50002, -0.224965, -0.553483, -0.614582, -0.658589}),
            stdToArmadilloVec({-0.1002, 1.54825, 1.51948, -0.22662, -0.543338, -0.614317, -0.658062}),
            stdToArmadilloVec({-0.0793886, 1.54343, 1.5273, -0.222725, -0.54333, -0.607497, -0.669542}),
            stdToArmadilloVec({-0.0529747, 1.53886, 1.52831, -0.217783, -0.524916, -0.607438, -0.692989}),
            stdToArmadilloVec({-0.0422586, 1.53685, 1.52831, -0.21765, -0.515485, -0.606345, -0.705268}),
            stdToArmadilloVec({-0.0237716, 1.53687, 1.52831, -0.21757, -0.515368, -0.604097, -0.705288}),
            stdToArmadilloVec({-0.0156862, 1.53686, 1.52831, -0.217573, -0.515372, -0.59964, -0.708731}),
            stdToArmadilloVec({-0.0156172, 1.53727, 1.52831, -0.219744, -0.511537, -0.566932, -0.708706}),
            stdToArmadilloVec({-0.0390551, 1.53972, 1.52831, -0.262562, -0.576725, -0.567193, -0.683141}),
            dmpFinalPush->getY0()
        };


        vector<arma::vec> jp = simPlan.planJointTrajectory(propPlan);
        for(auto j : jp) {
            leftQueue->addJointsPosToQueue(j);
        }
        leftQueue->synchronizeToControlQueue(1);

        sleep(1.0);


    }

    void FinalPushController::cleanUp() {

        simpleJointPtp(leftQueue, stdToArmadilloVec({-0.9692263007164001, 1.113829493522644, 1.1473214626312256, -1.444376826286316, -0.28663957118988037, -0.8957559466362, -0.2651996612548828}));
        moveHome(leftQueue, KUKADU_SHARED_PTR<ControlQueue>(), false);

    }

    void FinalPushController::performCore() {

        performCore(dmpFinalPush->getTmax());

    }

    void FinalPushController::performCore(double tmax) {

        DMPExecutor execFinalPush(dmpFinalPush, leftQueue);

        // execFinalPush.setRollbackTime(0.3);
        execFinalPush.doRollBackOnMaxForceEvent(false);
        // sets maximum force to 20 newtons (2 kg) --> if exceeded, the controller rolls back
        execFinalPush.enableMaxForceMode(DMPExecutor::IGNORE_FORCE, 10, DMPExecutor::IGNORE_FORCE, 10);
        execFinalPush.executeTrajectory(ac, 0, tmax, tolAbsErr, tolRelErr);

    }

    std::shared_ptr<ControllerResult> FinalPushController::performAction() {

        prepare();
        performCore();
        cleanUp();

        return nullptr;

    }

}
