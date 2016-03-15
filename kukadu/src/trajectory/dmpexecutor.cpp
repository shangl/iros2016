#include "dmpexecutor.hpp"

#include <math.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace arma;

namespace kukadu {

    DMPExecutor::DMPExecutor(KUKADU_SHARED_PTR<Dmp> traj, KUKADU_SHARED_PTR<ControlQueue> execQueue) {

        construct(traj, execQueue, 1);

    }

    DMPExecutor::DMPExecutor(KUKADU_SHARED_PTR<Dmp> traj, KUKADU_SHARED_PTR<ControlQueue> execQueue, int suppressMessages) {

        construct(traj, execQueue, suppressMessages);

    }

    DMPExecutor::DMPExecutor(KUKADU_SHARED_PTR<Trajectory> traj, KUKADU_SHARED_PTR<ControlQueue> execQueue) {

        KUKADU_SHARED_PTR<Dmp> dmp = KUKADU_DYNAMIC_POINTER_CAST<Dmp>(traj);
        construct(dmp, execQueue, suppressMessages);

    }

    void DMPExecutor::construct(KUKADU_SHARED_PTR<Dmp> traj, KUKADU_SHARED_PTR<ControlQueue> execQueue, int suppressMessages) {

        // max force safety is switched of
        doRollback = true;
        maxAllowedForce = DBL_MAX;
        maxXForce = DBL_MAX;
        maxYForce = DBL_MAX;
        maxZForce = DBL_MAX;
        executionRunning = false;

        rollbackTime = 1.0;

        this->isCartesian = traj->isCartesian();
        this->controlQueue = execQueue;

        this->dmp = traj;
        this->ac = 0.0;
        this->vecYs = arma::vec(1);

        this->dmpCoeffs = traj->getDmpCoeffs();
        this->baseDef = traj->getDmpBase();

        this->tau = traj->getTau(); this->az = traj->getAz(); this->bz = traj->getBz(); this->ax = traj->getAx(); this->gs = traj->getG();
        this->y0s = this->currentJoints = traj->getY0(); this->dy0s = traj->getDy0(); this->ddy0s = traj->getDdy0();
        this->trajGen = new DMPTrajectoryGenerator(this->baseDef, ax, tau);

        this->axDivTau = ax / tau;
        this->oneDivTau = 1 / tau;

        this->simulate = SIMULATE_DMP;
        this->degofFreedom = y0s.n_elem;
        if(isCartesian)
            this->odeSystemSize = 3 * 3 + 1;
        else
            this->odeSystemSize = 2 * this->degofFreedom + 1;
        this->suppressMessages = suppressMessages;

        previousDesiredJoints = y0s;

        externalErrorUsing = 0;
        externalError = 0.0;
        t = 0.0;

        this->odeSystemSizeMinOne = odeSystemSize - 1;

    }

    void  DMPExecutor::setRollbackTime(double rollbackTime) {
        this->rollbackTime = rollbackTime;
    }

    void DMPExecutor::runCheckMaxForces() {

        bool rollBack = false;
        ros::Rate pollingRate(50);

        while(executionRunning) {

            double currentForce = controlQueue->getAbsoluteCartForce();
            mes_result currentFrcTrq = controlQueue->getCurrentCartesianFrcTrq();

            // cout << abs(currentFrcTrq.joints(0)) << " " << abs(currentFrcTrq.joints(1)) << " " << abs(currentFrcTrq.joints(2)) << endl;
            // cout << maxXForce << " " << maxYForce << " " << maxZForce << endl;

            if((currentForce > maxAllowedForce && maxAllowedForce != IGNORE_FORCE)
                    || (abs(currentFrcTrq.joints(0)) > maxXForce && maxXForce != IGNORE_FORCE)
                    || (abs(currentFrcTrq.joints(1)) > maxYForce && maxYForce != IGNORE_FORCE)
                    || (abs(currentFrcTrq.joints(2)) > maxZForce && maxZForce != IGNORE_FORCE)) {
                executionRunning = false;
                rollBack = true;
            }

            pollingRate.sleep();

        }

        while(!executionStoppingDone) {
            pollingRate.sleep();
        }

        // if maxforce event detected --> kill execution and roll back
        if(rollBack && doRollback) {

            cout << "(DMPExecutor) max force threshold exceeded - rolling back a bit and stopping execution" << endl;
            controlQueue->rollBack(rollbackTime);
            cout << "(DMPExecutor) rollback done" << endl;

        }

    }

    void DMPExecutor::setTrajectory(KUKADU_SHARED_PTR<Trajectory> traj) {

        KUKADU_SHARED_PTR<Dmp> dmp = KUKADU_DYNAMIC_POINTER_CAST<Dmp>(traj);
        construct(dmp, controlQueue, suppressMessages);

        vec_t.clear();
        vec_y.clear();

    }

    void DMPExecutor::useExternalError(int external) {
        externalErrorUsing = external;
    }

    void DMPExecutor::setExternalError(double error) {
        externalError = error;
    }

    int DMPExecutor::usesExternalError() {
        return externalErrorUsing;
    }

    double DMPExecutor::getExternalError() {
        return externalError;
    }

    double DMPExecutor::addTerm(double t, const double* currentDesiredYs, int jointNumber, KUKADU_SHARED_PTR<ControlQueue> queue) {
        return 0.0;
    }

    int DMPExecutor::func(double t, const double* y, double* f, void* params) {

        if(!isCartesian) {

            // TODO: remove equation for z' and merge the first two equations
            // y' = z / tau
            // z' = 1 / tau * ( az * (bz * (g - y) - z) + f);
            // x' = -ax / tau * x

            for(int i = 0; i < odeSystemSizeMinOne; i = i + 2) {

                double yPlusOne = y[i + 1];
                int currentSystem = (int) (i / 2);
                f[i] = yPlusOne * oneDivTau;
                double g = gs(currentSystem);
                arma::vec currentCoeffs = dmpCoeffs.at(currentSystem);

                if(t <= (dmp->getTmax() - 1)) {
                    double addTerm = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
                    double x = this->addTerm(t, y, currentSystem, controlQueue);
                    if(!std::isnan(x))
                        f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm) + x; // + this->addTerm(t, y, currentSystem, controlQueue);
                    else
                        f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm);
                } else {
                    f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne));
                }

            }

        } else {

            vec vecF0(3);
            vec vecExtAdd(3);

            for(int i = 0, dim = 0; i < odeSystemSizeMinOne; i = i + 3, ++dim) {

                arma::vec currentCoeffs = dmpCoeffs.at(dim + 3);
                vecF0(dim) = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
                vecExtAdd(dim) =  this->addTerm(t, y, dim + 3, controlQueue);

            }

            if(t <= (dmp->getTmax() - 1))
                nextDEta = oneDivTau * (az * (2.0 * bz * log(qG * currentQ.inverse()) - currentEta) + vecF0);
            else
                nextDEta = oneDivTau * az * (2.0 * bz * log(qG * currentQ.inverse()) - currentEta);

            // cartesian position and orientation
            for(int i = 0; i < odeSystemSizeMinOne; i = i + 3) {

                double yPlusOne = y[i + 1];
                int currentSystem = (int) (i / 3);

                f[i] = yPlusOne * oneDivTau;
                double g = gs(currentSystem);
                arma::vec currentCoeffs = dmpCoeffs.at(currentSystem);

                if(t <= (dmp->getTmax() - 1)) {

                    double addTerm = trajGen->evaluateByCoefficientsSingleNonExponential(y[odeSystemSizeMinOne], currentCoeffs);
                    double x = this->addTerm(t, y, currentSystem, controlQueue);
                    if(!std::isnan(x))
                        f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm) + x; // + this->addTerm(t, y, currentSystem, controlQueue);
                    else
                        f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne) + addTerm);
                } else {
                    f[i + 1] = oneDivTau * (az * (bz * (g - y[i]) - yPlusOne));
                }

                f[i + 2] = nextDEta(currentSystem);

            }

        }

        if(this->simulate == EXECUTE_ROBOT) {

            if(!isCartesian)
                currentJoints = controlQueue->getCurrentJoints().joints;
            else {
                geometry_msgs::Pose currentPose = controlQueue->getCurrentCartesianPose();
                currentJoints(0) = currentPose.position.x;
                currentJoints(1) = currentPose.position.y;
                currentJoints(2) = currentPose.position.x;
                arma::vec currentOrient = log(tf::Quaternion(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w));
                for (int i = 0; i < 3; i++) currentJoints(i + 3) = currentOrient(i);

            }
            double corrector = 0.0;
            if(!usesExternalError()) {

                // include original  phase stopping
                for(int i = 0; i < degofFreedom; ++i) previousDesiredJoints[i] =  y[2 * i];
                double dist = computeDistance(previousDesiredJoints, currentJoints);

                corrector = 1.0 + ac * dist;

            } else {
                corrector = 1.0 + ac * getExternalError();
            }

            f[odeSystemSizeMinOne] = -axDivTau * y[odeSystemSizeMinOne] / corrector;

        } else if(this->simulate == SIMULATE_DMP) {

            // progress as usual
            f[odeSystemSizeMinOne] = - axDivTau * y[odeSystemSizeMinOne];

        }

        return GSL_SUCCESS;

    }

    double DMPExecutor::computeDistance(const arma::vec yDes, arma::vec yCurr) {

        if(!isCartesian) {
            // distance for quaternion has to be introduced if this is enabled
            arma::vec tmp = (yDes - yCurr).t() * (yDes - yCurr);
            return tmp(0);
        }
        else
            return 0;

    }

    int DMPExecutor::jac(double t, const double* y, double *dfdy, double* dfdt, void* params) {

        // not implemented (not required for most of the ode solvers)
        return GSL_SUCCESS;

    }

    KUKADU_SHARED_PTR<ControllerResult> DMPExecutor::executeTrajectory(double ac, double tStart, double tEnd, double tolAbsErr, double tolRelErr) {

        this->ac = ac;
        this->simulate = EXECUTE_ROBOT;
        return this->executeDMP(tStart, tEnd, tolAbsErr, tolRelErr);

    }

    KUKADU_SHARED_PTR<ControllerResult> DMPExecutor::simulateTrajectory(double tStart, double tEnd, double tolAbsErr, double tolRelErr) {

        this->simulate = SIMULATE_DMP;
        KUKADU_SHARED_PTR<ControllerResult> ret = this->executeDMP(tStart, tEnd, tolAbsErr, tolRelErr);
        return ret;

    }

    void DMPExecutor::doRollBackOnMaxForceEvent(bool doRollback) {
        this->doRollback = doRollback;
        cout << "(DMPExecutor) doRollback was set to " << this->doRollback << endl;
    }

    void DMPExecutor::enableMaxForceMode(double maxAbsForce, double maxXForce, double maxYForce, double maxZForce) {
        if(maxAbsForce == IGNORE_FORCE)
            maxAllowedForce = DBL_MAX;
        else
            maxAllowedForce = maxAbsForce;

        if(maxXForce == IGNORE_FORCE)
            maxXForce = DBL_MAX;
        else
            this->maxXForce = maxXForce;

        if(maxYForce == IGNORE_FORCE)
            maxYForce = DBL_MAX;
        else
            this->maxYForce = maxYForce;

        if(maxZForce == IGNORE_FORCE)
            maxZForce = DBL_MAX;
        else
            this->maxZForce = maxZForce;

    }

    KUKADU_SHARED_PTR<ControllerResult> DMPExecutor::simulateTrajectory() {

        this->simulate = SIMULATE_DMP;
        return this->executeDMP(0, dmp->getTmax(), dmp->getTolAbsErr(), dmp->getTolRelErr());

    }

    KUKADU_SHARED_PTR<ControllerResult> DMPExecutor::executeTrajectory() {

        this->simulate = EXECUTE_ROBOT;
        return this->executeDMP(0, dmp->getTmax(), dmp->getTolAbsErr(), dmp->getTolRelErr());

    }

    void DMPExecutor::initializeIntegration(double tStart, double tolAbsErr, double tolRelErr) {

        t = tStart;

        initializeIntegrationQuat();

        vec_y.clear();
        double ys[odeSystemSize];
        vecYs = vec(odeSystemSize);

        if(!isCartesian) {
            for(int i = 0; i < (odeSystemSize - 1); i = i + 2) {
                int iHalf = (int) i / 2;
                ys[i + 0] = y0s((int) iHalf);
                ys[i + 1] = tau * dy0s((int) iHalf);
            }
        } else {
            for(int i = 0, dim = 0; i < (odeSystemSize - 3); i = i + 3, ++dim) {

                ys[i + 0] = y0s(dim);
                ys[i + 1] = tau * dy0s(dim);
                ys[i + 2] = Eta0(dim);

            }
        }

        ys[odeSystemSize - 1] = 1;

        gsl_odeiv2_system tmp_sys = {static_func, NULL, odeSystemSize, this};
        sys = tmp_sys;
        d = KUKADU_SHARED_PTR<gsl_odeiv2_driver>(gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rkf45, controlQueue->getMeasuredTimeStep(), tolAbsErr, tolRelErr), gsl_delete_expression());

        for(int i = 0; i < odeSystemSize; ++i) {
            vecYs(i) = ys[i];
        }
    }

    void DMPExecutor::initializeIntegrationQuat() {

        if(t > 0.0) {
            throw KukaduException("(DMPExecutor) t > 0 is not considered yet with cartesian dmp");
        }

        if(isCartesian) {

            KUKADU_SHARED_PTR<CartesianDMP> cartDmp = KUKADU_DYNAMIC_POINTER_CAST<CartesianDMP>(dmp);
            double firstDt = cartDmp->getDeltaTByIdx(0);
            vec eta0 = cartDmp->getEta0();
            vec eta1 = cartDmp->getEtaByIdx(1);

            tf::Quaternion q0 = cartDmp->getQ0();
            currentQ = q0;
            currentEta = eta0;
            nextEta = eta0;

            //dQ0 = oneDivTau * 0.5 * eta0Quat * q0;

            dEta0 = 1.0 / firstDt * (eta1 - eta0);
            Eta0 = eta0;
            nextDEta = dEta0;

            qG = cartDmp->getQg();
        }

    }

    arma::vec DMPExecutor::doIntegrationStep(double ac) {

        double ys[odeSystemSize];

        for(int i = 0; i < odeSystemSize; ++i)
            ys[i] = vecYs(i);

        arma::vec retJoints(degofFreedom);
        retJoints.fill(0.0);

        if(t < dmp->getTmax()) {

            int s = gsl_odeiv2_driver_apply_fixed_step(d.get(), &t, controlQueue->getMeasuredTimeStep(), 1, ys);


            if (s != GSL_SUCCESS) {
                cout << "(DMPExecutor) error: driver returned " << s << endl;
                throw KukaduException(string("(DMPExecutor) error: driver returned " + s).c_str());
            }

        }

        if(!isCartesian) {

            for(int i = 0; i < degofFreedom; ++i)
                retJoints(i) = ys[2 * i];

        } else {


            for(int i = 0; i < 3; ++i) {
                retJoints(i) = ys[3 * i];
                nextEta(i) = ys[3 * i + 2];
            }


            vec alteredCurrentEta(3);
            alteredCurrentEta = controlQueue->getMeasuredTimeStep() / 2.0 * oneDivTau * nextEta;

            currentQ = exp(alteredCurrentEta) * currentQ;

            retJoints(3) = currentQ.getX(); retJoints(4) = currentQ.getY(); retJoints(5) = currentQ.getZ(); retJoints(6) = currentQ.getW();
            // cout << t << "\t" << currentQ.getX()<<"\t" << currentQ.getY()<<"\t" << currentQ.getZ()<<"\t" << currentQ.getW()<<endl;
            currentEta = nextEta;

        }
        for(int i = 0; i < odeSystemSize; ++i)
            vecYs(i) = ys[i];

        return retJoints;

    }

    void DMPExecutor::destroyIntegration() {

        d = KUKADU_SHARED_PTR<gsl_odeiv2_driver>();

    }

    KUKADU_SHARED_PTR<ControllerResult> DMPExecutor::executeDMP(double tStart, double tEnd, double tolAbsErr, double tolRelErr) {

        // two variables are really required here, because executionRunning is changed by other functions that really need to know
        // whether the exeuction was stopped (this is checked by executionStoppingDone)
        executionRunning = true;
        executionStoppingDone = false;
        if(!isCartesian) {
            maxFrcThread = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&DMPExecutor::runCheckMaxForces, this));
            controlQueue->startJointRollBackMode(3.0);
        }

        double currentTime = 0.0;

        vector<vec> retY;

        vector<double> retT;
        geometry_msgs::Pose start;

        if(!isCartesian) {

            vec currentState = controlQueue->getCurrentJoints().joints;
            for(int i = 0; i < y0s.n_elem; ++i)
                if(abs(currentState(i) - y0s(i)) > 0.01) {
                    controlQueue->jointPtp(y0s);
                    break;
                }

        } else {

            start = controlQueue->getCurrentCartesianPose();
            controlQueue->addCartesianPosToQueue(vectorarma2pose(&y0s));

        }

        initializeIntegration(0, tolAbsErr, tolRelErr);

        vec nextJoints(degofFreedom);
        nextJoints.fill(0.0);

        // execute dmps and compute linear combination
        for(int j = 0; currentTime < tEnd && executionRunning; ++j, currentTime += controlQueue->getMeasuredTimeStep()) {

            try {

                nextJoints = doIntegrationStep(ac);

            } catch(const char* s) {
                cerr << string(s) << ": stopped execution at time " << currentTime << endl;
                break;
            }

            retY.push_back(nextJoints);

            if(simulate == EXECUTE_ROBOT) {
                controlQueue->synchronizeToControlQueue(1);

            }

            if(!isCartesian) {

                controlQueue->addJointsPosToQueue(nextJoints);

            } else {

                geometry_msgs::Pose newP = vectorarma2pose(&nextJoints);
                controlQueue->addCartesianPosToQueue(newP);

            }

            retT.push_back(currentTime);

        }

        executionRunning = false;
        executionStoppingDone = true;

        maxFrcThread->join();

        return KUKADU_SHARED_PTR<ControllerResult>(new ControllerResult(stdToArmadilloVec(retT), retY));

    }

    int DMPExecutor::static_func(double t, const double y[], double f[], void *params) {
        return ((DMPExecutor*)params)->func(t, y, f, NULL);
    }

    int DMPExecutor::static_jac (double t, const double y[], double *dfdy, double dfdt[], void *params) {
        return ((DMPExecutor*)params)->jac(t, y, dfdy, dfdt, NULL);
    }

}
