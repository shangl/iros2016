#ifndef KUKADU_DMPEXECUTOR_H
#define KUKADU_DMPEXECUTOR_H

#include <vector>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <armadillo>
#include <ros/rate.h>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

#include "trajectoryexecutor.hpp"

#include "../types/dmp.hpp"
#include "../utils/types.hpp"
#include "../utils/utils.hpp"
#include "../types/dmpbase.hpp"
#include "../types/trajectory.hpp"
#include "../types/kukadutypes.hpp"
#include "../types/cartesiandmp.hpp"
#include "../robot/controlqueue.hpp"
#include "../trajectory/jointdmplearner.hpp"
#include "../trajectory/dmptrajectorygenerator.hpp"

namespace kukadu {

    struct gsl_delete_expression {
        void operator()(gsl_odeiv2_driver* p) const {
            gsl_odeiv2_driver_free(p);
        }
    };

    /** \brief This class is responsible for dmp execution
     *
     * The DMPExecutor computes the evolution of the dynmic movement primitives by using a numerical differential equation solver. It provides execution and simulation mode.
     * If the execution mode is selected, a control queue has to be passed to constructor.
     * \ingroup ControlPolicyFramework
     */
    class DMPExecutor : public TrajectoryExecutor {

    protected:

        bool doRollback;
        bool isCartesian;
        bool executionRunning;
        bool executionStoppingDone;

        int simulate;
        int degofFreedom;
        int suppressMessages;
        int externalErrorUsing;
        int odeSystemSizeMinOne;

        long unsigned int odeSystemSize;

        double t;
        double az;
        double bz;
        double ax;
        double ac;
        double tau;
        // for optimization
        double axDivTau;
        double oneDivTau;
        double externalError;
        double maxAllowedForce;

        double maxXForce;
        double maxYForce;
        double maxZForce;
        double rollbackTime;

        arma::vec gs;
        arma::vec y0s;
        arma::vec dy0s;
        arma::vec Eta0;
        arma::vec ddy0s;
        arma::vec vecYs;
        arma::vec dEta0;
        arma::vec nextEta;
        arma::vec nextDEta;
        arma::vec currentEta;
        arma::vec currentJoints;
        arma::vec previousDesiredJoints;

        std::vector<DMPBase> baseDef;
        std::vector<arma::vec> dmpCoeffs;

        DMPTrajectoryGenerator* trajGen;

        KUKADU_SHARED_PTR<Dmp> dmp;
        KUKADU_SHARED_PTR<gsl_odeiv2_driver> d;
        KUKADU_SHARED_PTR<ControlQueue> controlQueue;
        KUKADU_SHARED_PTR<kukadu_thread> maxFrcThread;

        std::vector<double> vec_t;
        std::vector<double> vec_y;
        std::vector<double> internalClock;

        gsl_odeiv2_system sys;

        tf::Quaternion qG;
        tf::Quaternion dQ0;
        tf::Quaternion nextQ;
        tf::Quaternion currentQ;

        void runCheckMaxForces();

        // needed for workaround (see here http://stackoverflow.com/questions/10687397/static-virtual-workaround-in-gsl)
        static int static_func(double t, const double y[], double f[], void *params);
        static int static_jac (double t, const double y[], double *dfdy, double dfdt[], void *params);

        double computeDistance(const arma::vec yDes, arma::vec yCurr);

        KUKADU_SHARED_PTR<ControllerResult> executeDMP(double tStart, double tEnd, double tolAbsErr, double tolRelErr);

    protected:

        virtual int func (double t, const double* y, double* f, void* params);
        virtual int jac(double t, const double* y, double *dfdy, double* dfdt, void* params);

        virtual double addTerm(double t, const double* currentDesiredYs, int jointNumber, KUKADU_SHARED_PTR<ControlQueue> queue);

    public:

        /**
         * \brief constructor
         * \param dmp the dmp that should be executed
         */
        DMPExecutor(KUKADU_SHARED_PTR<Dmp> dmp, KUKADU_SHARED_PTR<ControlQueue> execQueue);
        DMPExecutor(KUKADU_SHARED_PTR<Trajectory> dmp, KUKADU_SHARED_PTR<ControlQueue> execQueue);
        DMPExecutor(KUKADU_SHARED_PTR<Dmp> dmp, KUKADU_SHARED_PTR<ControlQueue> execQueue, int suppressMessages);

        int usesExternalError();
        void destroyIntegration();
        void initializeIntegrationQuat();
        void useExternalError(int external);
        void setExternalError(double error);
        // for now, only works in joint mode
        void enableMaxForceMode(double maxAbsForce, double maxXForce, double maxYForce, double maxZForce);
        void doRollBackOnMaxForceEvent(bool doRollback);
        void setTrajectory(KUKADU_SHARED_PTR<Trajectory> traj);
        void initializeIntegration(double tStart, double tolAbsErr, double tolRelErr);
        void construct(KUKADU_SHARED_PTR<Dmp> dmp, KUKADU_SHARED_PTR<ControlQueue> execQueue, int suppressMessages);
        void  setRollbackTime(double rollbackTime);

        double getExternalError();

        arma::vec doIntegrationStep(double ac);

        KUKADU_SHARED_PTR<ControllerResult> executeTrajectory();
        KUKADU_SHARED_PTR<ControllerResult> simulateTrajectory();
        KUKADU_SHARED_PTR<ControllerResult> simulateTrajectory(double tStart, double tEnd, double tolAbsErr, double tolRelErr);
        KUKADU_SHARED_PTR<ControllerResult> executeTrajectory(double ac, double tStart, double tEnd, double tolAbsErr, double tolRelErr);

        static const int SIMULATE_DMP = 1;
        static const int EXECUTE_ROBOT = 2;

        static const int KUKADU_EXEC_JOINT = 1;
        static const int KUKADU_EXEC_CART = 2;

        static const int IGNORE_FORCE = -1;

    };

}

#endif
