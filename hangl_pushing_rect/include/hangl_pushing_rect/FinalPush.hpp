#ifndef FINALPUSH_H
#define FINALPUSH_H

#include <memory>
#include <armadillo>
#include <kukadu/kukadu.h>
#include <hangl_pushing_rect/utils.hpp>

namespace pushing {

    class FinalPushController : public kukadu::Controller {

    private:

        double ac;
        double tolAbsErr;
        double tolRelErr;

        arma::vec timesFinalPush;

        std::shared_ptr<kukadu::Dmp> dmpFinalPush;
        std::shared_ptr<kukadu::SensorData> dataFinalPush;
        std::shared_ptr<kukadu::JointDMPLearner> learnerFinalPush;

        std::shared_ptr<kukadu::ControlQueue> leftQueue;

    public:

        FinalPushController(std::shared_ptr<kukadu::ControlQueue> leftQueue, std::string trajectoryPath, double az, double bz, double ac, double tolAbsErr, double tolRelErr);

        virtual void prepare();
        virtual void cleanUp();
        virtual void performCore();

        void performCore(double tmax);

        std::shared_ptr<kukadu::ControllerResult> performAction();

    };

}

#endif // FINALPUSH_H
