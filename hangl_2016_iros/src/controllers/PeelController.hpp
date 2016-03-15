#ifndef PEELCONTROLLER_H
#define PEELCONTROLLER_H

#include <armadillo>
#include <kukadu/kukadu.h>
#include <hangl_pushing_rect/utils.hpp>
#include <hangl_pushing_rect/pushforwardcontroller.hpp>
#include <hangl_pushing_rect/pushingtranscontroller.hpp>

class PeelController : public kukadu::ComplexController {

private:

    double ac;
    double tolAbsErr;
    double tolRelErr;
    double dmpStepSize;

    std::string trajectoryPath;

    KUKADU_SHARED_PTR<kukadu::ControlQueue> leftQueue;
    KUKADU_SHARED_PTR<kukadu::ControlQueue> rightQueue;

    KUKADU_SHARED_PTR<kukadu::RosSchunk> leftHand;
    KUKADU_SHARED_PTR<kukadu::RosSchunk> rightHand;

    pushing::PushForwardController pushForward;
    KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushOrigin;

protected:

    virtual double getSimulatedReward(KUKADU_SHARED_PTR<kukadu::SensingController> usedSensingController, KUKADU_SHARED_PTR<kukadu::PerceptClip> providedPercept, KUKADU_SHARED_PTR<kukadu::Controller> takenAction, int sensingClassIdx, int prepContIdx);

public:

    PeelController(std::string caption,
                   KUKADU_SHARED_PTR<kukadu::ControlQueue> leftQueue, KUKADU_SHARED_PTR<kukadu::ControlQueue> rightQueue, KUKADU_SHARED_PTR<kukadu::RosSchunk> leftHand,
                   KUKADU_SHARED_PTR<kukadu::RosSchunk> rightHand,
                   std::string trajectoryPath, KUKADU_SHARED_PTR<kukadu::Controller> pushForward,
                   std::vector<KUKADU_SHARED_PTR<kukadu::SensingController> > sensingControllers, std::vector<KUKADU_SHARED_PTR<Controller> > preparationControllers,
                   std::string corrPSPath, std::string rewardHistoryPath, bool storeReward, double senseStretch, double boredom, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int stdReward, int punishReward, double gamma, int stdPrepWeight, bool collectPrevRewards,
                   std::string pushForwardTrajPath, double ac, double tolAbsErr, double tolRelErr, double az, double bz,
                   double simuFailProb, KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushOrigin);

    virtual void executeComplexAction();

};

#endif // PEELCONTROLLER_H
