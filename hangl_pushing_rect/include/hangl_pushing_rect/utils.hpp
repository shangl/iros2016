#ifndef HANGL_PUSHING_UTILS
#define HANGL_PUSHING_UTILS

#include <vector>
#include <armadillo>
#include <kukadu/kukadu.h>
#include <geometry_msgs/Pose.h>

void ptpWithSimulation(geometry_msgs::Pose pose, KUKADU_SHARED_PTR<kukadu::ControlQueue>  simulationQueue, KUKADU_SHARED_PTR<kukadu::ControlQueue>  executionQueue, bool useReal);
void jointPtpWithSimulation(arma::vec joints, KUKADU_SHARED_PTR<kukadu::ControlQueue>  simulationQueue, KUKADU_SHARED_PTR<kukadu::ControlQueue>  executionQueue, bool useReal);
void moveHome(KUKADU_SHARED_PTR<kukadu::ControlQueue> simQueue, KUKADU_SHARED_PTR<kukadu::ControlQueue> realQueue, bool useReal);

void simpleJointPtp(KUKADU_SHARED_PTR<kukadu::ControlQueue> rightQueue, arma::vec joints, double maxForce = DBL_MAX);

void pushForward(KUKADU_SHARED_PTR<kukadu::ControlQueue> rightQueue, double maxForce);

void setWeakStiffness(KUKADU_SHARED_PTR<kukadu::ControlQueue> queue);
void setStandardStiffness(KUKADU_SHARED_PTR<kukadu::ControlQueue> queue);

void goToStartPos(KUKADU_SHARED_PTR<kukadu::ControlQueue> leftQueue);
void goToBlockingPos(KUKADU_SHARED_PTR<kukadu::ControlQueue> rightQueue, bool waitForConfirmation = true);

void goToPushHandPos(KUKADU_SHARED_PTR<kukadu::GenericHand> leftHand);
void goToRightHandBlocking(KUKADU_SHARED_PTR<kukadu::GenericHand> rightHand);

void goToFlipHandPos(KUKADU_SHARED_PTR<kukadu::GenericHand> leftHand);

void setHardStiffness(KUKADU_SHARED_PTR<kukadu::ControlQueue> queue);

double computeBookOrientation(double angle);

double computeMaxJointDistance(arma::vec joints1, arma::vec joints2);
std::vector<arma::vec> smoothJointPlan(std::vector<arma::vec> jointPlan);

std::vector<KUKADU_SHARED_PTR<kukadu::GenericHand> > createHandList(KUKADU_SHARED_PTR<kukadu::GenericHand> hand1);
std::vector<KUKADU_SHARED_PTR<kukadu::ControlQueue> > createQueueList(KUKADU_SHARED_PTR<kukadu::ControlQueue> queue1, KUKADU_SHARED_PTR<kukadu::ControlQueue> queue2);

#endif
