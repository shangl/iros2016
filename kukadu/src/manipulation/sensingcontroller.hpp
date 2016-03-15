#ifndef KUKADU_SENSINGCONTROLLER_H
#define KUKADU_SENSINGCONTROLLER_H

#include <string>
#include <vector>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <wordexp.h>

#include "controller.hpp"
#include "controllerresult.hpp"
#include "../types/kukadutypes.hpp"
#include "../robot/sensorstorage.hpp"
#include "../robot/kukiecontrolqueue.hpp"

namespace kukadu {

    class SensingController : public Controller {

    private:

        bool classifierParamsSet;

        int hapticMode;
        int currentIterationNum;
        int simulationGroundTruth;
        int simulatedClassificationPrecision;

        KUKADU_DISCRETE_DISTRIBUTION<int> classifierDist;

        double bestParamC;
        double bestParamD;
        double bestParamParam1;
        double bestParamParam2;

        std::string tmpPath;
        std::string databasePath;
        std::string classifierPath;
        std::string classifierFile;
        std::string classifierFunction;

        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;

        std::vector<KUKADU_SHARED_PTR<GenericHand> > hands;
        std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues;

        std::vector<double> callClassifier(std::string trainedPath, std::string passedFilePath, bool classify,
                                           double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2);

        void writeLabelFile(std::string baseFolderPath, std::vector<std::pair<int, std::string> > collectedSamples);

    public:

        SensingController(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, std::string caption, std::string databasePath, std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, std::vector<KUKADU_SHARED_PTR<GenericHand> > hands,
                          std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFunction,
                          int simClassificationPrecision);

        void setSimulationGroundTruth(int idx);
        void gatherData(std::string completePath);
        void setSimulationClassificationPrecision(int percent);
        void gatherData(std::string dataBasePath, std::string dataName);
        void setCLassifierParams(double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2);

        virtual void prepare() = 0;
        virtual void cleanUp() = 0;
        virtual void performCore() = 0;
        virtual void prepareNextState() = 0;

        int performClassification();
        int createRandomGroundTruthIdx();

        virtual int getSensingCatCount() = 0;

        double createDataBase();

        std::string getDatabasePath();
        std::string getFirstRobotFileName();

        std::vector<double> callClassifier();

        KUKADU_SHARED_PTR<ControllerResult> performAction();

        static const int HAPTIC_MODE_TERMINAL = 0;
        static const int HAPTIC_MODE_CLASSIFIER = 1;

    };

}

#endif
