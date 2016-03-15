#include <time.h>
#include <iostream>
#include <stdlib.h>
#include <kukadu/kukadu.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "controllers/PeelController.hpp"
#include "controllers/NothingController.hpp"
#include "controllers/FlipVertical.hpp"
#include "controllers/PokeSensingController.hpp"
#include "controllers/SlideSensingController.hpp"
#include "controllers/PressSensingController.hpp"

#include <hangl_pushing_rect/pushingrotcontroller.hpp>
#include <hangl_pushing_rect/pushforwardcontroller.hpp>
#include <hangl_pushing_rect/pushingtranscontroller.hpp>

namespace pf = boost::filesystem;
namespace po = boost::program_options;

using namespace std;
using namespace kukadu;
using namespace pushing;

int hapticMode = 1;

double az = 0.0;
double bz = 0.0;
double tau = 0.0;
double ac = 0.0;
double dmpStepSize = 0.0;
double tolAbsErr = 0.0;
double tolRelErr = 0.0;
double as = 0.0;
double alpham = 0.0;

string environment = "simulation";

int main(int argc, char** args) {

    int usePs = 0;
    int loadPs = 0;
    int useHands = 0;
    int simMode = 1;
    int numberOfActions = 0;
    int numberOfPercepts = 0;

    bool useReal = false;

    double gamma = 0.0;
    double stdReward = 250;

    int numberOfSteps = 1500;
    int numberOfAgents = 10000;
    int numberOfPrepActions = 50;
    int punishReward = -15;
    int stdWeight = 30;

    double boredom = 1.0;
    double senseStretch = 15;

    KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator = KUKADU_SHARED_PTR<kukadu_mersenne_twister>(new kukadu_mersenne_twister(time(NULL)));

    string psFile = "";
    string prepFinalPush = "";
    string prepRotHorPath = "";
    string prepRotVertPath = "";
    string pickTrajectoryPath = "";
    string prepRotHortPushBack = "";
    string prepRotVertPushBack = "";
    string prepTransPushForward = "";

    string tmpPath = "";
    string scriptPath = "";
    string scriptFile = "";
    string databasePath = "";
    string scriptFunction = "";
    string experimentBase = "/home/c7031109/tmp/experiment_simulator/";

    double peelSimFailingProb = 0.0;

    KUKADU_SHARED_PTR<ManualReward> manualRew;
    KUKADU_SHARED_PTR<ProjectiveSimulator> projSim;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("dmp.tau", po::value<double>(), "tau")
            ("dmp.az", po::value<double>(), "az")
            ("dmp.bz", po::value<double>(), "bz")
            ("dmp.dmpStepSize", po::value<double>(), "dmp time step size")
            ("dmp.tolAbsErr", po::value<double>(), "tolerated absolute error")
            ("dmp.tolRelErr", po::value<double>(), "tolerated relative error")
            ("dmp.ac", po::value<double>(), "ac")
            ("pick.trajectory", po::value<string>(), "pick up trajectory")
            ("prep.rothor90deg", po::value<string>(), "horizontal cw rotation")
            ("prep.rotvert90deg", po::value<string>(), "vertical cw rotation")
            ("prep.rothor90degpushback", po::value<string>(), "translation push for horizontal rotation")
            ("prep.rotvert90degpushback", po::value<string>(), "translation push for vertical rotation")
            ("prep.transfinalpush", po::value<string>(), "final push")
            ("prep.transpushforward", po::value<string>(), "first push forward")
            ("ps.useprojectivesimulation", po::value<int>(), "use projective simulation or fixed mapping")
            ("ps.loadps", po::value<int>(), "load pre-trained projective simulator")
            ("ps.numberofactions", po::value<int>(), "number of actions")
            ("ps.numberofpercepts", po::value<int>(), "number of percepts")
            ("ps.gamma", po::value<double>(), "gamma of ps")
            ("ps.stdreward", po::value<double>(), "standard reward for successful execution")
            ("ps.psfile", po::value<string>(), "path to store and load the ps model")
            ("ps.boredom", po::value<double>(), "boredom probability")
            ("ps.punishreward", po::value<double>(), "reward for punishment")
            ("ps.stdweight", po::value<double>(), "initial connection weight")
            ("ps.sensestretch", po::value<double>(), "sensing stretch factor")
            ("control.simulation", po::value<int>(), "use simulator")
            ("control.usehands", po::value<int>(), "use hands")
            ("control.hapticmode", po::value<int>(), "mode for determining the haptic category")
            ("class.database", po::value<string>(), "classificatin database path")
            ("class.mmrpath", po::value<string>(), "classifier path")
            ("class.mmrfile", po::value<string>(), "classifier file")
            ("class.mmrfunction", po::value<string>(), "classifier function")
            ("class.tmppath", po::value<string>(), "temporary folder")
            ("hhm.statisticspath", po::value<string>(), "output folder for statistics")
            ("hhm.peelSimFailProb", po::value<double>(), "simulated peel error for statistics")
    ;

    ifstream parseFile(resolvePath("/home/c7031109/iis_robot_sw/iis_catkin_ws/src/hangl_2016_iros/cfg/irosstat.prop").c_str(), std::ifstream::in);
    po::variables_map vm;
    po::store(po::parse_config_file(parseFile, desc), vm);
    po::notify(vm);

    if (vm.count("dmp.tau")) tau = vm["dmp.tau"].as<double>();
    else return 1;
    if (vm.count("dmp.az")) az = vm["dmp.az"].as<double>();
    else return 1;
    if (vm.count("dmp.bz")) bz = vm["dmp.bz"].as<double>();
    else return 1;
    if (vm.count("dmp.dmpStepSize")) dmpStepSize = vm["dmp.dmpStepSize"].as<double>();
    else return 1;
    if (vm.count("dmp.tolAbsErr")) tolAbsErr = vm["dmp.tolAbsErr"].as<double>();
    else return 1;
    if (vm.count("dmp.tolRelErr")) tolRelErr = vm["dmp.tolRelErr"].as<double>();
    else return 1;
    if (vm.count("dmp.ac")) ac = vm["dmp.ac"].as<double>();
    else return 1;
    if (vm.count("pick.trajectory")) pickTrajectoryPath = resolvePath(vm["pick.trajectory"].as<string>());
    else return 1;
    if (vm.count("prep.rothor90deg")) prepRotHorPath = resolvePath(vm["prep.rothor90deg"].as<string>());
    else return 1;
    if (vm.count("prep.rotvert90deg")) prepRotVertPath = resolvePath(vm["prep.rotvert90deg"].as<string>());
    else return 1;
    if (vm.count("prep.rothor90degpushback")) prepRotHortPushBack = resolvePath(vm["prep.rothor90degpushback"].as<string>());
    else return 1;
    if (vm.count("prep.rotvert90degpushback")) prepRotVertPushBack = resolvePath(vm["prep.rotvert90degpushback"].as<string>());
    else return 1;
    if (vm.count("prep.transpushforward")) prepTransPushForward = resolvePath(vm["prep.transpushforward"].as<string>());
    else return 1;
    if (vm.count("prep.transfinalpush")) prepFinalPush = resolvePath(vm["prep.transfinalpush"].as<string>());
    else return 1;
    if (vm.count("control.simulation")) simMode = vm["control.simulation"].as<int>();
    else return 1;
    if (vm.count("control.hapticmode")) hapticMode = vm["control.hapticmode"].as<int>();
    else return 1;
    if (vm.count("control.usehands")) useHands = vm["control.usehands"].as<int>();
    else return 1;
    if (vm.count("ps.useprojectivesimulation")) usePs = vm["ps.useprojectivesimulation"].as<int>();
    else return 1;
    if (vm.count("ps.loadps")) loadPs = vm["ps.loadps"].as<int>();
    else return 1;
    if (vm.count("ps.boredom")) boredom = vm["ps.boredom"].as<double>();
    else return 1;
    if (vm.count("ps.sensestretch")) senseStretch = vm["ps.sensestretch"].as<double>();
    else return 1;
    if (vm.count("ps.punishreward")) punishReward = vm["ps.punishreward"].as<double>();
    else return 1;
    if (vm.count("ps.stdweight")) stdWeight = vm["ps.stdweight"].as<double>();
    else return 1;
    if (vm.count("ps.numberofactions")) numberOfActions = vm["ps.numberofactions"].as<int>();
    else return 1;
    if (vm.count("ps.numberofpercepts")) numberOfPercepts = vm["ps.numberofpercepts"].as<int>();
    else return 1;
    if (vm.count("ps.gamma")) gamma = vm["ps.gamma"].as<double>();
    else return 1;
    if (vm.count("ps.stdreward")) stdReward = vm["ps.stdreward"].as<double>();
    else return 1;
    if (vm.count("ps.psfile")) psFile = resolvePath(vm["ps.psfile"].as<string>());
    else return 1;
    if (vm.count("class.database")) databasePath = resolvePath(vm["class.database"].as<string>());
    else return 1;
    if (vm.count("class.mmrpath")) scriptPath = resolvePath(vm["class.mmrpath"].as<string>());
    else return 1;
    if (vm.count("class.mmrfile")) scriptFile = resolvePath(vm["class.mmrfile"].as<string>());
    else return 1;
    if (vm.count("class.mmrfunction")) scriptFunction = resolvePath(vm["class.mmrfunction"].as<string>());
    else return 1;
    if (vm.count("class.tmppath")) tmpPath = resolvePath(vm["class.tmppath"].as<string>());
    else return 1;
    if (vm.count("hhm.statisticspath")) experimentBase = resolvePath(vm["hhm.statisticspath"].as<string>());
    else return 1;
    if (vm.count("hhm.peelSimFailProb")) peelSimFailingProb = vm["hhm.peelSimFailProb"].as<double>();
    else return 1;

    environment = (simMode == 0) ? "real" : "simulation";
    int kukaStepWaitTime = dmpStepSize * 1e6;

    string storePsPath = experimentBase + "psserial/";

    if(fileExists(experimentBase))
        deleteDirectory(experimentBase);

    createDirectory(experimentBase);
    createDirectory(storePsPath);

    storePsPath += "iteration";

    cout << "all properties loaded" << endl;
    cout << "execution mode is " << environment << endl;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); sleep(1);
    ros::AsyncSpinner spinner(10);
    spinner.start();

    vector<string> labels;
    labels.push_back("A1"); labels.push_back("A2"); labels.push_back("E1"); labels.push_back("A3"); labels.push_back("A4"); labels.push_back("A5"); labels.push_back("A6");
    KUKADU_SHARED_PTR<ControlQueue> realLeftQueue = KUKADU_SHARED_PTR<PlottingControlQueue>(new PlottingControlQueue(labels, kukaStepWaitTime));
    KUKADU_SHARED_PTR<ControlQueue> realRightQueue = KUKADU_SHARED_PTR<PlottingControlQueue>(new PlottingControlQueue(labels, kukaStepWaitTime));

    KUKADU_SHARED_PTR<ControlQueue> simulationLeftQueue = realLeftQueue;
    KUKADU_SHARED_PTR<ControlQueue> simulationRightQueue = realRightQueue;

    KUKADU_SHARED_PTR<RosSchunk> realLeftHand = KUKADU_SHARED_PTR<PlottingHand>(new PlottingHand(environment, "left"));
    KUKADU_SHARED_PTR<RosSchunk> realRightHand = KUKADU_SHARED_PTR<PlottingHand>(new PlottingHand(environment, "right"));

    KUKADU_SHARED_PTR<RosSchunk> simulationLeftHand = realLeftHand;
    KUKADU_SHARED_PTR<RosSchunk> simulationRightHand = realRightHand;

    realLeftQueue->stopCurrentMode();
    realLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);

    realRightQueue->stopCurrentMode();
    realRightQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);

    KUKADU_SHARED_PTR<kukadu_thread> lqThread = realLeftQueue->startQueueThread();
    KUKADU_SHARED_PTR<kukadu_thread> rqThread = realRightQueue->startQueueThread();

    std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues;
    queues.push_back(realLeftQueue); queues.push_back(realRightQueue);
    std::vector<KUKADU_SHARED_PTR<RosSchunk> > hands;
    hands.push_back(realLeftHand); hands.push_back(realRightHand);
    std::vector<KUKADU_SHARED_PTR<GenericHand> > genericHands;
    genericHands.push_back(realLeftHand); genericHands.push_back(realRightHand);

    try {

        KUKADU_SHARED_PTR<PushForwardController> pushForward = KUKADU_SHARED_PTR<PushForwardController>(new pushing::PushForwardController(realRightQueue, realRightHand,
                                                                                                                                           prepTransPushForward, az, bz, ac,
                                                                                                                                           tolAbsErr, tolRelErr, true));

        KUKADU_SHARED_PTR<KomoPlanner> leftSimKomoPlanner = KUKADU_SHARED_PTR<KomoPlanner>(new KomoPlanner(simulationLeftQueue, resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"), "left"));

        KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushTranslationController = KUKADU_SHARED_PTR<pushing::PushingTranslationController>(new pushing::PushingTranslationController(
                                                                                                    stdToArmadilloVec(createJointsVector(2, 0.2, 0.7)),
                                                                                                    simulationLeftQueue, realLeftQueue,
                                                                                                    simulationLeftHand, realLeftHand,
                                                                                                    leftSimKomoPlanner,
                                                                                                    useReal, pushForward, *node
                                                                                                    )
                                                                                                );

        KUKADU_SHARED_PTR<FinalPushController> finPush = KUKADU_SHARED_PTR<FinalPushController>(new FinalPushController(simulationLeftQueue, prepFinalPush, az, bz, ac, tolAbsErr, tolRelErr));

        KUKADU_SHARED_PTR<Controller> push90Deg = KUKADU_SHARED_PTR<Controller>(new pushing::PushingRotationController(M_PI / 2.0, simulationLeftQueue, realLeftQueue,
                                                                                                                       simulationLeftHand, realLeftHand,
                                                                                                                       leftSimKomoPlanner, useReal, pushForward, pushTranslationController, finPush, *node));

        KUKADU_SHARED_PTR<Controller> push180Deg = KUKADU_SHARED_PTR<Controller>(new pushing::PushingRotationController(M_PI, simulationLeftQueue, realLeftQueue,
                                                                                                                       simulationLeftHand, realLeftHand,
                                                                                                                       leftSimKomoPlanner, useReal, pushForward, pushTranslationController, finPush, *node));

        KUKADU_SHARED_PTR<Controller> push270Deg = KUKADU_SHARED_PTR<Controller>(new pushing::PushingRotationController(3.0 * M_PI / 2.0, simulationLeftQueue, realLeftQueue,
                                                                                                                       simulationLeftHand, realLeftHand,
                                                                                                                       leftSimKomoPlanner, useReal, pushForward, pushTranslationController, finPush, *node));

        KUKADU_SHARED_PTR<Controller> flipVert = KUKADU_SHARED_PTR<Controller>(new FlipVertical(*node,
                                                                                                push90Deg,
                                                                                                finPush, pushForward, pushTranslationController,
                                                                                                leftSimKomoPlanner, simulationLeftQueue, simulationLeftHand,
                                                                                                realLeftQueue, realLeftHand,
                                                                                                realRightQueue,
                                                                                                realRightHand));

        KUKADU_SHARED_PTR<Controller> nothingCont = KUKADU_SHARED_PTR<Controller>(new NothingController());

        KUKADU_SHARED_PTR<SensingController> pokeCont = KUKADU_SHARED_PTR<SensingController>(new PokeSensingController(generator, hapticMode, "/home/c7031109/tmp/pokeDatabase/",
                                                                                                                       realLeftQueue, realRightQueue, realLeftHand, realRightHand,
                                                                                                                       pushTranslationController,
                                                                                                                       tmpPath, scriptPath, scriptFile, scriptFunction,
                                                                                                                       finPush, push90Deg));

        KUKADU_SHARED_PTR<SensingController> slideCont = KUKADU_SHARED_PTR<SensingController>(new SlideSensingController(generator, hapticMode, "/home/c7031109/tmp/slideDatabase/",
                                                                                                                         queues, genericHands, pushTranslationController,
                                                                                                                         tmpPath, scriptPath, scriptFile, scriptFunction,
                                                                                                                         pushForward,
                                                                                                                         finPush, push90Deg));

        KUKADU_SHARED_PTR<SensingController> pressCont = KUKADU_SHARED_PTR<SensingController>(new PressSensingController(generator, hapticMode, "/home/c7031109/tmp/pressDatabase/",
                                                                                                                         realLeftQueue, realRightQueue, realLeftHand, realRightHand, tmpPath,
                                                                                                                         scriptPath, scriptFile, scriptFunction, push90Deg,
                                                                                                                         finPush));

        string rewardsFile = experimentBase + "peelRewards";
        string confFile = experimentBase + "prop";
        string peelPsFile = experimentBase + "peelPS";
        string statisticsFile = experimentBase + "statisticsPrepActionsNum2";

        cout << "writing statistics to file " << statisticsFile << endl;

        ofstream confFileStream;
        confFileStream.open(confFile.c_str());
        confFileStream << "NA" << numberOfAgents << endl;
        confFileStream << "REW" << stdReward << endl;
        confFileStream << "PUN" << punishReward << endl;
        confFileStream << "STDWEIGHT" << stdWeight << endl;
        confFileStream << "DAMP" << gamma << endl;
        confFileStream << "STRETCH" << senseStretch << endl;
        confFileStream << "BOREDOM" << boredom << endl;
        confFileStream << "AGENTCOUNT" << numberOfAgents << endl;

        //TreeDrawer draw(1600, 1000);

        for(int currPrepNum = 6; currPrepNum <= numberOfPrepActions; ++currPrepNum) {

            stringstream statisticsFiless; statisticsFiless << statisticsFile << "_" << currPrepNum;
            ofstream statisticsFileStream;
            statisticsFileStream.open(statisticsFiless.str().c_str());

            cout << "(icra2016) number of preparation actions: " << currPrepNum << endl;

            vector<double> statistics;
            for(int i = 0; i < numberOfSteps; ++i)
                statistics.push_back(0.0);

            vector<KUKADU_SHARED_PTR<Controller> > usefulPrepActions;
            usefulPrepActions.push_back(push90Deg);
            usefulPrepActions.push_back(push180Deg);
            usefulPrepActions.push_back(push270Deg);
            usefulPrepActions.push_back(flipVert);
            usefulPrepActions.push_back(nothingCont);

            vector<KUKADU_SHARED_PTR<Controller> > usedPreparationActions(usefulPrepActions);
            vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers; sensingControllers.push_back(slideCont); sensingControllers.push_back(pokeCont); sensingControllers.push_back(pressCont);

            // ad useless controllers
            for(int i = usefulPrepActions.size(); i < currPrepNum; ++i)
                usedPreparationActions.push_back(KUKADU_DYNAMIC_POINTER_CAST<Controller>(pushTranslationController));

            stringstream s5;
            s5 << experimentBase << "prepnum_" << currPrepNum;

            createDirectory(s5.str());

            for(int agentNum = 0; agentNum < numberOfAgents; ++agentNum) {

                cout << "(icra2016) perform agent number " << agentNum << endl;

                stringstream s3;
                s3 << s5.str() << "/psrewards_ag" << agentNum << "_";

                stringstream s4;
                s4 << s5.str() << "/statisticsPrepActions_ag" << agentNum;

                KUKADU_SHARED_PTR<PeelController> cc = KUKADU_SHARED_PTR<PeelController>(new PeelController("peel book",
                                                                                                realLeftQueue, realRightQueue,
                                                                                                realLeftHand, realRightHand,
                                                                                                pickTrajectoryPath,
                                                                                                KUKADU_DYNAMIC_POINTER_CAST<Controller>(pushTranslationController),
                                                                                                sensingControllers,
                                                                                                usedPreparationActions,
                                                                                                s3.str(), s4.str(), false,
                                                                                                senseStretch, boredom, generator, stdReward, punishReward,
                                                                                                gamma, stdWeight, true,
                                                                                                prepTransPushForward, ac, tolAbsErr, tolRelErr, az, bz,
                                                                                                            peelSimFailingProb,
                                                                                                            pushTranslationController));

                cc->initialize();
                cc->setSimulationMode(true);

                int stepNum = 0;
                int trueNumberOfSteps = 0;
                for(; stepNum < numberOfSteps; ++stepNum, ++trueNumberOfSteps) {

                    //draw.drawTree(cc->getProjectiveSimulator());
                    //draw.waitForEnter();
                //    cout << "(icra2016) performing next attempt" << endl;
                //    cc->storeNextIteration();

                    cc->setBoredom(boredom);
                    cc->setTrainingMode(true);

                    KUKADU_SHARED_PTR<ControllerResult> ret = cc->performAction();

                    if(!ret->wasBored()) {

                        /*
                        // switch off boredom and training for evaluation
                        cc->setBoredom(0.0);
                        cc->setTrainingMode(false);

                        double successRate = 0.0;
                        for(int evalStep = 0; evalStep < evalCount; ++evalStep) {
                            KUKADU_SHARED_PTR<ControllerResult> evalRes = cc->performAction();
                            successRate += (evalRes->getSuccess()) ? 1.0 : 0.0;
                        }

                        successRate /= evalCount;
                        statistics.at(stepNum) += successRate;
                        */

                        statistics.at(stepNum) += (ret->getSuccess()) ? 1.0 : 0.0;

                    } else {
                        cout << "was bored" << endl;
                        --stepNum;
                    }

                    if(!ret->wasBored()) {
                        if((stepNum % 100) == 0) {
                            cc->storeNextIteration();
                        }

                        //cout << "(icra2016statistics) stored next iteration (learning iteration " << stepNum << ")" << endl;
                    }

                }

                cout << "(icra2016) true number of steps: " << trueNumberOfSteps << "; number of rollouts: " << stepNum << endl;

            }

            for(int i = 0; i < numberOfSteps; ++i)
                statisticsFileStream << statistics.at(i) / numberOfAgents << endl;

            statisticsFileStream.close();

        }

        realLeftQueue->stopCurrentMode();
        realLeftQueue->setFinish();
        lqThread->join();

        realRightQueue->stopCurrentMode();
        realRightQueue->setFinish();
        rqThread->join();

    } catch(const char* ex) {
        cout << string(ex) << endl;
    }

    return 0;

}
