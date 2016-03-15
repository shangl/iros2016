#include <kukadu/kukadu.h>

#include <time.h>
#include <memory>
#include <iostream>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <Python.h>
#include <kukadu/kukadu.h>

#include "controllers/discretepush.hpp"
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
double boredom = 1.0;

int main(int argc, char** args) {

    int usePs = 0;
    int loadPs = 0;
    int useHands = 0;
    int simMode = 1;
    int numberOfActions = 0;
    int numberOfPercepts = 0;

    double gamma = 0.0;
    double stdReward = 0.0;
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

    string storePsPath = "";
    string storeRewardHistoryPath = "";

    double peelSimFailingProb = 0.0;

    bool useReal = false;

    if(argc > 1) {
        if(atoi(args[1]))
            useReal = true;
    }

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
            ("prep.transfinalpush", po::value<string>(), "final push")
            ("prep.transpushforward", po::value<string>(), "first push forward")
            ("ps.useprojectivesimulation", po::value<int>(), "use projective simulation or fixed mapping")
            ("ps.loadps", po::value<int>(), "load pre-trained projective simulator")
            ("ps.numberofactions", po::value<int>(), "number of actions")
            ("ps.numberofpercepts", po::value<int>(), "number of percepts")
            ("ps.gamma", po::value<double>(), "gamma of ps")
            ("ps.damping", po::value<double>(), "gamma of ps")
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
            ("hhm.storepspath", po::value<string>(), "path to store ps models")
            ("hhm.storerewardhistorypath", po::value<string>(), "path to store reward history")
            ("hhm.peelSimFailProb", po::value<string>(), "propbability of grasping to fail")
    ;

    ifstream parseFile(resolvePath("/home/c7031109/iis_robot_sw/iis_catkin_ws/src/hangl_2016_iros/cfg/haptic.prop").c_str(), std::ifstream::in);
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
    if (vm.count("hhm.storepspath")) storePsPath = resolvePath(vm["hhm.storepspath"].as<string>());
    else return 1;
    if (vm.count("hhm.storerewardhistorypath")) storeRewardHistoryPath = resolvePath(vm["hhm.storerewardhistorypath"].as<string>());
    else return 1;

    cout << "all properties loaded" << endl;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); sleep(1);
    ros::AsyncSpinner spinner(10);
    spinner.start();

    if(fileExists(storePsPath)) {
        char answer;
        cout << "path to trained ecm already exists. do you want to replace it? (yes = y)" << endl;
        cin >> answer;
        if(answer != 'y') {
            cerr << "aborting" << endl;
            return EXIT_FAILURE;
        }

    }
    deleteDirectory(storePsPath);
    createDirectory(storePsPath);

    if(!fileExists(databasePath))
        createDirectory(databasePath);

    cout << "creating simulation controllers" << endl;
    KUKADU_SHARED_PTR<KukieControlQueue> simulationLeftQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue(dmpStepSize, "simulation", "left_arm", *node));;
    KUKADU_SHARED_PTR<KukieControlQueue> simulationRightQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue(dmpStepSize, "simulation", "right_arm", *node, true));;

    KUKADU_SHARED_PTR<KukieControlQueue> realLeftQueue;
    KUKADU_SHARED_PTR<KukieControlQueue> realRightQueue;
    if(useReal) {
        cout << "creating real controllers" << endl;
        realLeftQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue(dmpStepSize, "real", "left_arm", *node));
        realRightQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue(dmpStepSize, "real", "right_arm", *node, true));
    }

    KUKADU_SHARED_PTR<RosSchunk> realLeftHand;
    KUKADU_SHARED_PTR<RosSchunk> realRightHand;
    KUKADU_SHARED_PTR<RosSchunk> simulationLeftHand;
    KUKADU_SHARED_PTR<RosSchunk> simulationRightHand;
    if(useHands) {
        cout << "creating simulation hand controllers" << endl;
        simulationLeftHand = KUKADU_SHARED_PTR<RosSchunk>(new RosSchunk(*node, "simulation", "left"));
        simulationRightHand = KUKADU_SHARED_PTR<RosSchunk>(new RosSchunk(*node, "simulation", "right"));
        if(useReal) {
            cout << "creating real hand controllers" << endl;
            realLeftHand = KUKADU_SHARED_PTR<RosSchunk>(new RosSchunk(*node, "real", "left"));
            realRightHand = KUKADU_SHARED_PTR<RosSchunk>(new RosSchunk(*node, "real", "right"));
        }
    } else {
        cout << "creating plotting hand controllers" << endl;
        simulationLeftHand = KUKADU_SHARED_PTR<PlottingHand>(new PlottingHand("real", "left"));
        simulationRightHand = KUKADU_SHARED_PTR<PlottingHand>(new PlottingHand("real", "right"));
        realLeftHand = KUKADU_SHARED_PTR<PlottingHand>(new PlottingHand("real", "left"));
        realRightHand = KUKADU_SHARED_PTR<PlottingHand>(new PlottingHand("real", "right"));
    }

    KUKADU_SHARED_PTR<kukadu_thread> realLqThread;
    KUKADU_SHARED_PTR<kukadu_thread> realRqThread;
    if(useReal) {
        cout << "starting real queue threads" << endl;
        setStandardStiffness(realLeftQueue);
        setStandardStiffness(realRightQueue);

        if(realLeftQueue->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
            realLeftQueue->stopCurrentMode();
            realLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
        }

        if(realRightQueue->getCurrentControlType() != KukieControlQueue::KUKA_JNT_IMP_MODE) {
            realRightQueue->stopCurrentMode();
            realRightQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
        }

        realLqThread = realLeftQueue->startQueueThread();
        realRqThread = realRightQueue->startQueueThread();
    }

    cout << "switching control modes in simulation" << endl;
    simulationLeftQueue->stopCurrentMode();
    simulationLeftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    simulationRightQueue->stopCurrentMode();
    simulationRightQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    KUKADU_SHARED_PTR<kukadu_thread> simulationLqThread = simulationLeftQueue->startQueueThread();
    KUKADU_SHARED_PTR<kukadu_thread> simulationRqThread = simulationRightQueue->startQueueThread();

    cout << "setting up main control hand" << endl;
    KUKADU_SHARED_PTR<RosSchunk> controllerLeftHand = realLeftHand;
    KUKADU_SHARED_PTR<RosSchunk> controllerRightHand = realRightHand;
    KUKADU_SHARED_PTR<KukieControlQueue> controllerLeftQueue = realLeftQueue;
    KUKADU_SHARED_PTR<KukieControlQueue> controllerRightQueue = realRightQueue;
    if(!useReal) {
        controllerLeftQueue = simulationLeftQueue;
        controllerRightQueue = simulationRightQueue;

        controllerLeftHand = simulationLeftHand;
        controllerRightHand = simulationRightHand;
    }

    std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues;
    queues.push_back(controllerLeftQueue); queues.push_back(controllerRightQueue);
    std::vector<KUKADU_SHARED_PTR<RosSchunk> > hands;
    hands.push_back(controllerLeftHand); hands.push_back(controllerRightHand);
    std::vector<KUKADU_SHARED_PTR<GenericHand> > genericHands;
    genericHands.push_back(controllerLeftHand); genericHands.push_back(controllerRightHand);

    cout << "creating komo planner for left arm" << endl;
    KUKADU_SHARED_PTR<KomoPlanner> leftSimKomoPlanner = KUKADU_SHARED_PTR<KomoPlanner>(new KomoPlanner(simulationLeftQueue, resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"), "left"));

    KUKADU_SHARED_PTR<PushForwardController> pushForward = KUKADU_SHARED_PTR<PushForwardController>(new pushing::PushForwardController(controllerRightQueue, controllerRightHand,
                                                                                                                                       prepTransPushForward, az, bz, ac,
                                                                                                                                       tolAbsErr, tolRelErr, true));

    KUKADU_SHARED_PTR<FinalPushController> finPush = KUKADU_SHARED_PTR<FinalPushController>(new FinalPushController(controllerLeftQueue, prepFinalPush, az, bz, ac, tolAbsErr, tolRelErr));
    cout << "creating push translation controller" << endl;
    KUKADU_SHARED_PTR<pushing::PushingTranslationController> pushTranslationController = KUKADU_SHARED_PTR<pushing::PushingTranslationController>(new pushing::PushingTranslationController(
                                                                                                stdToArmadilloVec(createJointsVector(2, 0.18, 0.7)),
                                                                                                simulationLeftQueue, realLeftQueue,
                                                                                                simulationLeftHand, realLeftHand,
                                                                                                leftSimKomoPlanner,
                                                                                                useReal, KUKADU_SHARED_PTR<PushForwardController>(), *node
                                                                                                )
                                                                                            );

    cout << "creating push 90 deg controller" << endl;
    KUKADU_SHARED_PTR<Controller> push90Deg = KUKADU_SHARED_PTR<Controller>(new pushing::PushingRotationController(M_PI / 2.0 + 0.3, simulationLeftQueue, realLeftQueue,
                                                                                                                   simulationLeftHand, realLeftHand,
                                                                                                                   leftSimKomoPlanner, useReal,
                                                                                                                   pushForward, pushTranslationController, finPush, *node));

    cout << "creating push 180 deg controller" << endl;
    KUKADU_SHARED_PTR<Controller> push180Deg = KUKADU_SHARED_PTR<Controller>(new DiscretePush(push90Deg, 2));

    cout << "creating push 270 deg controller" << endl;
    KUKADU_SHARED_PTR<Controller> push270Deg = KUKADU_SHARED_PTR<Controller>(new DiscretePush(push90Deg, 3));

    cout << "creating flip vertical controller" << endl;
    KUKADU_SHARED_PTR<Controller> flipVert = KUKADU_SHARED_PTR<Controller>(new FlipVertical(*node,
                                                                                            push90Deg,
                                                                                            finPush, pushForward, pushTranslationController,
                                                                                            leftSimKomoPlanner, simulationLeftQueue, simulationLeftHand,
                                                                                            realLeftQueue, realLeftHand,
                                                                                            realRightQueue,
                                                                                            realRightHand));

    cout << "creating nothing controller" << endl;
    KUKADU_SHARED_PTR<Controller> nothingCont = KUKADU_SHARED_PTR<Controller>(new NothingController());

    HapticPlanner planner;
    planner.addPreparationController("push 90 degrees", push90Deg);
    planner.addPreparationController("push 180 degrees", push180Deg);
    planner.addPreparationController("push 270 degrees", push270Deg);
    planner.addPreparationController("flip vertically", flipVert);
    planner.addPreparationController("do nothing", nothingCont);

    cout << "creating poke controller" << endl;
    KUKADU_SHARED_PTR<SensingController> pokeCont = KUKADU_SHARED_PTR<SensingController>(new PokeSensingController(generator, hapticMode, databasePath + "/poke_database/",
                                                                                                                   controllerLeftQueue, controllerRightQueue, controllerLeftHand, controllerRightHand,
                                                                                                                   pushTranslationController,
                                                                                                                   tmpPath, scriptPath, scriptFile, scriptFunction,
                                                                                                                   finPush, push90Deg));

    cout << "creating slide controller" << endl;
    KUKADU_SHARED_PTR<SensingController> slideCont = KUKADU_SHARED_PTR<SensingController>(new SlideSensingController(generator, hapticMode, databasePath + "/slide_database/",
                                                                                                                     queues, genericHands, pushTranslationController,
                                                                                                                     tmpPath, scriptPath, scriptFile, scriptFunction,
                                                                                                                     pushForward,
                                                                                                                     finPush,
                                                                                                                     push90Deg));

    cout << "creating press controller" << endl;
    KUKADU_SHARED_PTR<SensingController> pressCont = KUKADU_SHARED_PTR<SensingController>(new PressSensingController(generator, hapticMode, databasePath + "/press_database/",
                                                                                                                     controllerLeftQueue, controllerRightQueue,
                                                                                                                     controllerLeftHand, controllerRightHand, tmpPath,
                                                                                                                     scriptPath, scriptFile, scriptFunction, push90Deg,
                                                                                                                     finPush));

    planner.addSensingController("sliding", slideCont);
    planner.addSensingController("poking", pokeCont);
    planner.addSensingController("pressing", pressCont);

    vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers; sensingControllers.push_back(slideCont); sensingControllers.push_back(pokeCont); sensingControllers.push_back(pressCont);
    vector<KUKADU_SHARED_PTR<Controller> > prepControllers; prepControllers.push_back(nothingCont); prepControllers.push_back(push90Deg); prepControllers.push_back(push180Deg); prepControllers.push_back(push270Deg); prepControllers.push_back(flipVert);

    cout << "start moving stuff (please pay attention to the robot)" << endl;
    goToRightHandBlocking(controllerRightHand);
    goToPushHandPos(controllerLeftHand);
    goToStartPos(controllerLeftQueue);

    // uncomment this to bring the robot to the first position
    //simpleJointPtp(controllerRightQueue, stdToArmadilloVec(createJointsVector(7, -1.3800294399261475, 0.5, -2.280046224594116, 1.884490966796875, 2.1091063022613525, -0.4556314945220947, -0.7266652584075928)));
    goToBlockingPos(controllerRightQueue);

    controllerLeftHand->setGrasp(eGID_CYLINDRICAL);
    controllerLeftHand->closeHand(1.0, 1.0);

    // create complex controller here
    KUKADU_SHARED_PTR<PeelController> cc = KUKADU_SHARED_PTR<PeelController>(new PeelController("peel book",
                                                                            controllerLeftQueue, controllerRightQueue,
                                                                            controllerLeftHand, controllerRightHand,
                                                                            pickTrajectoryPath,
                                                                            KUKADU_DYNAMIC_POINTER_CAST<Controller>(pushTranslationController),
                                                                            sensingControllers,
                                                                            prepControllers,
                                                                            storePsPath + string("ecm"), storeRewardHistoryPath, false, senseStretch, boredom,
                                                                            generator, stdReward, punishReward, gamma, stdWeight, true,
                                                                            prepTransPushForward, ac, tolAbsErr, tolRelErr, az, bz,
                                                                                                peelSimFailingProb,
                                                                                                pushTranslationController));

    cc->initialize();
    cc->setTrainingMode(true);

    int stepNum = 0;
    while(true) {

        KUKADU_SHARED_PTR<ControllerResult> ret = cc->performAction();
        cc->storeNextIteration();

    }

    if(useReal) {
        realLeftQueue->stopCurrentMode();
        realLeftQueue->setFinish();
        realLqThread->join();

        realRightQueue->stopCurrentMode();
        realRightQueue->setFinish();
        realRqThread->join();
    }

    simulationLeftQueue->stopCurrentMode();
    simulationLeftQueue->setFinish();
    simulationLqThread->join();

    simulationRightQueue->stopCurrentMode();
    simulationRightQueue->setFinish();
    simulationRqThread->join();

    return 0;

}
