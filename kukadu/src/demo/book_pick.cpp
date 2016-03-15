#include "../utils/easyloggingpp/src/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP

#include <iostream>
#include <string>
#include <armadillo>
#include <thread>
#include <vector>
#include <boost/program_options.hpp>

#include "../../include/kukadu.h"

#define ROBOT_TYPE "simulation"
#define ROBOT_SIDE "left"

#define CONTROL_RIGHT true

using namespace std;
using namespace arma;
namespace po = boost::program_options;

int main(int argc, char** args) {

    int importanceSamplingCount;
    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, as, alpham;
    string inDir, cfFile, dataFolder, trajFile;
    vector<double> rlExploreSigmas;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("metric.database", po::value<string>(), "database folder")
            ("metric.goldStandard", po::value<string>(), "gold standard file")
            ("metric.alpham", po::value<double>(), "alpham")
            ("metric.exploreSigmas", po::value<string>(), "reinforcement learning exploration sigmas")
            ("metric.importanceSamplingCount", po::value<int>(), "size of importance sampling vector")
            ("metric.as", po::value<double>(), "as")
            ("dmp.tau", po::value<double>(), "tau")
            ("dmp.az", po::value<double>(), "az")
            ("dmp.bz", po::value<double>(), "bz")
            ("dmp.dmpStepSize", po::value<double>(), "dmp time step size")
            ("dmp.tolAbsErr", po::value<double>(), "tolerated absolute error")
            ("dmp.tolRelErr", po::value<double>(), "tolerated relative error")
            ("dmp.ac", po::value<double>(), "ac")
            ("mes.folder", po::value<string>(), "measurment data folder")
            ("pick.trajectory", po::value<string>(), "data for measured trajectory")
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/book_pick.prop"), std::ifstream::in);
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

    if (vm.count("metric.as")) as = vm["metric.as"].as<double>();
    else return 1;
    if (vm.count("metric.database")) inDir = resolvePath(vm["metric.database"].as<string>());
    else return 1;
    if (vm.count("metric.goldStandard")) cfFile = resolvePath(vm["metric.goldStandard"].as<string>());
    else return 1;
    if (vm.count("metric.alpham")) alpham = vm["metric.alpham"].as<double>();
    else return 1;
    if (vm.count("metric.exploreSigmas")) {
        string tokens = vm["metric.exploreSigmas"].as<string>();
        stringstream parseStream(tokens);
        char bracket;
        double dToken;
        parseStream >> bracket;
        while(bracket != '}') {
            parseStream >> dToken >> bracket;
            rlExploreSigmas.push_back(dToken);
        }
    } else return 1;
    if (vm.count("metric.importanceSamplingCount")) importanceSamplingCount = vm["metric.importanceSamplingCount"].as<int>();
    else return 1;

    if (vm.count("mes.folder")) dataFolder = resolvePath(vm["mes.folder"].as<string>());
    else return 1;

    if (vm.count("pick.trajectory")) trajFile = resolvePath(vm["pick.trajectory"].as<string>());
    else return 1;

    cout << "all properties loaded" << endl;
    int kukaStepWaitTime = dmpStepSize * 1e6;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    cout << "ros connection initialized" << endl;

    /*
    KukieSimulator sim(*node);
    sim.addPrimitiveObject("book", stdToArmadilloVec({0.3, 0.5, 0}), stdToArmadilloVec({0, 0, 0}), 1.0, KS_BOX, stdToArmadilloVec({0.2, 0.3, 0.1}));
    cout << "(main) object created" << endl;
    getchar();
    */

    shared_ptr<RosSchunk> leftHand = shared_ptr<RosSchunk>(new RosSchunk(*node, ROBOT_TYPE, ROBOT_SIDE));
    vector<double> leftHandJoints = {0, -0.38705404571511043, 0.7258474179992682, 0.010410616072391092, -1.2259735578027993, -0.4303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    shared_ptr<RosSchunk> rightHand = shared_ptr<RosSchunk>(new RosSchunk(*node, ROBOT_TYPE, "right"));
    vector<double> rightHandJoints = {0, -0.5238237461313833, 0.2120872918378427, 0.8655742259109377, 1.5389379959387146, -0.6260686922290597, 0.218843743489235877};
    rightHand->publishSdhJoints(rightHandJoints);

    shared_ptr<KukieControlQueue> leftQueue = shared_ptr<KukieControlQueue>(new KukieControlQueue(kukaStepWaitTime, ROBOT_TYPE, ROBOT_SIDE + string("_arm"), *node));
    shared_ptr<thread> lqThread = leftQueue->startQueueThread();

    shared_ptr<KukieControlQueue> rightQueue = nullptr;
    shared_ptr<thread> rqThread = nullptr;

    if(CONTROL_RIGHT) {

        rightQueue = shared_ptr<KukieControlQueue>(new KukieControlQueue(kukaStepWaitTime, ROBOT_TYPE, "right_arm", *node));
        rqThread = rightQueue->startQueueThread();

    }

    usleep(1e6);

    if(CONTROL_RIGHT) {

        rightQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
        rightQueue->moveJoints(stdToArmadilloVec({-2.0805978775024414, 1.4412447214126587, -2.203258752822876, 1.7961543798446655, 2.526369333267212, -1.6385622024536133, -0.7103539705276489}));

    }

    // measured joints were -0.2252   1.3174  -2.1671   0.4912   0.8510  -1.5699   1.0577
    mes_result currentJoints = leftQueue->getCurrentJoints();
    cout << currentJoints.joints.t() << endl;

    cout << "(main) place book on expected position (press enter)" << endl;
    // getchar();

//    leftQueue->setStiffness(0.2, 0.01, 0.2, 15000, 150, 2.0);
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    //leftQueue->moveJoints(stdToArmadilloVec({-0.514099, 1.83194, 1.95971, -0.99676, -0.0903862, 0.987185, 1.16542}));
    leftQueue->moveJoints(stdToArmadilloVec({-0.3990752398967743, 1.7550331354141235, 1.9612526893615723, -0.6003820300102234, -0.05098132789134979, 1.200370192527771, 1.168589472770691}));

    cout << "(main) press key to continue" << endl;
    // getchar();

    shared_ptr<SensorData> dat = SensorStorage::readStorage(leftQueue, trajFile);
    JointDMPLearner learner(az, bz, dat->getRange(0, 8));
    std::shared_ptr<Dmp> leftDmp = learner.fitTrajectories();
    DMPExecutor leftExecutor(leftDmp, leftQueue);
    leftExecutor.executeTrajectory(ac, 0, leftDmp->getTmax(), dmpStepSize, tolAbsErr, tolRelErr);

    // first finger

    cout << "(main) ready to move first finger? (press enter)" << endl;
    // getchar();

    leftHandJoints = {0, -0.38705404571511043, 0.7258474179992682, -0.2, -0.9, -0.8303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.8303327436948519, 0};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.3303327436948519, 0};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.3303327436948519, 1.285967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    cout << "(main) ready to move second finger? (press enter)" << endl;
    // getchar();

    // second finger follows
    leftHandJoints = {0, -0.9303327436948519, 0.8185967300722126, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, -0.8303327436948519, 0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, 0, 0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, 0, 0.5, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(leftHandJoints);

    // first finger again
    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.9303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.8303327436948519, 0};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0, 0};
    leftHand->publishSdhJoints(leftHandJoints);

    leftHandJoints = {0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0, 0.5};
    leftHand->publishSdhJoints(leftHandJoints);

    cout << "(main) finally grasp it? (press enter)";
    // getchar();
    leftHandJoints = {SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
    leftHand->publishSdhJoints(leftHandJoints);

    cout << "(main) press a key to lift it (press enter)" << endl;
    // getchar();

    leftQueue->moveJoints(stdToArmadilloVec({-0.9291561841964722, 1.9066647291183472, 1.9648972749710083, -0.949062168598175, -0.10840536653995514, 1.199838638305664, 1.1655352115631104}));
    leftQueue->setFinish();

    if(CONTROL_RIGHT)
        rightQueue->setFinish();

    lqThread->join();
    rqThread->join();

}
