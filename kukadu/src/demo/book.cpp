#include "../utils/easyloggingpp/src/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP

#include <iostream>
#include <string>
#include <armadillo>
#include <thread>
#include <vector>
#include <boost/program_options.hpp>
#include <geometry_msgs/Pose.h>
#include "../utils/utils.h"

#include "../../include/kukadu.h"


#define ROBOT_TYPE "real"
#define ROBOT_SIDE "left"

#define CONTROL_RIGHT false
#define USE_HANDS true

using namespace std;
using namespace ros;
using namespace arma;
using namespace geometry_msgs;
namespace po = boost::program_options;

int main(int argc, char** args) {

    int importanceSamplingCount;
    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, as, alpham;
    double handVelocity = 20.0;
    string inDir, cfFile, dataFolder;
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
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/book.prop"), std::ifstream::in);
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

    cout << "all properties loaded" << endl;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    cout << "ros connection initialized" << endl;

    int kukaStepWaitTime = dmpStepSize * 1e6;

    shared_ptr<KukieControlQueue> leftQueue = shared_ptr<KukieControlQueue>(new KukieControlQueue(kukaStepWaitTime, ROBOT_TYPE, ROBOT_SIDE + string("_arm"), *node));
    shared_ptr<thread> lqThread = leftQueue->startQueueThread();

    shared_ptr<KukieControlQueue> rightQueue = nullptr;
    shared_ptr<thread> rqThread = nullptr;

    if(CONTROL_RIGHT) {

        rightQueue = shared_ptr<KukieControlQueue>(new KukieControlQueue(kukaStepWaitTime, ROBOT_TYPE, "right_arm", *node));
        rqThread = rightQueue->startQueueThread();

    }

    usleep(1e6);

    shared_ptr<RosSchunk> rightHand = nullptr;

    if(CONTROL_RIGHT) {

        if(USE_HANDS) {

            rightHand = shared_ptr<RosSchunk>(new RosSchunk(*node, ROBOT_TYPE, "right"));
            vector<double> rightHandJoints = {0, -0.5238237461313833, 0.2120872918378427, 0.8655742259109377, 1.5389379959387146, -0.6260686922290597, 0.218843743489235877};
            rightHand->publishSdhJoints(rightHandJoints);

        }

    }

    shared_ptr<RosSchunk> leftHand = nullptr;
    vector<double> handJoints = {0, -1.5, -1.0, 0, -0.2, -1.5, -1};
    if(USE_HANDS) {

        leftHand = shared_ptr<RosSchunk>(new RosSchunk(*node, ROBOT_TYPE, ROBOT_SIDE));

        leftHand->setGrasp(eGID_PARALLEL);
        leftHand->closeHand(0.0, handVelocity);
        leftHand->publishSdhJoints(handJoints);

    }

    cout << "(main) press key to continue" << endl;
    getchar();

    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);

    if(CONTROL_RIGHT) {
        rightQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
        rightQueue->moveJoints(stdToArmadilloVec({-2.3800294399261475, 1.5282957553863525, -2.280046224594116, 1.884490966796875, 2.1091063022613525, -1.4556314945220947, -0.7266652584075928}));
    }

//    Pose nextPose;
//    nextPose.position.x = 0.3; nextPose.position.y = 0.72; nextPose.position.z = 0.33;
//    nextPose.orientation.x = 0.684919567855; nextPose.orientation.y = 0.700748577783; nextPose.orientation.z = -0.138084983006; nextPose.orientation.w = -0.144113681622;

    // first moving to position where it is known that cartesian ptp works
//    leftQueue->moveJoints(stdToArmadilloVec({-1.3115264177322388, 1.1336190700531006, 1.8791016340255737, -1.4257887601852417, -1.3585442304611206, 0.5410774350166321, -1.9308620691299438}));
    leftQueue->moveJoints(stdToArmadilloVec({-0.40275293588638306, 1.7016545534133911, 1.8671916723251343, -0.6587858200073242, 0.0556875579059124, 1.1993221044540405, -1.9818705320358276}));

    /*
    leftQueue->switchMode(KukieControlQueue::KUKA_STOP_MODE);

    cout << "(main) search for book? press key to continue" << endl;
    getchar();
    // leftQueue->switchMode(KukieControlQueue::KUKA_CART_IMP_MODE);
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);

    int maxMovementDuration = 10;
    Pose currentPose = leftQueue->getCartesianPose();
    cout << currentPose.position.x << " " << currentPose.position.y << " " << currentPose.position.z << endl;
    Pose relativePose;
    relativePose.position.x = -0.0009; relativePose.position.y = relativePose.position.z = 0.0;
    // Rate slRate(20);
    double initCartForce = leftQueue->getAbsoluteCartForce();
    double currentCartForce = initCartForce;
    double maxDeviation = 10;
    for(int i = 0; abs(currentCartForce - initCartForce) < maxDeviation && i < maxMovementDuration; ++i) {
        // currentPose = leftQueue->moveCartesianRelativeWf(currentPose, relativePose);
        currentPose.position.x -= 0.01;
        leftQueue->moveCartesian(currentPose);
        // slRate.sleep();
        currentCartForce = leftQueue->getAbsoluteCartForce();
        cout << currentCartForce << " " << maxDeviation << " " << (currentCartForce - initCartForce) << " " << abs(currentCartForce - initCartForce) << endl;
    }
    */

    vector<shared_ptr<ControlQueue>> queues = {leftQueue};
    vector<shared_ptr<GenericHand>> hands = {leftHand};

    for(int i = 0;  i < 20; ++i) {

        cout << "(book) press key to prepare for next experiment" << endl;
        getchar();

        leftHand->publishSdhJoints(handJoints);
        leftQueue->moveJoints(stdToArmadilloVec({-0.40275293588638306, 1.7016545534133911, 1.8671916723251343, -0.6587858200073242, 0.0556875579059124, 1.1993221044540405, -1.9818705320358276}));

        cout << "(book) press key to start experiment number " << i << endl;
        getchar();

        stringstream s;
        s << dataFolder << "_" << i;

        SensorStorage store(queues, hands, 100);
        shared_ptr<thread> storageThread = store.startDataStorage(s.str());

        sleep(1);

        vector<double> newHandJoints = {SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, 0.4, 1.2, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
        leftHand->publishSdhJoints(newHandJoints);

        sleep(2);

        store.stopDataStorage();

        if(storageThread)
            storageThread->join();

    }

    leftQueue->setFinish();
    lqThread->join();

}
