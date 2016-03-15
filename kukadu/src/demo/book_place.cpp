#include "../utils/easyloggingpp/src/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP

#include <geometry_msgs/Pose.h>
#include <iostream>
#include <string>
#include <armadillo>
#include <thread>
#include <vector>
#include <boost/program_options.hpp>

#include "../../include/kukadu.h"

#define ROBOT_TYPE "simulation"
#define ROBOT_SIDE "left"

#define CONTROL_RIGHT false

using namespace std;
using namespace ros;
using namespace arma;
using namespace geometry_msgs;
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

    shared_ptr<RosSchunk> leftHand = shared_ptr<RosSchunk>(new RosSchunk(*node, ROBOT_TYPE, ROBOT_SIDE));
    vector<double> leftHandJoints = {0, -0.38705404571511043, 0.7258474179992682, 0.010410616072391092, -1.2259735578027993, -0.4303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    shared_ptr<RosSchunk> rightHand = shared_ptr<RosSchunk>(new RosSchunk(*node, ROBOT_TYPE, "right"));
    //vector<double> rightHandJoints = {0, -0.5231897140548627, -0.09207113810135568, 0.865287869514727, 1.5399390781924753, -0.6258846012187079, 0.01877915351042305};
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

    leftQueue->setJntPtpThresh(2.5);

    if(CONTROL_RIGHT) {

        rightQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
        rightQueue->moveJoints(stdToArmadilloVec({-2.0805978775024414, 1.4412447214126587, -2.203258752822876, 1.7961543798446655, 2.526369333267212, -1.6385622024536133, -0.7103539705276489}));

    }

    leftQueue->switchMode(KukieControlQueue::KUKA_STOP_MODE);
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);

    cout << "(main) preparing to grasp (press key)" << endl;
    getchar();
    leftQueue->moveJoints(stdToArmadilloVec({-1.0587048530578613, 1.879784107208252, 2.1199851036071777, -1.9964550733566284, -1.975645899772644, 1.5456539392471313, 2.5950534343719482}));

    cout << "(main) finally grasp it? (press key)";
    getchar();
    leftHandJoints = {0, 0, 0.5, -0.2, -0.9, 0, 0.5};
    leftHand->publishSdhJoints(leftHandJoints);

    cout << "(main) dropping book" << endl;
    leftQueue->moveJoints(stdToArmadilloVec({-0.3186831772327423, 1.877084493637085, 2.4300241470336914, -1.6613903045654297, -2.41784930229187, 1.776635766029358, 2.6922547817230225}));

    cout << "(main) opening fingers (press key)" << endl;
    vector<double> openJoints = leftHandJoints = {0, -0.48705404571511043, 0.7258474179992682, -0.650410616072391092, 0.5, -0.5303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(leftHandJoints);
    getchar();
    leftQueue->switchMode(KukieControlQueue::KUKA_STOP_MODE);
    cout << "(main) starting fine grained placement (press key)" << endl;
    getchar();
    leftQueue->switchMode(KukieControlQueue::KUKA_CART_IMP_MODE);

    cout << "(main) start moving" << endl;

    int movementDuration = 70;
    double frcVal1, frcVal2, dist1, dist2;
    mes_result mes;
    Pose currentPose = leftQueue->getCartesianPose();
    Pose relativePose;
    relativePose.position.x = 0.0009; relativePose.position.y = relativePose.position.z = 0.0;
    Rate slRate(20);
    Rate waitRate(0.5);
    int stableCount = 0;
    double moveAbs = 0.0014;

    while(1) {

        // check back side
        mes = leftQueue->getCurrentCartesianFrcTrq();
        frcVal1 = mes.joints(4);
        cout << "1: " << mes.joints.t() << endl;
        leftHandJoints = {SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.8, 0.3, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT};
        leftHand->publishSdhJoints(leftHandJoints);
        waitRate.sleep();
        mes = leftQueue->getCurrentCartesianFrcTrq();
        frcVal2 = mes.joints(4);
        cout << "1: " << mes.joints.t() << endl;
        dist1 = abs(frcVal1 - frcVal2);
        leftHandJoints = openJoints;
        leftHand->publishSdhJoints(leftHandJoints);


        // check front side
        mes = leftQueue->getCurrentCartesianFrcTrq();
        frcVal1 = mes.joints(4);
        cout << "2: " << mes.joints.t() << endl;
        leftHandJoints = {SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, SDH_IGNORE_JOINT, -0.7, 0.6};
        leftHand->publishSdhJoints(leftHandJoints);
        waitRate.sleep();
        mes = leftQueue->getCurrentCartesianFrcTrq();
        frcVal2 = mes.joints(4);
        cout << "2: " << mes.joints.t() << endl;
        dist2 = abs(frcVal1 - frcVal2);

        leftHandJoints = openJoints;
        leftHand->publishSdhJoints(leftHandJoints);

        if(dist1 < 0.07 && dist2 < 0.07) {
            cout << "book stable with dists " << dist1 << " " << dist2 << endl;

            if(++stableCount == 3) {
                leftHand->setGrasp(eGID_PARALLEL);
                leftHand->closeHand(0.0, 20);
                break;
            }

        } else if(dist1 > dist2) {

            cout << "forwards with dists " << dist1 << " " << dist2 << endl;
            relativePose.position.x = moveAbs;

            for(int i = 0; i < movementDuration; ++i) {
                currentPose = leftQueue->moveCartesianRelativeWf(currentPose, relativePose);
                slRate.sleep();
            }

        } else if(dist1 < dist2) {

            cout << "backwards with dists " << dist1 << " " << dist2 << endl;
            relativePose.position.x = -moveAbs;

            for(int i = 0; i < movementDuration; ++i) {
                currentPose = leftQueue->moveCartesianRelativeWf(currentPose, relativePose);
                slRate.sleep();
            }

        }

    //    getchar();

    }

    leftQueue->switchMode(KukieControlQueue::KUKA_STOP_MODE);
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    leftQueue->moveJoints(stdToArmadilloVec({-1.032808542251587, 1.772948980331421, 2.3052618503570557, -1.9651068449020386, -1.8061085939407349, 1.7254467010498047, 2.702965259552002}));
    cout << "done" << endl;

    leftQueue->switchMode(KukieControlQueue::KUKA_STOP_MODE);
    leftQueue->setFinish();

    if(CONTROL_RIGHT)
        rightQueue->setFinish();

    lqThread->join();
    rqThread->join();

    return EXIT_SUCCESS;

}
