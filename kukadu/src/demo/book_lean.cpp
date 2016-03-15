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

#define ROBOT_TYPE "real"
#define ROBOT_SIDE "left"

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

    shared_ptr<KukieControlQueue> leftQueue = shared_ptr<KukieControlQueue>(new KukieControlQueue(kukaStepWaitTime, ROBOT_TYPE, ROBOT_SIDE + string("_arm"), *node));
    shared_ptr<thread> lqThread = leftQueue->startQueueThread();

    leftQueue->setJntPtpThresh(2.5);

    leftQueue->switchMode(KukieControlQueue::KUKA_STOP_MODE);
    leftQueue->switchMode(KukieControlQueue::KUKA_JNT_IMP_MODE);
    //leftQueue->moveJoints(stdToArmadilloVec({-0.514099, 1.83194, 1.95971, -0.99676, -0.0903862, 0.987185, 1.16542}));


    cout << "(main) preparing to drop (press enter)" << endl;
    getchar();
    leftQueue->moveJoints(stdToArmadilloVec({-1.0587048530578613, 1.879784107208252, 2.1199851036071777, -1.9964550733566284, -1.975645899772644, 1.5456539392471313, 2.5950534343719482}));

    cout << "(main) finally grasp it? (press enter)";
    getchar();
    leftHandJoints = {0, 0, 0.5, -0.2, -0.9, 0, 0.5};
    leftHand->publishSdhJoints(leftHandJoints);

    cout << "(main) prepare for leaning (press enter)" << endl;
    getchar();
    leftQueue->moveJoints(stdToArmadilloVec({-0.6866786479949951, 1.6707192659378052, 2.3672842979431152, -1.725509524345398, -2.167886734008789, 1.6949800252914429, 2.642235040664673}));

    cout << "(main) lean book (press enter)" << endl;
    getchar();
    leftQueue->moveJoints(stdToArmadilloVec({-0.42627784609794617, 1.879958152770996, 2.4736852645874023, -1.6400219202041626, -2.337477207183838, 1.3205868005752563, 2.0334768295288086}));

    cout << "(main) release book (press enter)" << endl;
    getchar();
    leftHandJoints = {0, -0.48705404571511043, 0.7258474179992682, -0.650410616072391092, 0.5, -0.5303327436948519, 0.8185967300722126};
    leftHand->publishSdhJoints(leftHandJoints);

    cout << "(main) releasing book (press enter)" << endl;
    getchar();
    leftQueue->moveJoints(stdToArmadilloVec({-0.5591384172439575, 1.8514913320541382, 2.669778823852539, -1.981245756149292, -2.3048179149627686, 1.3562568426132202, 2.085831642150879}));

    leftQueue->switchMode(KukieControlQueue::KUKA_STOP_MODE);
    cout << "done" << endl;

    leftQueue->setFinish();
    lqThread->join();

    return EXIT_SUCCESS;

}
