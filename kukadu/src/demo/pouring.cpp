#include <stdio.h>
#include <iostream>
#include <string>
#include <armadillo>
#include <thread>
#include <vector>
#include <boost/program_options.hpp>

#include "../../include/kukadu.h"

using namespace std;
using namespace arma;
namespace po = boost::program_options;

void moveLeftArm();
void switch2dQueryPoint();

bool stopThread;
double k = 11.778701751;
double d = 0.982426177;
double currentY = 0.48;
shared_ptr<DictionaryGeneralizer> dmpGen;
shared_ptr<ControlQueue> holderQueue;

int main(int argc, char** args) {

    int importanceSamplingCount, useSimulation, performRl, maxRLIterations, learnMetric;
    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, as, alpham;
    string inDir, cfFile, simPrefix;
    vector<double> rlExploreSigmas;

    // only one time center selected
    vec timeCenters(1); timeCenters(0) = 2.0;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("metric.database", po::value<string>(), "database folder")
            ("metric.goldStandard", po::value<string>(), "gold standard file")
            ("metric.alpham", po::value<double>(), "alpham")
            ("metric.exploreSigmas", po::value<string>(), "reinforcement learning exploration sigmas")
            ("metric.importanceSamplingCount", po::value<int>(), "size of importance sampling vector")
            ("metric.performRL", po::value<int>(), "use reinforcement learning?")
            ("metric.as", po::value<double>(), "as")
            ("metric.maxRLIterations", po::value<int>(), "maximum number of reinforcement learning iterations")
            ("metric.learnMetric", po::value<int>(), "learn metric or use predefined one?")
            ("dmp.tau", po::value<double>(), "tau")
            ("dmp.az", po::value<double>(), "az")
            ("dmp.bz", po::value<double>(), "bz")
            ("dmp.dmpStepSize", po::value<double>(), "dmp time step size")
            ("dmp.tolAbsErr", po::value<double>(), "tolerated absolute error")
            ("dmp.tolRelErr", po::value<double>(), "tolerated relative error")
            ("dmp.ac", po::value<double>(), "ac")
            ("dmp.useSimulation", po::value<int>(), "use simulation?")
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/pouring.prop"), std::ifstream::in);
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
    if (vm.count("dmp.useSimulation")) useSimulation = vm["dmp.useSimulation"].as<int>();
    else return 1;

    simPrefix = useSimulation ? "simulation" : "real";

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
    if (vm.count("metric.performRL")) performRl = vm["metric.performRL"].as<int>();
    else return 1;
    if (vm.count("metric.maxRLIterations")) maxRLIterations = vm["metric.maxRLIterations"].as<int>();
    else return 1;
    if (vm.count("metric.learnMetric")) learnMetric = vm["metric.learnMetric"].as<int>();
    else return 1;

    cout << "all loaded" << endl;

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    cout << "ros connection initialized" << endl;

    int kukaStepWaitTime = dmpStepSize * 1e6;
    shared_ptr<ControlQueue> simulationQueue = shared_ptr<ControlQueue>(new PlottingControlQueue(7, kukaStepWaitTime));
    holderQueue = shared_ptr<ControlQueue>(new KukieControlQueue(kukaStepWaitTime, simPrefix, "right_arm", *node));
    shared_ptr<ControlQueue> executionQueue = shared_ptr<ControlQueue>(new KukieControlQueue(kukaStepWaitTime, simPrefix, "left_arm", *node));
    shared_ptr<Gnuplot> g1;
    shared_ptr<thread> switchThr;

    int rolloutsPerUpdate = 2 * 4 * timeCenters.n_elem;

    vec trajMetricWeights(7);
    trajMetricWeights.fill(1.0);

    tau = 5.0;
    double relativeDistanceThresh = 0.6;

    vec newQueryPoint(2);
    newQueryPoint(0) = 11.0;
    newQueryPoint(1) = 234.0;

    mat m(2,2);
    m(0, 0) = 1.0;
    m(1, 0) = 0;
    m(0, 1) = 0;
    m(1, 1) = 0.0007;

    cout << "(main) creating dictionary generalizer object" << endl;
    if(learnMetric)
        dmpGen = std::shared_ptr<DictionaryGeneralizer>(new DictionaryGeneralizer(timeCenters, newQueryPoint, simulationQueue, executionQueue, inDir, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, trajMetricWeights, relativeDistanceThresh, as, alpham));
    else
        dmpGen = std::shared_ptr<DictionaryGeneralizer>(new DictionaryGeneralizer(timeCenters, newQueryPoint, simulationQueue, executionQueue, inDir, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac, as, m, relativeDistanceThresh, alpham));

    cout << "(main) done" << endl;

    std::shared_ptr<DmpRewardComputer> reward = std::shared_ptr<DmpRewardComputer>(new DmpRewardComputer(resolvePath(cfFile), az, bz, dmpStepSize, dmpGen->getDegOfFreedom(), dmpGen->getTrajectory()->getTmax(), 0.1));
    cout << "execute ground truth" << endl;
    shared_ptr<ControllerResult> opt = reward->getOptimalTraj();
    cout << "execution done" << endl;

    cout << "(main) initializing trajectory" << endl;
    std::vector<std::shared_ptr<Trajectory>> initTraj;
    initTraj.push_back(dmpGen->getTrajectory());
    cout << "(main) done" << endl;

    cout << "(main) query point to maximize is: " << newQueryPoint << endl;
    std::shared_ptr<LinCombDmp> lastRollout = std::shared_ptr<LinCombDmp>(nullptr);

    cout << "(main) creating reinforcement learning environment" << endl;
    PoWER pow(dmpGen, initTraj, rlExploreSigmas, rolloutsPerUpdate, importanceSamplingCount, reward, simulationQueue, executionQueue, ac, dmpStepSize, tolAbsErr, tolRelErr);
    cout << "(main) done" << endl;

    vector<std::shared_ptr<Gnuplot>> gs;
    for(int i = 0; i < dmpGen->getDegOfFreedom(); ++i)
        gs.push_back(std::shared_ptr<Gnuplot>(new Gnuplot("PoWER demo")));

    int i = 0;
    vec initT;
    vector<vec> initY;

    cout << "(main) setting up robot" << endl;

    geometry_msgs::Pose rightStartPos;
    rightStartPos.position.x = 0.1266098458318735; rightStartPos.position.y = 0.6804381841413323; rightStartPos.position.z = 0.26416318113180093;
    rightStartPos.orientation.x = -0.351192181682; rightStartPos.orientation.y = 0.563953194328; rightStartPos.orientation.z = 0.461341670882; rightStartPos.orientation.w = 0.588061582881;

    vec leftStartPos = {-1.3379182815551758, 1.1682424545288086, 1.1133999824523926, -1.7567673921585083, -1.0026493072509766, 0.8772866725921631, 0.6723997592926025};

    holderQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    holderQueue->moveCartesian(rightStartPos);

    simulationQueue->startQueueThread();
    simulationQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);

    executionQueue->startQueueThread();
    executionQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);


    simulationQueue->moveJoints(leftStartPos);
    cout << "if simulation was ok, press a key to execute on real robot..." << endl; getchar();
    executionQueue->moveJoints(leftStartPos);

    cout << "(main) done" << endl;
    int plotNum = dmpGen->getDegOfFreedom();

    if(learnMetric && useSimulation) {

        while(i < maxRLIterations) {

            cout << "(main) performing rollout" << endl;
            pow.performRollout(1, 0);
            lastRollout = std::dynamic_pointer_cast<LinCombDmp>(pow.getLastUpdate());
            cout << "(main) retrieving last metric" << endl;
            vector<Mahalanobis> metrics = lastRollout->getMetric();

            cout << "used metrics: " << endl;
            for(int i = 0; i < metrics.size(); ++i)
                cout << metrics.at(i).getM() << endl;

            if(i == 0) {
                initT = pow.getLastUpdateRes()->getTimes();
                initY = pow.getLastUpdateRes()->getYs();
            }


            if( (i % 1) == 0 ) {

                for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {

                    ostringstream convert;   // stream used for the conversion
                    convert << plotTraj;

                    g1 = gs.at(plotTraj);
                    g1->set_style("lines").plot_xy(armadilloToStdVec(opt->getTimes()), armadilloToStdVec(opt->getYs()[plotTraj]), "optimal trajectoy");
                    g1->set_style("lines").plot_xy(armadilloToStdVec(initT), armadilloToStdVec(initY[plotTraj]), "initial trajectoy");
                    g1->set_style("lines").plot_xy(armadilloToStdVec(pow.getLastUpdateRes()->getTimes()), armadilloToStdVec(pow.getLastUpdateRes()->getYs()[plotTraj]), "generalized trajectory");
                    g1->showonscreen();

                }

            }

            for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {
                g1 = gs.at(plotTraj);
                g1->reset_plot();
            }

            if( (i % 20) == 19) {
                for(int plotTraj = 0; plotTraj < plotNum; ++plotTraj) {
                    g1 = gs.at(plotTraj);
                    g1->remove_tmpfiles();
                }
            }

            ++i;

        }

    }

    cout << "(PouringExperiment) switching mode for holding arm" << endl;
    holderQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);

    cout << "(PouringExperiment) execution of trajectory at new position" << endl;

    while( 1 ) {

        cout << "(PouringExperiment) location (first coordinate) (< 0 for exit): " << endl;
        cin >> newQueryPoint(0);

        if(newQueryPoint(0) < 0)
            break;

        cout << "(PouringExperiment) amount to pour (second coordinate) (< 0 for exit): " << endl;
        cin >> newQueryPoint(1);

        if(newQueryPoint(1) < 0)
            break;

        fflush(stdin); fflush(stdout);

        dmpGen->setAs(as);

        lastRollout = std::dynamic_pointer_cast<LinCombDmp>(dmpGen->getTrajectory());
        lastRollout->setCurrentQueryPoint(newQueryPoint);
        dmpGen->switchQueryPoint(newQueryPoint);

        vec startingPos = dmpGen->getTrajectory()->getStartingPos();

        /*
        switchThr = shared_ptr<thread>(new std::thread(switch2dQueryPoint));

        stopThread = true;
        switchThr->join();

        simulationQueue->moveJoints(startingPos);
        */

        char cont = 'n';
        cout << "(main) do you want to execute this trajectory? (y/N) ";
        cin >> cont;

        if(cont == 'y' || cont == 'Y') {

            cout << "(main) executing trial" << endl;
            shared_ptr<thread> moveArmThread = shared_ptr<thread>(new std::thread(moveLeftArm));

            lastRollout = std::dynamic_pointer_cast<LinCombDmp>(dmpGen->getTrajectory());
            lastRollout->setCurrentQueryPoint(newQueryPoint);
            dmpGen->switchQueryPoint(newQueryPoint);

            executionQueue->moveJoints(startingPos);
            switchThr = shared_ptr<thread>(new std::thread(switch2dQueryPoint));
            dmpGen->executeTrajectory();
            stopThread = true;
            switchThr->join();

            ungetc('c', stdin);
            moveArmThread->join();

        }

    }


}

void moveLeftArm() {

    geometry_msgs::Pose currentPose = holderQueue->getCartesianPose();
    cout << currentPose.position.x << " " << currentPose.position.y << " " << currentPose.position.z << endl;
    ros::Rate sl(20);
    while( 1 ) {

        int key = getch();
        if(key == 68) {
            currentPose.position.y -= 0.005;
            holderQueue->moveCartesianNb(currentPose);
        } else if(key == 67) {
            currentPose.position.y += 0.005;
            holderQueue->moveCartesianNb(currentPose);
        } else if(key == 'q')
            break;

        sl.sleep();

    }
}

/* switch query point for pouring */
void switch2dQueryPoint() {

    stopThread = false;

    vec qp(2);
    qp(0) = 9;
    qp(1) = 400;

    ros::Rate sleepRate(50);
    for(int i = 0; !stopThread; ++i) {

        ros::spinOnce();

        double currentY = holderQueue->getCartesianPos().joints(1);

        sleepRate.sleep();
        double oldQp = qp(0);
        qp(0) = k * currentY + d;

        if(oldQp != qp(0)) {
            cout << "currentY: " << currentY << endl;
            cout << "switching execution to position " << qp(0) << endl;
            dmpGen->switchQueryPoint(qp);
        }

    }

}
