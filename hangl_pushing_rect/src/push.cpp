#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <armadillo>
#include <kukadu/kukadu.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <iis_fast_ik/kinematics.h>
#include <boost/program_options.hpp>
#include <iis_vrep/SimInterface.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <uibk_planning_node/TrajectoryPlanner.h>
#include <moveit_msgs/ExecuteKnownTrajectoryRequest.h>

#include <hangl_vision/visionservice.h>

#include <hangl_pushing_rect/pushingrotcontroller.hpp>
#include <hangl_pushing_rect/pushforwardcontroller.hpp>
#include <hangl_pushing_rect/pushingtranscontroller.hpp>

using namespace std;
using namespace pcl;
using namespace arma;
using namespace kukadu;
using namespace pushing;
namespace pf = boost::filesystem;
namespace po = boost::program_options;

int main(int argc, char** args) {

    double az = 0.0;
    double bz = 0.0;
    double ac = 0.0;
    double tau = 0.0;
    double tolAbsErr = 0.0;
    double tolRelErr = 0.0;
    double dmpStepSize = 0.0;

    bool useReal = true;

    ifstream parseFile(resolvePath("/home/c7031109/iis_robot_sw/iis_catkin_ws/src/hangl_2016_icra/cfg/push.prop").c_str(), std::ifstream::in);

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
    ;

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

    ros::init(argc, args, "hanglpush"); ros::NodeHandle node; usleep(1e6);

    ros::AsyncSpinner spinner(10);
    spinner.start();

    ros::ServiceClient recoServ = node.serviceClient<hangl_vision::visionservice>("recognize_book");
    while(!recoServ.exists()) {
        ROS_WARN("Waiting for planning service to come up...");
        recoServ.waitForExistence(ros::Duration(10));
    }

    KUKADU_SHARED_PTR<KukieControlQueue> leftSimQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue(dmpStepSize * 1.25, string("simulation"), "left_arm", node));
    KUKADU_SHARED_PTR<KukieControlQueue> leftRealQueue;

    KUKADU_SHARED_PTR<RosSchunk> rs = KUKADU_SHARED_PTR<RosSchunk>(new RosSchunk(node, "simulation", "left"));
    rs->connectHand();

    KUKADU_SHARED_PTR<RosSchunk> rr;
    if(useReal) {
        rr = KUKADU_SHARED_PTR<RosSchunk>(new RosSchunk(node, "real", "left"));
        rr->connectHand();
    }

    if(useReal)
        leftRealQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue(dmpStepSize, string("real"), "left_arm", node));

    KUKADU_SHARED_PTR<kukadu_thread> th = leftSimQueue->startQueueThread();
    KUKADU_SHARED_PTR<kukadu_thread> thReal;
    if(useReal)
        thReal = leftRealQueue->startQueueThread();

    leftSimQueue->stopCurrentMode();
    leftSimQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);

    KUKADU_SHARED_PTR<KomoPlanner> komoPlanner = KUKADU_SHARED_PTR<KomoPlanner>(new KomoPlanner(leftSimQueue, resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/data/iis_robot.kvg"), resolvePath("$KUKADU_HOME/external/komo/share/data/kuka/config/MT.cfg"), "left"));

    if(useReal) {
        leftRealQueue->stopCurrentMode();
        leftRealQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    }

    vec pushTo = stdToArmadilloVec(createJointsVector(2, 0.2, 0.7));

    KUKADU_SHARED_PTR<PushingRotationController> push90Cont = KUKADU_SHARED_PTR<PushingRotationController>(new PushingRotationController(M_PI / 2.0, leftSimQueue, leftRealQueue, rs, rr, komoPlanner, useReal,
                                                                                                                                         KUKADU_SHARED_PTR<PushForwardController>(), KUKADU_SHARED_PTR<PushingTranslationController>(),
                                                                                                                                         KUKADU_SHARED_PTR<FinalPushController>(), node));

    KUKADU_SHARED_PTR<PushingTranslationController> transCont = KUKADU_SHARED_PTR<PushingTranslationController>(
                new PushingTranslationController(pushTo, leftSimQueue, leftRealQueue, rs, rr, komoPlanner, useReal,
                                                 KUKADU_SHARED_PTR<PushForwardController>(), node));

    bool keepRunning = true;
    while(keepRunning) {

        cout << "press key to detect book and start another rollout" << endl;
        getchar();

        //push90Cont->performAction();
        transCont->performAction();

        double newX, newY;
        cout << "where do you want to push the book next? (enter x y)" << endl;
        cin >> newX >> newY;
        pushTo(0) = newX; pushTo(1) = newY;
        transCont->setPushTo(pushTo);

    }

    if(useReal)
        rr->disconnectHand();

    rs->disconnectHand();

    leftSimQueue->stopCurrentMode();

    if(useReal)
        leftRealQueue->stopCurrentMode();

    leftSimQueue->setFinish();
    th->join();

    if(useReal) {
        leftRealQueue->setFinish();
        thReal->join();
    }

    return EXIT_SUCCESS;

}
