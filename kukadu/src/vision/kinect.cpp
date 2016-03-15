    #include "kinect.hpp"

#include "../utils/utils.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

namespace kukadu {

    Kinect::Kinect(ros::NodeHandle node) {
        string kinectPrefix = "kinect";
        construct(kinectPrefix, kinectPrefix + "_depth_frame", node);
    }

    Kinect::Kinect(std::string kinectPrefix, ros::NodeHandle node) {
        construct(kinectPrefix, kinectPrefix + "_depth_frame", node);
    }

    Kinect::Kinect(std::string kinectPrefix, std::string targetFrame, ros::NodeHandle node) {
        construct(kinectPrefix, targetFrame, node);
    }

    void Kinect::construct(std::string kinectPrefix, std::string targetFrame, ros::NodeHandle node) {

        stdVisPubTopic = "/kukadu/rviz";

        isInit = false;
        keepRunning = false;
        pcRequested = false;
        firstCloudSet = false;

        this->node = node;

        this->kinectPrefix = "/" + kinectPrefix;
        subKinect = node.subscribe(kinectPrefix + "/depth_registered/points", 1, &Kinect::callbackKinectPointCloud, this);

        this->visPubTopic = stdVisPubTopic;
        visPublisher = node.advertise<sensor_msgs::PointCloud2>(visPubTopic, 1);

        transformListener = KUKADU_SHARED_PTR<tf::TransformListener>(new tf::TransformListener());

        ros::Rate r(10);
        while(!firstCloudSet) {
            r.sleep();
            ros::spinOnce();
        }

        transformListener->waitForTransform(targetFrame, currentPc.header.frame_id, ros::Time::now(), ros::Duration(5.0));

        this->targetFrame = targetFrame;

    }

    KUKADU_SHARED_PTR<kukadu_thread> Kinect::startSensing() {
        keepRunning = true;
        thr = KUKADU_SHARED_PTR<kukadu_thread>(new kukadu_thread(&Kinect::runThread, this));
        while(!this->isInitialized());
        return thr;
    }

    void Kinect::stopSensing() {
        keepRunning = false;
    }

    void Kinect::runThread() {

        isInit = true;
        ros::Rate r(10);
        while(keepRunning) {
            ros::spinOnce();
            r.sleep();
        }

    }

    bool Kinect::isInitialized() {
        return isInit;
    }

    void Kinect::callbackKinectPointCloud(const sensor_msgs::PointCloud2& pc) {

        pcMutex.lock();

            if(pcRequested || !firstCloudSet) {
                currentPc = pc;
            }

            pcRequested = false;
            firstCloudSet = true;

        pcMutex.unlock();

    }

    sensor_msgs::PointCloud2 Kinect::getCurrentPointCloud() {

        pcRequested = true;

        pcMutex.lock();

            sensor_msgs::PointCloud2 retCloud = currentPc;
            firstCloudSet = false;

        pcMutex.unlock();

        pcl_ros::transformPointCloud(targetFrame, retCloud, retCloud, *transformListener);

        return retCloud;

    }

    std::string Kinect::getVisPubTopic() {
        return visPubTopic;
    }

    void Kinect::setVisPubTopic(std::string visPubTopic) {
        this->visPubTopic = visPubTopic;
        visPublisher = node.advertise<sensor_msgs::PointCloud2>(this->visPubTopic, 1);
    }

    void Kinect::visualizeCurrentPc() {
        visPublisher.publish(getCurrentPointCloud());
    }

    void Kinect::visualizeCurrentTransformedPc(KUKADU_SHARED_PTR<PCTransformator> transformator) {

        pcl::PointCloud<pcl::PointXYZ> currentPc = sensorMsgsPcToPclPc(getCurrentPointCloud());
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed = transformator->transformPc(currentPc.makeShared());
        visPublisher.publish(pclPcToSensorMsgsPc(transformed));

    }

    void Kinect::storeCurrentPc(std::string fileName) {

        pcl::io::savePCDFile(fileName, sensorMsgsPcToPclPc2(getCurrentPointCloud()), Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);

    }

}
