#ifndef KUKADU_KINECT_H
#define KUKADU_KINECT_H

#include <vector>
#include <cstdio>
#include <string>
#include <fstream>
#include <iostream>
#include <wordexp.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf2_msgs/TFMessage.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include "pcltransformator.hpp"
#include "../types/kukadutypes.hpp"

namespace kukadu {

    class Kinect {

    private:

        std::string stdVisPubTopic;

        bool isInit;
        bool keepRunning;
        bool firstCloudSet;

        bool pcRequested;

        KUKADU_SHARED_PTR<kukadu_thread> thr;

        kukadu_mutex pcMutex;

        std::string targetFrame;
        std::string visPubTopic;
        std::string kinectPrefix;

        KUKADU_SHARED_PTR<tf::TransformListener> transformListener;

        sensor_msgs::PointCloud2 currentPc;

        ros::NodeHandle node;

        ros::Subscriber subKinect;
        ros::Subscriber subTransformation;

        ros::Publisher visPublisher;

        void runThread();
        void callbackKinectPointCloud(const sensor_msgs::PointCloud2& pc);
        void construct(std::string kinectPrefix, std::string targetFrame, ros::NodeHandle node);

    public:

        Kinect(ros::NodeHandle node);
        Kinect(std::string kinectPrefix, ros::NodeHandle node);
        Kinect(std::string kinectPrefix, std::string targetFrame, ros::NodeHandle node);

        void stopSensing();
        void visualizeCurrentPc();
        void storeCurrentPc(std::string fileName);
        void setVisPubTopic(std::string visPubTopic);
        void visualizeCurrentTransformedPc(KUKADU_SHARED_PTR<PCTransformator> transformator);

        bool isInitialized();

        std::string getVisPubTopic();

        sensor_msgs::PointCloud2 getCurrentPointCloud();

        KUKADU_SHARED_PTR<kukadu_thread> startSensing();

    };

}

#endif
