#ifndef KUKADU_VISIONINTERFACE_H
#define KUKADU_VISIONINTERFACE_H

#include <tf/tf.h>
#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <eigen3/Eigen/Dense>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

#include "../utils/utils.hpp"

namespace kukadu {

    class VisionInterface {

    private:

        int sleepTime;

        bool firstSet;
        bool arTagTracker;

        std::string arTagTopic;
        std::string currentCameraTag;

        ros::NodeHandle node;
        ros::Subscriber subArTag;

        tf::Transform tfChestKin;

        geometry_msgs::Pose currentArPose;

		void setArTagTracker();
        void setCameraTag(std::string cameraTag);
        void arTagCallback(const tf::tfMessage& msg);
        void construct(int sleepTime, ros::NodeHandle node, std::string cameraTag, bool arTagTracker);

    public:

        VisionInterface(int sleepTime, ros::NodeHandle node);
        VisionInterface(int sleepTime, std::string cameraTag, ros::NodeHandle node);

        geometry_msgs::Pose getArPose();

    };

}

#endif // VISIONINTERFACE_H
