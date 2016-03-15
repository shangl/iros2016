#ifndef KUKADU_CONTROLQUEUE
#define KUKADU_CONTROLQUEUE

#include "../utils/types.hpp"
#include "../utils/tictoc.hpp"
#include "../types/kukadutypes.hpp"
#include "../utils/destroyableobject.hpp"
#include "kinematics/restriction/restriction.hpp"

#include <queue>
#include <armadillo>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

namespace kukadu {

    /**
     * \defgroup RobotFramework
     * This framework provides hardware access to robots and hands
     */

    /** \brief Provides an interface for robot controllers
     *
     * This class provides a set of methods that are necessary to execute basic manipulations with robots. As this typically is a real time task, this class
     * contains a queue and sends data to the robot at specified times.
     * \ingroup RobotFramework
     */
    class ControlQueue : public DestroyableObject, public KUKADU_ENABLE_SHARED_FROM_THIS<ControlQueue> {

    private:

        bool isInit;
        bool finish;
        bool isShutUpFlag;
        bool rollbackMode;

        bool continueCollecting;

        bool jointPtpRunning;
        bool cartesianPtpRunning;

        int degOfFreedom;
        int rollBackQueueSize;

        double sleepTime;
        double currentTime;
        double rollBackTime;
        double lastDuration;
        double desiredCycleTime;

        arma::vec currentJoints;
        arma::vec startingJoints;
        arma::vec internalJointPasser;

        geometry_msgs::Pose currentCartPose;
        geometry_msgs::Pose internalPosePasser;

        std::vector<mes_result> collectedJoints;

        std::queue<arma::vec> movementQueue;
        std::queue<geometry_msgs::Pose> cartesianMovementQueue;

        std::deque<arma::vec> rollBackQueue;

        TicToc t;

        KUKADU_SHARED_PTR<kukadu_thread> thr;
        KUKADU_SHARED_PTR<kukadu_thread> cartPtpThr;
        KUKADU_SHARED_PTR<kukadu_thread> jointPtpThr;
        KUKADU_SHARED_PTR<kukadu_thread> jointsColletorThr;

        void setInitValuesInternal();
        void internalCartPtpCaller();
        void internalJointPtpCaller();

        void jointsCollector();

    protected:

        int currentControlType;

        virtual void setInitValues() = 0;
        virtual void jointPtpInternal(arma::vec joints) = 0;
        virtual void submitNextJointMove(arma::vec joints) = 0;
        virtual void submitNextCartMove(geometry_msgs::Pose pose) = 0;
        virtual void setCurrentControlTypeInternal(int controlType) = 0;
        virtual void cartPtpInternal(geometry_msgs::Pose pose, double maxForce) = 0;

        virtual void startQueueThreadHook() = 0;

        /**
         * @brief this method determines, if the queue execetion should be stopped while ptp commands are executed
         * (this is typically the case when ptp is done outside of the control queue implementation). if two different
         * controls interfere it can result in dangerous behaviour
         * @return
         */
        virtual bool stopQueueWhilePtp() = 0;

    public:

        /** \brief Constructor taking the robot dependent degrees of freedom
         * \param degOfFreedom number of robots degrees of freedom
         */
        ControlQueue(int degOfFreedom, double desiredCycleTime);

        /**
         * \brief Returns number of robots degrees of freedom
         */
        int getMovementDegreesOfFreedom();

        /**
         * \brief Starts new thread to control the robot with real time capability
         */
        KUKADU_SHARED_PTR<kukadu_thread> startQueueThread();

        /**
         * \brief This method is started in a new thread by startQueue
         */
        virtual void run();

        virtual double getTimeStep();

        virtual double getMeasuredTimeStep();

        virtual bool getQueueRunning();

        /**
         * \brief Sets a flag to stop the control thread after current iteration is executed
         */
        virtual void setFinish();

        /**
         * \brief Adds next joint position to queue
         * \param joints joints to add
         */
        virtual void addJointsPosToQueue(arma::vec joints);

        /**
         * \brief Adds next cartesian position to queue
         * \param pose end-effector pose to add
         */
        virtual void addCartesianPosToQueue(geometry_msgs::Pose pose);

        /**
         * \brief Switches robot modes. A state might be a real time command mode or an monitoring mode
         * \param mode mode id
         */
        virtual void switchMode(int mode);

        /**
         * \brief Stops current mode and switches back to default mode (e.g. monitoring mode)
         */
        virtual void stopCurrentMode();

        /**
         * \brief Blocks, if more than the defined maximum element count is in the queue
         * \param maxNumJointsInQueue maximum number of joints in queue
         */
        virtual void synchronizeToControlQueue(int maxNumJointsInQueue);

        /**
         * \brief Sets joints in which the should be in before robot enters command mode
         * \param joints array of joint positions
         */
        virtual void setStartingJoints(arma::vec joints);

        /**
         * \brief Implements simple point to point movement in joint space (blocks until target reached)
         * \param joints array of joint positions
         */
        virtual std::vector<mes_result> jointPtp(arma::vec joints);

        /**
         * \brief Implements simple point to point movement in joint space (not blocking)
         * \param joints array of joint positions
         */
        KUKADU_SHARED_PTR<kukadu_thread> jointPtpNb(arma::vec joints);

        /**
         * \brief Implements simple point to point movement in cartesian space
         * \param pose of end-effector
         */
        virtual std::vector<mes_result> cartesianPtp(geometry_msgs::Pose pos);

        virtual std::vector<mes_result> cartesianPtp(geometry_msgs::Pose pos, double maxForce);

        /**
         * \brief Implements simple point to point movement in cartesian space (does not block until the position is reached)
         * \param pose of end-effector
         */
        virtual KUKADU_SHARED_PTR<kukadu_thread> cartesianPtpNb(geometry_msgs::Pose pos);

        /**
         * \brief Changes the load data of the robot (e.g. needs to be used whenever robot picks up an object)
         * \param loadMass mass of the picked up object
         * \param loadPos position of the objects center of gravity relative to the manipulator
         */
        virtual void setAdditionalLoad(float loadMass, float loadPos) = 0;

        /**
         * \brief Sets certain stiffness parameters in cartesian space (if the robot supports this) according to a mass spring damper system model
         * \param cpstiffnessxyz stiffness of the robot in the cartesian space
         * \param cpstiffnessabc stiffness of the rotational axis of the tool mounting point in cartesian space
         * \param cpdamping damping of the robots axis in cartesian space
         * \param cpmaxdelta maximum allows deviation in cartesian space
         * \param maxforce maximum allowed applied force
         * \param axismaxdeltatrq maximum allowed applied torque
         */
        virtual void setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) = 0;

        /**
         * \brief Returns current robot position in cartesian space
         */
        virtual mes_result getCurrentCartesianPos();

        /**
         * \brief Returns current robot position in cartesian space
         */
        virtual geometry_msgs::Pose getCurrentCartesianPose() = 0;

        /**
         * \brief Returns the robot joints the robot has been directly before starting command mode
         */
        virtual arma::vec getStartingJoints();

        /**
         * \brief Returns joints if the robot is in command mode
         */
        virtual mes_result getCurrentJoints() = 0;

        virtual mes_result getCurrentJntFrcTrq() = 0;

        virtual mes_result getCurrentCartesianFrcTrq() = 0;

        virtual void setJntPtpThresh(double thresh) = 0;

        /**
         * \brief Returns true if the command mode initialization is done
         */
        virtual bool isInitialized();

        virtual std::string getRobotName() = 0;
        virtual std::string getRobotFileName() = 0;
        virtual std::vector<std::string> getJointNames() = 0;

        virtual double getAbsoluteCartForce();

        // kills command line output of queue
        virtual void shutUp();
        virtual void startTalking();

        virtual void rollBack(double time);
        virtual void stopJointRollBackMode();
        virtual void startJointRollBackMode(double possibleTimeReach);

        virtual double getCurrentTime();

        bool isShutUp();

        virtual int getCurrentControlType() = 0;

        static const int CONTROLQUEUE_STOP_MODE = 0;
        static const int CONTROLQUEUE_JNT_POS_MODE = 10;
        static const int CONTROLQUEUE_CART_IMP_MODE = 20;
        static const int CONTROLQUEUE_JNT_IMP_MODE = 30;

    };

}

#endif
