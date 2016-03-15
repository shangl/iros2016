#ifndef KUKADU_KUKADUUTILS
#define KUKADU_KUKADUUTILS

#include <queue>
#include <vector>
#include <math.h>
#include <vector>
#include <string>
#include <limits>
#include <cstring>
#include <sstream>
#include <stdio.h>
#include <fstream>
#include <utility>
#include <unistd.h>
#include <stdarg.h>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <dirent.h>
#include <termios.h>
#include <stdbool.h>
#include <armadillo>
#include <wordexp.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <gsl/gsl_multifit.h>
#include <eigen3/Eigen/Dense>
#include <pcl/PCLPointCloud2.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <Python.h>

#include "types.hpp"
#include "kukadutokenizer.hpp"
#include "../types/dmpbase.hpp"
#include "../robot/controlqueue.hpp"
#include "gnuplot-cpp/gnuplot_i.hpp"
#include "../learning/tricubekernel.hpp"
#include "../learning/gaussiankernel.hpp"
#include "../manipulation/controllerresult.hpp"
#include "../trajectory/dmptrajectorygenerator.hpp"
#include "../learning/gaussianprocessregressor.hpp"

namespace kukadu {

    int createDirectory(std::string path);
    void deleteFile(std::string path);
    void deleteDirectory(std::string path);

    void printDoubleVector(std::vector<double>* data);
    void printDoubleVector(double* data, int size);
    void printDoubleMatrix(double** data, int rows, int columns);
    void freeDoubleArray(double** data, int columns);
    std::string resolvePath(std::string path);

    int getch();

    arma::mat readMovements(std::string file);
    arma::mat readMovements(std::ifstream& stream);
    std::pair<std::vector<std::string>, arma::mat> readSensorStorage(std::string file);

    arma::vec readQuery(std::string file);
    std::vector<double>* createStdVectorFromGslVector(gsl_vector* vec);

    arma::vec computeDiscreteDerivatives(arma::vec x, arma::vec y);
    gsl_vector* createGslVectorFromStdVector(std::vector<double>* data);
    gsl_matrix* createMatrixFromQueueArray(std::queue<double>** data, int columns);
    gsl_matrix* invertSquareMatrix(gsl_matrix* mat);

    arma::vec createArmaVecFromDoubleArray(double* data, int n);

    double* createDoubleArrayFromStdVector(std::vector<double>* data);
    double* createDoubleArrayFromArmaVector(arma::vec data);
    float* createFloatArrayFromStdVector(std::vector<float>* data);
    double* createDoubleArrayFromVector(gsl_vector* data);
    double* polyder(double* c, int len);
    double* poly_eval_multiple(double* data, int data_len, double* c, int c_len);
    float* copyJoints(const float* arr, int arrSize);

    double roundByDigits(double number, int numDigitsBehindComma);

    double** createDoubleArrayFromMatrix(gsl_matrix* data);

    arma::vec stdToArmadilloVec(std::vector<double> stdVec);
    arma::mat fillTrajectoryMatrix(arma::mat joints, double tMax);

    std::string stringFromDouble(double d);

    std::vector<double> armadilloToStdVec(arma::vec armadilloVec);
    std::vector<double> computeDMPMys(std::vector<double> mys, double ax, double tau);

    // taken from http://rosettacode.org/wiki/Polynomial_regression
    double* polynomialfit(int obs, int degree, double *dx, double *dy);

    std::vector<std::string> getFilesInDirectory(std::string folderPath);
    std::vector<std::string> sortPrefix(std::vector<std::string> list, std::string prefix);

    std::string buildPolynomialEquation(double* w, int paramCount);
    std::vector<double>* getDoubleVectorFromArray(double* arr, int size);
    std::vector<DMPBase> buildDMPBase(std::vector<double> tmpmys, std::vector<double> tmpsigmas, double ax, double tau);

    arma::mat gslToArmadilloMatrix(gsl_matrix* matrix);
    std::vector<double>* testGaussianRegressor();
    std::vector<double> constructDmpMys(arma::mat joints);

    arma::vec squareMatrixToColumn(arma::mat m);
    arma::mat columnToSquareMatrix(arma::vec c);

    arma::vec symmetricMatrixToColumn(arma::mat m);
    arma::mat columnToSymmetricMatrix(arma::vec c);

    void set_ctrlc_exit_handler();
    void exit_handler(int s);

    arma::mat armaJoinRows(arma::vec v1, arma::mat m2);
    arma::mat armaJoinRows(arma::mat m1, arma::mat m2);

    double absolute(double val);

    geometry_msgs::Pose vectorarma2pose(arma::vec* vectorpose);
    arma::vec pose2vectorarma(geometry_msgs::Pose posepose);

    arma::vec log(const tf::Quaternion quat);
    tf::Quaternion exp(arma::vec logQuat);
    double distQuat(tf::Quaternion q1, tf::Quaternion q2);

    tf::Transform Matrix4f2Transform(Eigen::Matrix4f Tm);
    /*
     * converts roll pitch yaw to quaternions
     */
    tf::Quaternion rpyToQuat(const double roll, const double pitch, const double yaw);
    tf::Quaternion axisAngle2Quat (const double& xx, const double &yy, const double &zz, const double &a);

    double distancePoint2Line(double xp,double yp,double x1,double y1,double x2,double y2);
    arma::vec pointOnLine2Point(double xp,double yp,double x1,double y1,double x2,double y2);

    bool isDirectory(const std::string dirPath);
    bool fileExists(const std::string filePath);
    void copyFile(const std::string source, const std::string destination);

    std::vector<double> createJointsVector(int n_args, ...);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sensorMsgsPcToPclPc(sensor_msgs::PointCloud2::Ptr pc);
    sensor_msgs::PointCloud2 pclPcToSensorMsgsPc(pcl::PointCloud<pcl::PointXYZ> pc);
    pcl::PCLPointCloud2 sensorMsgsPcToPclPc2(sensor_msgs::PointCloud2 pc);
    sensor_msgs::PointCloud2 pclPcToSensorMsgsPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

    long getFileSize(std::string filename);
    std::wstring stringToWString(const std::string& s);

}

#endif
