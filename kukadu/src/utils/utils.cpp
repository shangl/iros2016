#include <math.h>
#include <istream>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sys/stat.h>
#include <sys/types.h>
#include <boost/filesystem.hpp>

#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include "../utils/utils.hpp"
#include "../types/kukadutypes.hpp"
#include "../trajectory/dmpexecutor.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    int createDirectory(std::string path) {

        struct stat st = {0};

        if (stat(path.c_str(), &st) == -1) {
            mkdir(path.c_str(), 0700);
            return true;
        } else return false;

    }

    std::string resolvePath(std::string path) {

        wordexp_t p;
        wordexp(path.c_str(), &p, 0 );
        char** w = p.we_wordv;
        string ret = string(*w);
        wordfree( &p );

        return ret;

    }

    arma::vec createArmaVecFromDoubleArray(double* data, int n) {
        arma::vec ret(n);
        for(int i = 0; i < n; ++i)
            ret(i) = data[i];
        return ret;
    }

    /* reimplements the getch() function available on windows
     *
     * returns: sign take from console
     * input: -
    */
    int getch() {
        struct termios oldt, newt;
        int ch;
        tcgetattr( STDIN_FILENO, &oldt );
        newt = oldt;
        newt.c_lflag &= ~( ICANON | ECHO );
        tcsetattr( STDIN_FILENO, TCSANOW, &newt );
        ch = getchar();
        tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
        return ch;
    }

    std::vector<double> constructDmpMys(mat joints) {
        vector<double> ret;
        double tmax = joints(joints.n_rows - 1, 0);
        for(double i = 0; i < (tmax + 1); i += 1.0) {
            ret.push_back(i);
        }
        return ret;
    }

    float* copyJoints(const float* arr, int arrSize) {
        float* ret = new float[arrSize];
        for(int i = 0; i < arrSize; ++i) ret[i] = arr[i];
        return ret;
    }

    void printDoubleVector(vector<double>* data) {
        int dataSize = data->size();
        for(int i = 0; i < dataSize; ++i) {
            cout << data->at(i) << ", ";
        }
        cout << endl;
    }

    vector<double> computeDMPMys(vector<double> mys, double ax, double tau) {
        int mysSize = mys.size();
        vector<double> dmpMys;
        for(int i = 0; i < mysSize; ++i) {
            double val = std::exp(-ax / tau * mys.at(i));
            dmpMys.push_back(val);
        }
        return dmpMys;
    }

    std::vector<double>* getDoubleVectorFromArray(double* arr, int size) {

        vector<double>* v = new vector<double>(arr, arr + size);
        return v;

    }

    vec computeDiscreteDerivatives(vec x, vec y) {

        double y0, y1, x0, x1, k;
        vec ret(x.n_elem);
        for(int i = 1; i < x.n_elem; ++i) {

            y0 = y(i - 1);
            y1 = y(i);
            x0 = x(i - 1);
            x1 = x(i);

            k = (y1 - y0) / (x1 - x0);
            //        cout << (y1 - y0) << " " << (x1 - x0) << endl;
            ret(i - 1) = k;
        }

        ret(ret.n_elem - 1) = k;

        return ret;

    }

    string buildPolynomialEquation(double* w, int paramCount) {
        string ret = "0";
        for(int i = 0; i < paramCount; ++i) {
            std::ostringstream s;
            s << " + " << w[i] << " * x**" << i;
            ret += s.str();
        }
        return ret;
    }

    double* poly_eval_multiple(double* data, int data_len, double* c, int c_len) {
        double* evals = new double[data_len];
        for(int i = 0; i < data_len; ++i) {
            evals[i] = gsl_poly_eval(c, c_len, data[i]);
        }
        return evals;
    }

    double* polyder(double* c, int len) {
        double* dc = new double[len];
        for(int i = 1; i < len; ++i) {
            dc[i - 1] = c[i] * i;
        }
        dc[len - 1] = 0;
        return dc;
    }

    double* polynomialfit(int obs, int degree, double *dx, double *dy) {

        gsl_multifit_linear_workspace *ws;
        gsl_matrix *cov, *X;
        gsl_vector *y, *c;
        double chisq;
        double* store = new double[degree];

        int i, j;

        X = gsl_matrix_alloc(obs, degree);
        y = gsl_vector_alloc(obs);
        c = gsl_vector_alloc(degree);
        cov = gsl_matrix_alloc(degree, degree);

        // computation of design matrix according to bishop equation 3.16 (page 142)
        for(i = 0; i < obs; ++i) {
            gsl_matrix_set(X, i, 0, 1.0);
            for(j=0; j < degree; j++) {
                gsl_matrix_set(X, i, j, pow(dx[i], j));
            }
            gsl_vector_set(y, i, dy[i]);
        }

        ws = gsl_multifit_linear_alloc(obs, degree);
        gsl_multifit_linear(X, y, c, cov, &chisq, ws);

        for(i = 0; i < degree; ++i) {
            store[i] = gsl_vector_get(c, i);
        }

        gsl_multifit_linear_free(ws);
        gsl_matrix_free(X);
        gsl_matrix_free(cov);
        gsl_vector_free(y);
        gsl_vector_free(c);

        return store;

    }

    float* createFloatArrayFromStdVector(vector<float>* data) {
        int size = data->size();
        float* retArr = new float[size];
        for(int i = 0; i < size; ++i) retArr[i] = data->at(i);
        return retArr;
    }

    double* createDoubleArrayFromStdVector(vector<double>* data) {
        int size = data->size();
        double* retArr = new double[size];
        for(int i = 0; i < size; ++i) retArr[i] = data->at(i);
        return retArr;
    }

    double* createDoubleArrayFromArmaVector(vec data) {
        int size = data.n_elem;
        double* retArr = new double[size];
        for(int i = 0; i < size; ++i) retArr[i] = data(i);
        return retArr;
    }

    double* createDoubleArrayFromVector(gsl_vector* data) {
        int size = data->size;
        double* retArr = new double[size];
        for(int i = 0; i < size; ++i) retArr[i] = gsl_vector_get(data, i);
        return retArr;
    }

    /*
     * transforms gsl_matrix to double** matrix
     * returns: double** representation of matrix
     * input:
     *	gsl_matrix* data:	data in gsl_matrix format
     *
    */
    double** createDoubleArrayFromMatrix(gsl_matrix* data) {
        int rows = data->size1;
        int columns = data->size2;
        double** retArr = new double*[rows];
        for(int i = 0; i < rows; ++i) {
            retArr[i] = new double[columns];
            for(int j = 0; j < columns; ++j) {
                retArr[i][j] = gsl_matrix_get(data, i, j);
            }
        }
        return retArr;
    }

    /*
     * releases allocated memory for double array
     * returns: -
     * input:
     * 	double** data:		double array
     * 	int columns:		number of columns in array
     *
    */
    void freeDoubleArray(double** data, int columns) {
        for(int i = 0; i < columns; ++i) {
            free(data[i]);
        }
        free(data);
    }

    void printDoubleVector(double* data, int size) {
        for(int i = 0; i < size; ++i) {
            cout << data[i] << ",";
        }
        cout << endl;
    }

    /*
     * prints a double matrix to console
     * returens: -
     * input:
     * 	double** data:		matrix
     * 	int rows:		number of rows in matrix
     * 	int columns:		number of columns in matrix
    */
    void printDoubleMatrix(double** data, int rows, int columns) {
        for(int i = 0; i < rows; ++i) {
            for(int j = 0; j < columns; ++j) {
                cout << data[i][j] << ",  ";
            }
            cout << endl;
        }
        fflush(stdout);
    }

    gsl_vector* createGslVectorFromStdVector(std::vector<double>* data) {
        gsl_vector* ret = gsl_vector_alloc(data->size());
        for(int i = 0; i < data->size(); ++i) gsl_vector_set(ret, i, data->at(i));
        return ret;
    }

    std::vector<double>* createStdVectorFromGslVector(gsl_vector* vec) {
        std::vector<double>* ret = new std::vector<double>();
        for(int i = 0; i < vec->size; ++i) {
            ret->push_back(gsl_vector_get(vec, i));
        }
        return ret;
    }

    /*
     * converts a queue array representation to the gsl_matrix format
     * returns: gsl_matrix version of data
     * input:
     * 	std:queue<double>** data:	queue array containing data
     * 	int columns:			number of columns (length of data array)
    */
    gsl_matrix* createMatrixFromQueueArray(std::queue<double>** data, int columns) {
        gsl_matrix* retMatrix = gsl_matrix_alloc(data[0]->size(), columns);
        int numRows = data[0]->size();
        for(int i = 0; i < numRows; ++i) {
            for(int j = 0; j < columns; ++j) {
                double val = data[j]->front();
                data[j]->pop();
                gsl_matrix_set(retMatrix, i, j, val);
            }
        }
        return retMatrix;
    }

    gsl_matrix* invertSquareMatrix(gsl_matrix* mat) {

        //	gsl_linalg_SV_decomp(

        int s;
        int size = mat->size1;
        gsl_matrix* retMatrix = gsl_matrix_alloc(size, size);
        gsl_permutation* p = gsl_permutation_alloc(size);
        gsl_linalg_LU_decomp (mat, p, &s);
        gsl_linalg_LU_invert(mat, p, retMatrix);
        return retMatrix;

    }

    /*
     * reads double numbers from file
     *
     * returns: array of queues, where each queue stores a column
     * input:
     *	char* file:		complete path to input file
     *	int fileColumns:	count of columns to import
    */
    mat readMovements(string file) {

        ifstream inFile;
        inFile.open(file.c_str(), ios::in | ios::app | ios::binary);
        mat retMat = readMovements(inFile);
        inFile.close();

        return retMat;

    }

    arma::mat readMovements(std::ifstream& inFile) {

        mat joints;
        string line;
        string token;
        double dn = 0.0;
        double t0 = 0.0;
        double prevTime = DBL_MIN;
        int fileColumns = 0;

        bool firstIteration = true;

        if (inFile.is_open()) {

            int j = 0;
            while(inFile.good()) {

                getline(inFile, line);

                if(line != "") {

                    KukaduTokenizer tok(line);

                    if(firstIteration) {
                        int i = 0;
                        for(i = 0; tok.next() != ""; ++i);
                        fileColumns = i;
                        tok = KukaduTokenizer(line);
                        firstIteration = false;
                        joints = mat(1, fileColumns);
                    }

                    bool ignoreLine = false;

                    for(int i = 0; (token = tok.next()) != "" && i < fileColumns; ++i) {

                        dn = string_to_double(token);

                        if(i == 0 && prevTime == dn) {
                            ignoreLine = true;
                            break;
                        } else if(i == 0)
                            prevTime = dn;

                        // normalization
                        if(j == 0 && i == 0) { t0 = dn; dn = 0; }
                        else if(i == 0) dn -= t0;

                        joints(j, i) = dn;

                    }

                    if(!ignoreLine) {

                        j++;
                        joints.resize(j + 1, fileColumns);

                    }

                }

            }
            joints.resize(j - 1, fileColumns);
        }

        return joints;

    }

    std::pair<std::vector<std::string>, arma::mat> readSensorStorage(std::string file) {

        vector<string> labels;
        string token;
        string line;
        ifstream inFile;
        inFile.open(file.c_str(), ios::in | ios::app | ios::binary);
        getline(inFile, line);

        KukaduTokenizer tok(line);
        while((token = tok.next()) != "")
            labels.push_back(token);

        mat data = readMovements(inFile);
        pair<std::vector<std::string>, arma::mat> retPair(labels, data);

        inFile.close();
        return retPair;

    }

    vec readQuery(string file) {
        vec ret(1);
        string line;
        string token;
        double dn = 0.0;
        int i = 0;
        ifstream inFile;
        inFile.open(file.c_str(), ios::in | ios::app | ios::binary);
        if (inFile.is_open()) {
            cout << string("(utils) query file ") + file + " opened" << endl;
            if(inFile.good()) {
                getline(inFile, line);
                KukaduTokenizer tok(line);
                while((token = tok.next()) != "") {
                    dn = string_to_double(token);
                    ret(i) = dn;
                    ++i;
                    ret.resize(i + 1);
                }
            }
        }
        ret.resize(i);

        inFile.close();
        return ret;
    }

    vector<string> getFilesInDirectory(string folderPath) {

        vector<string> ret;
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (folderPath.c_str())) != NULL) {
            while ((ent = readdir(dir)) != NULL) {
                string tmp(ent->d_name);
                ret.push_back(tmp);
            }

            closedir (dir);
        }
        return ret;
    }

    vector<string> sortPrefix(vector<string> list, string prefix) {

        vector<string> ret;
        int listSize = list.size();

        for(int i = 0; i < listSize; ++i) {
            string currentString = list.at(i);
            int pos = currentString.find(prefix);
            if(!pos) ret.push_back(currentString);
        }
        return ret;
    }

    std::vector<DMPBase> buildDMPBase(vector<double> tmpmys, vector<double> tmpsigmas, double ax, double tau) {

        std::vector<DMPBase> baseDef;
        vector<DMPBase>::iterator it = baseDef.begin();

        vector<double> mys = computeDMPMys(tmpmys, ax, tau);

        for(int i = 0; i < mys.size(); ++i) {

            double realMy = tmpmys.at(i);
            double my = mys.at(i);

            vector<double> sigmas;

            for(int j = 0; j < tmpsigmas.size(); ++j) {
                double realSigma = tmpsigmas.at(j);
                double sigma = my - std::exp( -ax / tau * ( realMy + realSigma ) );
                sigmas.push_back(sigma);
            }

            DMPBase base(my, sigmas);

            // with this implementation, currently all sigmas have to be of the same size (see DMPTrajectoryGenerator::evaluateBasisFunction)
            it = baseDef.insert(it, base);

        }

        return baseDef;

    }

    mat gslToArmadilloMatrix(gsl_matrix* matrix) {
        int lines = matrix->size1;
        int columns = matrix->size2;
        mat ret = mat(lines, columns);
        for(int i = 0; i < lines; ++i)
            for(int j = 0; j < columns; ++j)
                ret(i, j) = gsl_matrix_get(matrix, i, j);
        return ret;
    }

    vector<double> armadilloToStdVec(vec armadilloVec) {
        vector<double> retVec;
        for(int i = 0; i < armadilloVec.n_elem; ++i)
            retVec.push_back(armadilloVec(i));
        return retVec;
    }

    vec stdToArmadilloVec(vector<double> stdVec) {
        vec ret(stdVec.size());
        for(int i = 0; i < stdVec.size(); ++i) {
            ret(i) = stdVec.at(i);
        }
        return ret;
    }

    vector<double>* testGaussianRegressor() {

        vector<double>* ret = new vector<double>[4];
        vector<vec> data;

        vector<double> sampleXs;
        vector<double> sampleYs;
        cout << "producing sample data" << endl;
        /*
        vec ts(20);
        for(double i = 0; i < 1; i = i + 0.05) {
            cout << "\t" << i; fflush(stdout);
            vec point(1);
            point(0) = i;
            data.push_back(point);
            ts(i) = sin(i * 6);

            sampleXs.push_back(i);
            sampleYs.push_back(ts(i));
        }
    */

        /*
        vec x0(1); x0(0) = -1.5; data.push_back(x0);
        vec x1(1); x1(0) = -1.0; data.push_back(x1);
        vec x2(1); x2(0) = -0.75; data.push_back(x2);
        vec x3(1); x3(0) = -0.4; data.push_back(x3);
        vec x4(1); x4(0) = -0.25; data.push_back(x4);
        vec x5(1); x5(0) = 0.0; data.push_back(x5);

        vec ts(6);
        ts(0) = -1.7; ts(1) = -1.2; ts(2) = -0.35; ts(3) = 0.1; ts(4) = 0.5; ts(5) = 0.75;

        sampleXs.push_back(-1.5);
        sampleXs.push_back(-1.0);
        sampleXs.push_back(-0.75);
        sampleXs.push_back(-0.4);
        sampleXs.push_back(-0.25);
        sampleXs.push_back(0.0);

        sampleYs.push_back(-1.7);
        sampleYs.push_back(-1.2);
        sampleYs.push_back(-0.35);
        sampleYs.push_back(0.1);
        sampleYs.push_back(0.5);
        sampleYs.push_back(0.75);
    */
        /*
        vec ts(15);
        int j = 0;
        for(double i = 0.0; i < 3.0; i = i + 0.2) {

            vec dat(1);
            dat(0) = i;
            data.push_back(dat);

            ts(j) = sin(2 * i);

            sampleXs.push_back(i);
            sampleYs.push_back(sin(2 * i));

            j++;
        }
    */
        vec ts(6);
        for(int i = 0; i < 6; ++i) {
            vec dat(1);
            dat(0) = i + 2;
            data.push_back(dat);

            sampleXs.push_back(i + 2);
        }

        ts(0) = -1.75145; sampleYs.push_back(-1.75145);
        ts(1) = -1.74119; sampleYs.push_back(-1.74119);
        ts(2) = -1.73314; sampleYs.push_back(-1.73314);
        ts(3) = -1.72789; sampleYs.push_back(-1.72789);
        ts(4) = -1.73026; sampleYs.push_back(-1.73026);
        ts(5) = -1.72266; sampleYs.push_back(-1.72266);

        vector<double> xs;
        vector<double> ys;
        GaussianProcessRegressor* reg = new GaussianProcessRegressor(data, ts, new GaussianKernel(0.5, 0.05), 10000);
        //	GaussianProcessRegressor* reg = new GaussianProcessRegressor(data, ts, new GaussianKernel(0.5, 1), 10000);
        //	GaussianProcessRegressor* reg = new GaussianProcessRegressor(data, ts, new TricubeKernel(), 1000);
        cout << endl << "do fitting" << endl;
        for(double i = 2.0; i < 7.0; i = i + 0.01) {
            vec query(1);
            query(0) = i;
            double res = reg->fitAtPosition(query)(0);
            xs.push_back(i);
            ys.push_back(res);
        }

        ret[0] = sampleXs;
        ret[1] = sampleYs;
        ret[2] = xs;
        ret[3] = ys;

        return ret;
    }

    arma::vec squareMatrixToColumn(arma::mat Z) {

        vec zCoeffs(Z.n_cols * Z.n_rows);

        int k = 0;
        for(int i = 0; i < Z.n_cols; ++i) {
            for(int j = 0; j < Z.n_rows; ++j) {
                double currVal = Z(i, j);
                zCoeffs(k) = currVal;
                ++k;
            }
        }

        return zCoeffs;

    }

    arma::mat columnToSquareMatrix(arma::vec c) {

        int dim = sqrt(c.n_elem);
        arma::mat newM(dim, dim);

        int k = 0;
        for(int i = 0; i < dim; ++i) {
            for(int j = 0; j < dim; ++j) {
                newM(i, j) = c(k);
                ++k;
            }
        }

        return newM;
    }

    arma::vec symmetricMatrixToColumn(arma::mat Z) {

        vec zCoeffs(Z.n_cols * (Z.n_cols + 1) / 2);

        int k = 0;
        for(int i = 0; i < Z.n_cols; ++i) {
            for(int j = i; j < Z.n_rows; ++j) {
                double currVal = Z(i, j);
                zCoeffs(k) = currVal;
                ++k;
            }
        }

        return zCoeffs;
    }


    arma::mat columnToSymmetricMatrix(arma::vec c) {

        int n = c.n_elem;
        int dim = (sqrt(8 * n + 1) - 1) / 2;
        arma::mat newM(dim, dim);

        int k = 0;
        for(int i = 0; i < dim; ++i) {
            for(int j = i; j < dim; ++j) {
                newM(i, j) = c(k);
                if(i != j)
                    newM(j, i) = c(k);
                ++k;
            }
        }

        return newM;

    }

    std::string stringFromDouble(double d) {

        std::stringstream s;
        s << d;
        return s.str();

    }

    arma::mat fillTrajectoryMatrix(arma::mat joints, double tMax) {

        int prevMaxIdx = joints.n_rows;
        double prevTMax = joints(joints.n_rows - 1, 0);

        if(tMax > prevTMax) {
            double tDiff = prevTMax - joints(joints.n_rows - 2, 0);

            int insertSteps = (int) ((tMax - prevTMax) / tDiff);
            joints.resize(joints.n_rows + insertSteps + 1, joints.n_cols);
            for(int i = 0; i < insertSteps; ++i) {
                for(int j = 1; j < joints.n_cols; ++j) {
                    joints(prevMaxIdx + i, j) = joints(prevMaxIdx - 1, j);
                }
                joints(prevMaxIdx + i, 0) = prevTMax + (i + 1) * tDiff;
            }

            if(joints(prevMaxIdx + insertSteps - 1, 0) != tMax) {
                for(int j = 1; j < joints.n_cols; ++j) {
                    joints(prevMaxIdx + insertSteps, j) = joints(prevMaxIdx - 1, j);
                }
                joints(prevMaxIdx + insertSteps, 0) = tMax;
            } else
                joints.resize(joints.n_rows - 1, joints.n_cols);

        }

        return joints;

    }

    arma::mat armaJoinRows(arma::vec v1, arma::mat m2) {

        if(v1.n_elem != m2.n_rows)
            throw KukaduException("(armaJoinRows) matrix dimensions do not match");

        arma::mat retMat(m2.n_rows, 1 + m2.n_cols);
        for(int i = 0; i < m2.n_rows; ++i) {
            retMat(i, 0) = v1(i);
            for(int j = 0; j < m2.n_cols; ++j)
                retMat(i, 1 + j) = m2(i, j);
        }

        return retMat;

    }

    arma::mat armaJoinRows(arma::mat m1, arma::mat m2) {

        if(m1.n_rows != m2.n_rows)
            throw KukaduException("(armaJoinRows) matrix dimensions do not match");

        arma::mat retMat(m1.n_rows, m1.n_cols + m2.n_cols);
        for(int i = 0; i < m1.n_rows; ++i) {
            for(int j = 0; j < m1.n_cols; ++j)
                retMat(i, j) = m1(i, j);
            for(int j = 0; j < m2.n_cols; ++j)
                retMat(i, m1.n_cols + j) = m2(i, j);
        }

        return retMat;

    }

    double absolute(double val) {
        return (val >= 0) ? val : -val;
    }

    void set_ctrlc_exit_handler() {
        struct sigaction sigIntHandler;
        sigIntHandler.sa_handler = exit_handler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;
        sigaction(SIGINT, &sigIntHandler, NULL);
    }

    void exit_handler(int s) {

        exit(1);

    }


    geometry_msgs::Pose vectorarma2pose(arma::vec* vectorpose) {

        geometry_msgs:: Pose posepose;
        posepose.position.x = vectorpose->at(0);
        posepose.position.y = vectorpose->at(1);
        posepose.position.z = vectorpose->at(2);
        posepose.orientation.x = vectorpose->at(3);
        posepose.orientation.y = vectorpose->at(4);
        posepose.orientation.z = vectorpose->at(5);
        posepose.orientation.w = vectorpose->at(6);

        return posepose;

    }

    arma::vec pose2vectorarma(geometry_msgs::Pose posepose) {

        vec armapose(7);
        armapose(0) = posepose.position.x;
        armapose(1) = posepose.position.y;
        armapose(2) = posepose.position.z;
        armapose(3) = posepose.orientation.x;
        armapose(4) = posepose.orientation.y;
        armapose(5) = posepose.orientation.z;
        armapose(6) = posepose.orientation.w;
        return armapose;

    }

    arma::vec log(tf::Quaternion quat) {

        vec logQuat(3);
        double modU = sqrt(quat.x() * quat.x()  + quat.y() * quat.y() + quat.z() * quat.z());
        double acosw;

        acosw = acos(quat.w());
         if(std::isnan(acosw)) acosw = 0.0;

        if (modU > 0.0) {

            logQuat(0) = acosw * quat.x() / modU;
            logQuat(1) = acosw * quat.y() / modU;
            logQuat(2) = acosw * quat.z() / modU;

        } else {

            logQuat(0) = 0.0;
            logQuat(1) = 0.0;
            logQuat(2) = 0.0;

        }

        return logQuat;

    }

    tf::Quaternion exp(arma::vec logQuat) {

        double x, y, z, w;
        double modR = sqrt(logQuat(0) * logQuat(0) + logQuat(1) * logQuat(1) + logQuat(2) * logQuat(2));
        if (modR > M_PI / 2) cout<< "(utils) mod out of limits "<< modR << endl;

        if (modR > 0) {

            w = cos(modR);
            x = sin(modR) * logQuat(0) / modR;
            y = sin(modR) * logQuat(1) / modR;
            z = sin(modR) * logQuat(2) / modR;

        } else {

            w = 1.0;
            x = 0.0;
            y = 0.0;
            z = 0.0;

        }

        return tf::Quaternion(x, y, z, w);

    }


    double distQuat(tf::Quaternion q1, tf::Quaternion q2) {

        double d;
        const tf::Quaternion q = q1 * q2.inverse();
        vec logQuat = log(q);

        if ((q.x() == 0) && (q.y() == 0) && (q.z() == 0) && (q.w() == -1)) d = 2 * M_PI;

        else {

            d = 2 * sqrt(logQuat(0) * logQuat(0) + logQuat(1) * logQuat(1) + logQuat(2) * logQuat(2));

        }

        return d;
    }

    tf::Transform Matrix4f2Transform(Eigen::Matrix4f Tm) {

        tf::Vector3 origin;
        origin.setValue(static_cast<double>(Tm(0,3)),static_cast<double>(Tm(1,3)),static_cast<double>(Tm(2,3)));

        tf::Matrix3x3 tf3d;
        tf3d.setValue(static_cast<double>(Tm(0,0)), static_cast<double>(Tm(0,1)), static_cast<double>(Tm(0,2)),
                      static_cast<double>(Tm(1,0)), static_cast<double>(Tm(1,1)), static_cast<double>(Tm(1,2)),
                      static_cast<double>(Tm(2,0)), static_cast<double>(Tm(2,1)), static_cast<double>(Tm(2,2)));

        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);

        tf::Transform transform;
        transform.setOrigin(origin);
        transform.setRotation(tfqt);

        return transform;

    }

    /*
     * provides rotation quaterion with angle a around the vector that is defined by xx, yy, zz
     */
    tf::Quaternion axisAngle2Quat(const double &xx, const double &yy, const double &zz, const double &a) {

        double result = sin( a / 2.0 );

        double x = xx * result;
        double y = yy * result;
        double z = zz * result;

        double w = cos( a / 2.0 );

        return tf::Quaternion(x, y, z, w).normalize();

    }

    double distancePoint2Line(double xp,double yp,double x1,double y1,double x2,double y2) {
        return abs((y2 - y1) * xp - (x2 -x1) * yp + x2 * y1 - x1 * y2) / sqrt((y2-y1) * (y2-y1) + (x2 -x1) * (x2 -x1));
    }

    arma::vec pointOnLine2Point(double xp,double yp,double x1,double y1,double x2,double y2) {

        double a = x2 - x1;
        double b = y1 - y2;
        double c = -y1 * a - x1 * b;
        double x = (b * (b * xp - a * yp) - a * c) / sqrt(a * a + b * b);
        double y = (a * (a * yp - b * xp) - b * c) / sqrt(a * a + b * b);

        arma::vec p(2);
        p(0) = x * 2;
        p(1) = y * 2;

        return p;

    }

    bool fileExists(const std::string filePath) {
        struct stat info;
        if(stat(filePath.c_str(), &info) != 0 )
            return false;
        return true;
    }

    bool isDirectory(const std::string dirPath) {
        boost::filesystem::path p = dirPath.c_str();
        return boost::filesystem::is_directory(p);
    }

    void copyFile(const std::string source, const std::string destination) {

        fstream f(source.c_str(), fstream::in|fstream::binary);
        f << noskipws;
        istream_iterator<unsigned char> begin(f);
        istream_iterator<unsigned char> end;

        fstream f2(destination.c_str(), fstream::out | fstream::trunc | fstream::binary);
        ostream_iterator<char> begin2(f2);

        copy(begin, end, begin2);

    }

    void deleteDirectory(std::string path) {
        if(isDirectory(path)) {
            boost::filesystem::path p = path.c_str();
            boost::filesystem::remove_all(p);
        }
    }

    void deleteFile(std::string path) {
        if(!isDirectory(path)) {
            boost::filesystem::path p = path.c_str();
            boost::filesystem::remove_all(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sensorMsgsPcToPclPc(sensor_msgs::PointCloud2::Ptr pc) {
        pcl::PCLPointCloud2 intermediate;
        pcl::PointCloud<pcl::PointXYZ> output;
        pcl::PointCloud<pcl::PointXYZ>::Ptr outputPtr;
        pcl_conversions::toPCL(*pc, intermediate);
        pcl::fromPCLPointCloud2(intermediate, output);
        outputPtr = output.makeShared();
        return outputPtr;
    }

    sensor_msgs::PointCloud2 pclPcToSensorMsgsPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 intermediate;
        pcl::toPCLPointCloud2(*pc, intermediate);
        pcl_conversions::fromPCL(intermediate, output);
        return output;
    }

    sensor_msgs::PointCloud2 pclPcToSensorMsgsPc(pcl::PointCloud<pcl::PointXYZ> pc) {
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 intermediate;
        pcl::toPCLPointCloud2(pc, intermediate);
        pcl_conversions::fromPCL(intermediate, output);
        return output;
    }

    pcl::PCLPointCloud2 sensorMsgsPcToPclPc2(sensor_msgs::PointCloud2 pc) {
        pcl::PCLPointCloud2 tmp;
        pcl_conversions::toPCL(pc, tmp);
        return tmp;
    }

    std::vector<double> createJointsVector(int n_args, ...) {
        va_list ap;
        va_start(ap, n_args);
        vector<double> retVec;
        for(int i = 1; i < (n_args + 1); ++i) {
            double curVal = va_arg(ap, double);
            retVec.push_back(curVal);
        }
        return retVec;
    }

    tf::Quaternion rpyToQuat(const double roll, const double pitch, const double yaw) {
        tf::Quaternion quat(yaw, pitch, roll);
        return quat;
    }

    long getFileSize(std::string filename) {
        struct stat stat_buf;
        int rc = stat(filename.c_str(), &stat_buf);
        return rc == 0 ? stat_buf.st_size : -1;
    }

    double roundByDigits(double number, int numDigitsBehindComma) {
        return ceilf(number * pow(10.0, (double) numDigitsBehindComma)) / 100;
    }

    std::wstring stringToWString(const std::string& s) {

        std::wstring wsTmp(s.begin(), s.end());
        return wsTmp;

    }

}
