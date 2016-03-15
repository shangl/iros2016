#include "sensingcontroller.hpp"

#include <Python.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace pf = boost::filesystem;

namespace kukadu {

    SensingController::SensingController(KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, int hapticMode, string caption, std::string databasePath, std::vector<KUKADU_SHARED_PTR<ControlQueue> > queues, vector<KUKADU_SHARED_PTR<GenericHand> > hands, std::string tmpPath, std::string classifierPath, std::string classifierFile, std::string classifierFunction, int simClassificationPrecision) : Controller(caption, 1) {

        currentIterationNum = 0;
        classifierParamsSet = false;
        simulationGroundTruth = 0;
        simulatedClassificationPrecision = simClassificationPrecision;

        this->generator = generator;

        this->hands = hands;
        this->queues = queues;
        this->tmpPath = tmpPath;
        this->hapticMode = hapticMode;
        this->databasePath = databasePath;
        this->classifierFile = classifierFile;
        this->classifierPath = classifierPath;
        this->classifierFunction = classifierFunction;

        bestParamC = 0.0;
        bestParamD = 0.0;
        bestParamParam1 = 0.0;
        bestParamParam2 = 0.0;

        Py_Initialize();

    }

    void SensingController::gatherData(std::string dataBasePath, std::string dataName) {

        if(!fileExists(dataBasePath))
            createDirectory(dataBasePath);

        gatherData(dataBasePath + dataName);

    }

    std::string SensingController::getDatabasePath() {
        return databasePath;
    }

    void SensingController::gatherData(std::string completePath) {

        vector<KUKADU_SHARED_PTR<ControlQueue> > castedQueues;
        for(int i = 0; i < queues.size(); ++i) {
            KUKADU_SHARED_PTR<ControlQueue> queue = queues.at(i);
            castedQueues.push_back(queue);
        }

        SensorStorage store(castedQueues, hands, 100);

        prepare();

        KUKADU_SHARED_PTR<kukadu_thread> storageThread = store.startDataStorage(completePath);

        performCore();

        store.stopDataStorage();
        storageThread->join();

        cleanUp();

    }

    std::string SensingController::getFirstRobotFileName() {
        return queues.at(0)->getRobotFileName();
    }

    std::vector<double> SensingController::callClassifier() {
        return callClassifier(databasePath, tmpPath + "hapticTest/" + queues.at(0)->getRobotFileName() + "_0", true, bestParamC, bestParamD, bestParamParam1, bestParamParam2);
    }

    int SensingController::performClassification() {

        int classifierRes = -1;
        if(!getSimulationMode()) {

            int executeIt = 0;
            int temporaryHapticMode = hapticMode;
            cout << "(SensingController) selected sensing action is \"" << getCaption() << "\"; want to execute it? (0 = no / 1 = yes)" << endl;
            cin >> executeIt;

            if(executeIt == 1) {

                if(!classifierParamsSet) {
                    string errorMsg = "(SensingController) classifier parameters not yet set" ;
                    cerr << errorMsg << endl;
                    throw KukaduException(errorMsg.c_str());
                }

                pf::remove_all(tmpPath + "hapticTest");

                gatherData(tmpPath, "hapticTest");
                stringstream s;
                s << tmpPath << "hapticTest_" << queues.at(0)->getRobotFileName() << "_0_" << currentIterationNum;
                copyFile(tmpPath + "hapticTest/" + queues.at(0)->getRobotFileName() + "_0", s.str());

            } else {
                if(!isShutUp) {
                    cout << "(ControllerActionClip) you decided not to perform the action" << endl;
                    cout << "(ControllerActionClip) switching temporarily to haptic mode HAPTIC_MODE_TERMINAL; continue" << endl;
                }
                temporaryHapticMode = SensingController::HAPTIC_MODE_TERMINAL;
            }

            if(temporaryHapticMode == SensingController::HAPTIC_MODE_TERMINAL) {
                cout << "(SensingController) what was the haptic result? [0, " << (getSensingCatCount() - 1) << "]" << endl;
                cin >> classifierRes;
            } else if(temporaryHapticMode == SensingController::HAPTIC_MODE_CLASSIFIER) {
                vector<double> res = callClassifier(databasePath, tmpPath + "hapticTest/" + queues.at(0)->getRobotFileName() + "_0", true, bestParamC, bestParamD, bestParamParam1, bestParamParam2);
                int maxIdx = 0;
                double maxElement = res.at(0);
                for(int i = 1; i < getSensingCatCount(); ++i) {
                    if(res.at(i) > maxElement) {
                        maxElement = res.at(i);
                        maxIdx = i;
                    }
                }
                classifierRes = maxIdx;
            } else {
                throw KukaduException("haptic mode not known");
            }

            if(!isShutUp)
                cout << "(main) classifier result is category " << classifierRes << endl << "(main) press enter to continue" << endl;
            getchar();

            pf::remove_all(tmpPath + "hapticTest");
            ++currentIterationNum;


        } else {

            vector<double> precisionProbVec;
            precisionProbVec.push_back((double) simulatedClassificationPrecision);
            precisionProbVec.push_back((double) (100 - simulatedClassificationPrecision));
            KUKADU_DISCRETE_DISTRIBUTION<int> precisionProb(precisionProbVec.begin(), precisionProbVec.end());

            int correctClass = precisionProb(*generator);

            // simulate correct classification
            if(!correctClass)
                classifierRes = simulationGroundTruth;
            else {
                // classify it wrongly (random)
                classifierRes = simulationGroundTruth;
                while(classifierRes == simulationGroundTruth)
                    classifierRes = createRandomGroundTruthIdx();
            }

        }

        return classifierRes;

    }

    int SensingController::createRandomGroundTruthIdx() {
        vector<int> randValues;
        for(int i = 0; i < getSensingCatCount(); ++i)
            randValues.push_back(1);
        classifierDist = KUKADU_DISCRETE_DISTRIBUTION<int>(randValues.begin(),randValues.end());
        return classifierDist(*generator);
    }

    void SensingController::setSimulationClassificationPrecision(int percent) {
        simulatedClassificationPrecision = percent;
    }

    void SensingController::setSimulationGroundTruth(int idx) {
        simulationGroundTruth = idx;
    }

    double SensingController::createDataBase() {

        int numClasses = 0;
        string path = getDatabasePath();
        vector<pair<int, string> > collectedSamples;
        if(!isShutUp)
            cout << "(SensingController) data is stored to " << path << endl;
        if(!fileExists(path)) {

            if(!isShutUp)
                cout << "(SensingController) folder doesn't exist - create" << endl;
            createDirectory(path);

            // create the database
            numClasses = getSensingCatCount();
            if(!isShutUp)
                cout << "(SensingController) " << getCaption() << " offers " << numClasses << " classes" << endl;

            for(int currClass = 0; currClass < numClasses; ++currClass) {

                if(currClass != 0)
                    this->prepareNextState();

                int cont = 1;
                for(int sampleNum = 0; cont == 1; ++sampleNum) {

                    cout << "(SensingController) press key to collect sample number " << sampleNum << " for class " << currClass << endl;
                    getchar();

                    stringstream s;
                    s << "class_" << currClass << "_sample_" << sampleNum;
                    string relativePath = s.str();
                    string relativeClassifyPath = relativePath + "/" + getFirstRobotFileName() + "_0";
                    string nextSamplePath = path + relativePath;
                    gatherData(nextSamplePath);

                    collectedSamples.push_back(pair<int, string>(currClass, relativeClassifyPath));

                    cout << "(SensingController) want to collect another sample for class " << currClass << "? (0 = no / 1 = yes): ";
                    cin >> cont;

                }

            }

            writeLabelFile(path, collectedSamples);

        } else {
            if(!isShutUp)
                cout << "(SensingController) database for controller " << getCaption() << " exists - no collection required" << endl;
        }

        // if no classifier file exists
        if(!fileExists(path + "classRes")) {

            // determine confidence value on database
            vector<double> classRes = callClassifier(path, "", false, 0.0, 0.0, 0.0, 0.0);
            for(double res : classRes)
                cout << res << endl;

            cerr << "(SensingController) this part currently wont work (switched back to old classifier (repair later))" << endl;
            double confidence = classRes.at(classRes.size() - 1);
            /*
            double bestParamC = classRes.at(classRes.size() - 4);
            double bestParamD = classRes.at(classRes.size() - 3);
            double bestParamPar1 = classRes.at(classRes.size() - 2);
            double bestParamPar2 = classRes.at(classRes.size() - 1);
            */

            double bestParamC = 0.0;
            double bestParamD = 0.0;
            double bestParamPar1 = 0.0;
            double bestParamPar2 = 0.0;

            ofstream ofile;
            ofile.open((path + "classRes").c_str());
            ofile << confidence << "\t" << bestParamC << "\t" << bestParamD << "\t" << bestParamPar1 << "\t" << bestParamPar2 << endl;
            ofile.close();

        }

        ifstream infile;
        infile.open((path + "classRes").c_str());
        double confidence = 0.0;
        double bestParamC = 0.0;
        double bestParamD = 0.0;
        double bestParamPar1 = 0.0;
        double bestParamPar2 = 0.0;
        infile >> confidence >> bestParamC >> bestParamD >> bestParamPar1 >> bestParamPar2;

        setCLassifierParams(bestParamC, bestParamD, bestParamParam1, bestParamParam2);

        if(!isShutUp)
            cout << "(SensingController) determined a confidence of " << confidence << endl;

        return confidence;

    }

    KUKADU_SHARED_PTR<ControllerResult> SensingController::performAction() {

        prepare();
        performCore();
        cleanUp();

        return KUKADU_SHARED_PTR<ControllerResult>();

    }

    void SensingController::setCLassifierParams(double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2) {
        classifierParamsSet = true;
        this->bestParamC = bestParamC;
        this->bestParamD = bestParamD;
        this->bestParamParam1 = bestParamParam1;
        this->bestParamParam2 = bestParamParam2;
    }

    std::vector<double> SensingController::callClassifier(std::string trainedPath, std::string passedFilePath, bool classify, double bestParamC, double bestParamD, double bestParamParam1, double bestParamParam2) {

        vector<double> retVals;
        string mName = classifierFile;
        string fName = classifierFunction;
        string argumentVal = trainedPath;

        PyObject *pName, *pModule, *pFunc;
        PyObject *pArgs, *pValue;

        PyRun_SimpleString("import sys");
        PyRun_SimpleString(string(string("sys.path.append('") + classifierPath + string("')")).c_str());
        PyRun_SimpleString("import trajlab_main");

        pName = PyUnicode_FromString(mName.c_str());
        pModule = PyImport_Import(pName);
        Py_DECREF(pName);

        if (pModule != NULL) {

            pFunc = PyObject_GetAttrString(pModule, fName.c_str());

            if (pFunc && PyCallable_Check(pFunc)) {

                //pArgs = PyTuple_New(7);
                pArgs = PyTuple_New(3);
                pValue = PyUnicode_FromString(argumentVal.c_str());

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");
                    return retVals;

                }

                PyTuple_SetItem(pArgs, 0, pValue);

                pValue = PyUnicode_FromString(passedFilePath.c_str());

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");

                }

                PyTuple_SetItem(pArgs, 1, pValue);

                pValue = PyFloat_FromDouble((classify) ? 1.0 : -1.0);

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");

                }

                PyTuple_SetItem(pArgs, 2, pValue);

                pValue = PyFloat_FromDouble(bestParamC);

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");

                }
/*
                PyTuple_SetItem(pArgs, 3, pValue);

                pValue = PyFloat_FromDouble(bestParamD);

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");

                }

                PyTuple_SetItem(pArgs, 4, pValue);

                pValue = PyFloat_FromDouble(bestParamParam1);

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");

                }

                PyTuple_SetItem(pArgs, 5, pValue);

                pValue = PyFloat_FromDouble(bestParamParam2);

                if (!pValue) {

                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");

                }

                PyTuple_SetItem(pArgs, 6, pValue);
*/
                pValue = PyObject_CallObject(pFunc, pArgs);
                Py_DECREF(pArgs);

                if (pValue != NULL) {

                    int count = (int) PyList_Size(pValue);
                    for(int i = 0; i < count; ++i) {
                        PyObject* ptemp = PyList_GetItem(pValue, i);
                        retVals.push_back(PyFloat_AsDouble(ptemp));
                    }

                    // retVal = PyLong_AsLong(pValue);
                    Py_DECREF(pValue);

                } else {

                    Py_DECREF(pFunc);
                    Py_DECREF(pModule);
                    PyErr_Print();
                    fprintf(stderr,"Call failed\n");

                }

            } else {

                if (PyErr_Occurred())
                    PyErr_Print();
                cerr << "Cannot find function " << fName << endl;

            }

            Py_XDECREF(pFunc);
            Py_DECREF(pModule);

        }
        else {

            PyErr_Print();
            cerr << "Failed to load " << mName << endl;

        }

        return retVals;

    }

    void SensingController::writeLabelFile(std::string baseFolderPath, std::vector<std::pair<int, std::string> > collectedSamples) {

        ofstream outFile;
        outFile.open((baseFolderPath + "labels").c_str());

        for(int i = 0; i < collectedSamples.size(); ++i) {
            pair<int, string> sample = collectedSamples.at(i);
            outFile << sample.second << " " << sample.first << endl;
        }

    }

}
