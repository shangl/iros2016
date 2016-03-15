#include "dmp.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    Dmp::Dmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
            double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr)
                    : SingleSampleTrajectory(supervisedTs, sampleYs) {

        construct(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax, ac, dmpStepSize, tolAbsErr, tolRelErr);

    }

    Dmp::Dmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
            double tau, double az, double bz, double ax) : SingleSampleTrajectory(supervisedTs, sampleYs) {

        construct(supervisedTs, sampleYs, fitYs, dmpCoeffs, dmpBase, designMatrices, tau, az, bz, ax, 10.0, 1.8 * 1e4 * 1e-6, 0.1, 0.1);

    }

    Dmp::Dmp(std::string dmpFile) {

        std::ifstream dmpFileStream(dmpFile.c_str(), std::ifstream::in);
        mat m = readMat(dmpFileStream);
        int degreesOfFreedom = m(0,0);

        m = readMat(dmpFileStream);
        int baseTermCount = m(0,0);

        m = readMat(dmpFileStream);
        vec fileSuperVisedTs = m.col(0);

        vector<vec> fileSampleYs;
        for(int i = 0; i < degreesOfFreedom; ++i) {
            vec fileCurrentSampleY = readMat(dmpFileStream).col(0);
            fileSampleYs.push_back(fileCurrentSampleY);
        }

        vector<vec> fileFitYs;
        for(int i = 0; i < degreesOfFreedom; ++i) {
            vec fileCurrentFitY = readMat(dmpFileStream).col(0);
            fileFitYs.push_back(fileCurrentFitY);
        }

        vector<vec> fileDmpCoeffs;
        for(int i = 0; i < degreesOfFreedom; ++i) {
            vec fileCurrentCoeffs = readMat(dmpFileStream).col(0);
            fileDmpCoeffs.push_back(fileCurrentCoeffs);
        }

        vector<DMPBase> fileDmpBases;
        for(int i = 0; i < baseTermCount; ++i) {

            mat fileCurrentMyMat = readMat(dmpFileStream);
            double fileCurrentMy = fileCurrentMyMat(0,0);
            vec fileCurrentSigmasVec = readMat(dmpFileStream).col(0);
            vector<double> fileCurrentSigmas;
            for(int j = 0; j < fileCurrentSigmasVec.n_elem; ++j)
                fileCurrentSigmas.push_back(fileCurrentSigmasVec(j));
            DMPBase currentBase(fileCurrentMy, fileCurrentSigmas);
            fileDmpBases.push_back(currentBase);

        }

        vector<mat> fileDesignMatrices;
        for(int i = 0; i < degreesOfFreedom; ++i) {
            mat fileCurrentDesignMatrix = readMat(dmpFileStream);
            fileDesignMatrices.push_back(fileCurrentDesignMatrix);
        }

        vec fileDmpConsts = readMat(dmpFileStream).row(0).t();
        this->setSupervisedTs(fileSuperVisedTs);
        this->setSampleYs(fileSampleYs);

        construct(fileSuperVisedTs, fileSampleYs, fileFitYs, fileDmpCoeffs, fileDmpBases, fileDesignMatrices,
                  fileDmpConsts(0), fileDmpConsts(1), fileDmpConsts(2), fileDmpConsts(3), fileDmpConsts(4),
                  fileDmpConsts(5), fileDmpConsts(6), fileDmpConsts(7));

        cout << "Dmp loaded from file" << endl;

    }

    void Dmp::serialize(string dmpFile) {

        string baseString = "";

        int i = 0;
        for(i = 0; i < getDmpBase().size(); ++i) {

            // serialize base
            DMPBase currentBase = getDmpBase().at(i);
            double currentMy = currentBase.getMy();

            vector<double> currentSigmas = currentBase.getSigmas();
            baseString += double_to_string(currentMy) + "\n\n" + "=" + "\n";
            for(int j = 0; j < currentSigmas.size(); ++j)
                baseString += double_to_string(currentSigmas.at(j)) + "\n\n";
            baseString = baseString + "=" + "\n";

        }

        ofstream dmpFileStream;
        dmpFileStream.open(dmpFile.c_str());

        dmpFileStream << getDegreesOfFreedom() << endl;
        dmpFileStream << "=" << endl;
        dmpFileStream << i << endl << getDmpBase().at(0).getSigmas().size() << endl;
        dmpFileStream << "=" << endl;

        dmpFileStream << getSupervisedTs() << endl;
        dmpFileStream << "=" << endl;

        for(int i = 0; i < getSampleYs().size(); ++i) {
            dmpFileStream << getSampleYByIndex(i) << endl;
            dmpFileStream << "=" << endl;
        }

        for(int i = 0; i < getFitYs().size(); ++i) {
            dmpFileStream << getFitYs().at(i) << endl;
            dmpFileStream << "=" << endl;
        }

        for(int i = 0; i < getDmpCoeffs().size(); ++i) {
            dmpFileStream << getDmpCoeffs().at(i) << endl;
            dmpFileStream << "=" << endl;
        }

       dmpFileStream << baseString;

        for(int i = 0; i < getDesignMatrixCount(); ++i) {
            dmpFileStream << getDesignMatrix(i) << endl;
            dmpFileStream << "=" << endl;
        }

        dmpFileStream << tau << " " << az << " " << bz << " " << ax << " " << ac << " " << dmpStepSize << " " << tolAbsErr << " " << tolRelErr << endl;

    }

    int Dmp::getSampleCount() {
        return sampleYs.at(0).size();
    }

    double Dmp::getDeltaTByIdx(int idx) {
        return (supervisedTs(idx + 1) - supervisedTs(idx));
    }

    void Dmp::construct(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
            double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr) {

        this->dmpCoeffs = dmpCoeffs;
        this->dmpBase = dmpBase;
        this->designMatrices = designMatrices;
        this->tau = tau;
        this->az = az;
        this->bz = bz;
        this->ax = ax;
        this->fitYs = fitYs;

        this->ac = ac;
        this->dmpStepSize = dmpStepSize;
        this->tolAbsErr = tolAbsErr;
        this->tolRelErr = tolRelErr;

        initializeY0();
        initializeDy0();
        initializeDdy0();
        initializeG();

        int s = getDataPointsNum();
        if(s > 0)
            tmax = getT(s - 1);
        else
            tmax = 0.0;

    }

    void Dmp::setDmpCoeffs(std::vector<arma::vec> coeffs) {
        dmpCoeffs = coeffs;
    }

    Dmp::Dmp(const Dmp& copy) : SingleSampleTrajectory(copy) {

        this->dmpCoeffs = copy.dmpCoeffs;
        this->dmpBase = copy.dmpBase;
        this->designMatrices = copy.designMatrices;
        this->tau = copy.tau;
        this->az = copy.az;
        this->bz = copy.bz;
        this->ax = copy.ax;
        this->dmpStepSize = copy.dmpStepSize;
        this->ac = copy.ac;
        this->tolAbsErr = copy.tolAbsErr;
        this->tolRelErr = copy.tolRelErr;

        initializeY0();
        initializeDy0();
        initializeDdy0();
        initializeG();

        this->tmax = copy.tmax;

    }

    Dmp::Dmp() : SingleSampleTrajectory(vec(), vector<vec>()) {

    }

    std::vector<arma::vec> Dmp::getFitYs() {
        return fitYs;
    }

    arma::vec Dmp::getY0() {
        return y0;
    }

    arma::vec Dmp::getDy0() {
        return dy0;
    }

    arma::vec Dmp::getDdy0() {
        return ddy0;
    }

    arma::vec Dmp::getG() {
        return g;
    }

    int Dmp::getDesignMatrixCount() {
        return designMatrices.size();
    }

    arma::mat Dmp::getDesignMatrix(int freedomIdx) {
        return designMatrices.at(freedomIdx);
    }

    std::vector<arma::vec> Dmp::getDmpCoeffs() {
        return dmpCoeffs;
    }

    std::vector<arma::vec> Dmp::getCoefficients() {
        return getDmpCoeffs();
    }

    double Dmp::getY0(int freedomIdx) {
        return y0(freedomIdx);
    }

    double Dmp::getDy0(int freedomIdx) {
        return dy0(freedomIdx);
    }

    double Dmp::getDdy0(int freedomIdx) {
        return ddy0(freedomIdx);
    }

    double Dmp::getG(int freedomIdx) {
        return g(freedomIdx);
    }

    arma::vec Dmp::getDmpCoeffs(int freedomIdx) {
        return dmpCoeffs.at(freedomIdx);
    }

    void Dmp::setCoefficients(std::vector<arma::vec> coeffs) {
        setDmpCoeffs(coeffs);
    }

    std::vector<DMPBase> Dmp::getDmpBase() {
        return dmpBase;
    }

    double Dmp::getTau() {
        return tau;
    }

    double Dmp::getAz() {
        return az;
    }

    double Dmp::getBz() {
        return bz;
    }

    double Dmp::getAx() {
        return ax;
    }

    double Dmp::getStepSize() {
        return dmpStepSize;
    }

    double Dmp::getTolAbsErr() {
        return tolAbsErr;
    }

    double Dmp::getTolRelErr() {
        return tolRelErr;
    }

    double Dmp::getTmax() {

        return tmax;

    }

    void Dmp::setTmax(double tmax) {
        this->tmax = tmax;
    }

    void Dmp::initializeY0() {

        int degOfFreemdom = getDegreesOfFreedom();

        y0 = vec(degOfFreemdom);
        for(int i = 0; i < degOfFreemdom; ++i)
            y0(i) = getDataPoint(i, 0);

    }

    void Dmp::initializeDy0() {

        int degOfFreemdom = getDegreesOfFreedom();
        dy0 = vec(degOfFreemdom);
        for(int i = 0; i < degOfFreemdom; ++i)
            dy0(i) = ( getDataPoint(i, 1) - getDataPoint(i, 0) ) / ( getT(1) - getT(0) );

    }

    void Dmp::initializeDdy0() {

        int degOfFreemdom = getDegreesOfFreedom();
        ddy0 = vec(degOfFreemdom);
        for(int i = 0; i < degOfFreemdom; ++i) {
            double dy = ( getDataPoint(i, 2) - getDataPoint(i, 1) ) / ( getT(2) - getT(1) );
            ddy0(i) = ( dy - dy0(i) ) / ( getT(2) - getT(0) );
        }

    }

    void Dmp::initializeG() {

        int s = getDataPointsNum();
        int degOfFreemdom = getDegreesOfFreedom();
        g = vec(degOfFreemdom);
        for(int i = 0; i < degOfFreemdom; ++i)
            g(i) = getDataPoint(i, s - 1);

    }

    int Dmp::operator==(KUKADU_SHARED_PTR<Dmp> const& comp) const {

        // design matrices are ignored here
        return (compareVectorOfArmadillos(dmpCoeffs, comp->dmpCoeffs) && compareArmadilloVec(y0, comp->y0) && compareArmadilloVec(dy0, comp->dy0) &&
                    compareArmadilloVec(ddy0, comp->ddy0) && compareArmadilloVec(g, comp->g) && dmpBase == comp->dmpBase &&
                    tau == comp->tau && az == comp->az && bz == comp->bz && ax == comp->ax );

    }

}
