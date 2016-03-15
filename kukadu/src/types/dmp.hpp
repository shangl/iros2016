#ifndef KUKADU_DMP
#define KUKADU_DMP

#include <vector>
#include <armadillo>

#include "dmpbase.hpp"
#include "singlesampletrajectory.hpp"

#include "../types/kukadutypes.hpp"
#include "../utils/conversion_utils.hpp"

namespace kukadu {

    class Dmp : public SingleSampleTrajectory {

    private:

        std::vector<arma::vec> dmpCoeffs;
        arma::vec y0;
        arma::vec dy0;
        arma::vec ddy0;
        arma::vec g;

        std::vector<DMPBase> dmpBase;
        std::vector<arma::mat> designMatrices;

        double tau;
        double az;
        double bz;
        double ax;
        double ac;
        double tolAbsErr;
        double tolRelErr;
        double tmax;

        void initializeY0();
        void initializeDy0();
        void initializeDdy0();
        void initializeG();

        void construct(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
            double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

    protected:
        double dmpStepSize;

        std::vector<arma::vec> fitYs;

    public:

        Dmp();
        Dmp(const Dmp& copy);
        Dmp(std::string dmpFile);
        Dmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
            double tau, double az, double bz, double ax, double ac, double dmpStepSize, double tolAbsErr, double tolRelErr);

        Dmp(arma::vec supervisedTs, std::vector<arma::vec> sampleYs, std::vector<arma::vec> fitYs, std::vector<arma::vec> dmpCoeffs, std::vector<DMPBase> dmpBase, std::vector<arma::mat> designMatrices,
            double tau, double az, double bz, double ax);

        void serialize(std::string dmpFile);

        int getSampleCount();

        double getY0(int freedomIdx);
        double getDy0(int freedomIdx);
        double getDdy0(int freedomIdx);
        double getG(int freedomIdx);

        arma::vec getY0();
        arma::vec getDy0();
        arma::vec getDdy0();
        arma::vec getG();

        std::vector<arma::vec> getCoefficients();
        void setCoefficients(std::vector<arma::vec> coeffs);

        std::vector<arma::vec> getDmpCoeffs();
        std::vector<arma::vec> getFitYs();

        void setDmpCoeffs(std::vector<arma::vec> coeffs);

        arma::vec getDmpCoeffs(int freedomIdx);
        arma::mat getDesignMatrix(int freedomIdx);
        int getDesignMatrixCount();
        std::vector<DMPBase> getDmpBase();

        double getDeltaTByIdx(int idx);

        double getTau();
        double getAz();
        double getBz();
        double getAx();

        double getStepSize();
        double getTolAbsErr();
        double getTolRelErr();

        double getTmax();
        void setTmax(double tmax);

        virtual bool isCartesian() = 0;

        int operator==(KUKADU_SHARED_PTR<Dmp> const& comp) const;

    };

}

#endif
