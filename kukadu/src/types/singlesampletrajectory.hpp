#ifndef KUKADU_SINGLESAMPLETRAJECTORY
#define KUKADU_SINGLESAMPLETRAJECTORY

#include <vector>
#include <armadillo>

#include "trajectory.hpp"
#include "../utils/types.hpp"
#include "../utils/conversion_utils.hpp"

namespace kukadu {

    class SingleSampleTrajectory : public Trajectory {

    private:

    protected:

        std::vector<arma::vec> sampleYs;
        arma::vec supervisedTs;

    public:

        SingleSampleTrajectory(arma::vec supervisedTs, std::vector<arma::vec> sampleYs);
        SingleSampleTrajectory(const SingleSampleTrajectory& copy);
        SingleSampleTrajectory();

        int getDegreesOfFreedom() const;
        int getDataPointsNum();

        double getT(int ptIdx);
        double getDataPoint(int freedomIdx, int ptIdx);

        void setSupervisedTs(arma::vec supervisedTs);
        void setSampleYs(std::vector<arma::vec> sampleYs);

        arma::vec getStartingPos();
        arma::vec getSupervisedTs();
        arma::vec getSampleYByIndex(int idx);
        std::vector<arma::vec> getSampleYs();

        virtual std::vector<arma::vec> getCoefficients() = 0;
        virtual void setCoefficients(std::vector<arma::vec> coeffs) = 0;

        int operator==(SingleSampleTrajectory const& comp) const;

    };

}

#endif
