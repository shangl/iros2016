#ifndef KUKADU_PLANARCUTTRANSFORMATOR_H
#define KUKADU_PLANARCUTTRANSFORMATOR_H

#include <armadillo>

#include "pcltransformator.hpp"

namespace kukadu {

    class PlanarCutTransformator : public PCTransformator {

    private:

        arma::vec normalVec;
        arma::vec plainOriginVec;

    public:

        PlanarCutTransformator(arma::vec normalVec, arma::vec plainOriginVec);

        virtual pcl::PointCloud<pcl::PointXYZ>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

        void setPlane(arma::vec normalVec, arma::vec plainOriginalVec);

    };

}

#endif // PLANARCUTTRANSFORMATOR
