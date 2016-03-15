#ifndef KUKADU_OPENBOXFILTER_H
#define KUKADU_OPENBOXFILTER_H

#include <armadillo>

#include "pcltransformator.hpp"

namespace kukadu {

    class OpenBoxFilter : PCTransformator {

    private:

        double xOffset;
        double yOffset;

        arma::vec center;

    public:

        OpenBoxFilter(arma::vec center, double xOffset, double yOffset);

        virtual pcl::PointCloud<pcl::PointXYZ>::Ptr transformPc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        void setBox(arma::vec center, double xOffset, double yOffset);

    };

}

#endif
