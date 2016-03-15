#ifndef KUKADU_DMPBASESET
#define KUKADU_DMPBASESET

#include "../utils/conversion_utils.hpp"

#include <vector>
#include <armadillo>

namespace kukadu {

    class DMPBase {

    private:

        float my;
        std::vector<double> sigmas;

    public:

        DMPBase();
        DMPBase(float my, std::vector<double> sigmas);

        float getMy();
        std::vector<double> getSigmas();

        int operator==(DMPBase const& comp) const;

    };

}

#endif
