#ifndef KUKADU_TOGERSON_H
#define KUKADU_TOGERSON_H

#include <set>
#include <math.h>
#include <vector>
#include <armadillo>

#include "mahalanobis.hpp"
#include "mahalanobicslearner.hpp"
#include "../../utils/customset.hpp"
#include "../../types/kukadutypes.hpp"

namespace kukadu {

    struct armacomp {

        int operator() (const arma::vec& vec1, const arma::vec& vec2) const {

            if(vec1.n_elem != vec2.n_elem)
                return vec1.n_elem - vec2.n_elem;

            // check equality
            for(int i = 0; i < vec1.n_elem; ++i) {
                if(vec1(i) == vec2(i)) {

                } else if (vec1(i) > vec2(i)) {
                    std::cout << vec1.t() << " " << vec2.t() << "===============" << 1 << "==============" << std::endl;
                    return 1;
                } else {
                    std::cout << vec1.t() << " " << vec2.t() << "===============" << -1 << "==============" << std::endl;
                    return -1;
                }
            }

            std::cout << vec1.t() << " " << vec2.t() << "===============" << 0 << "==============" << std::endl;

            return 0;

        }

    };

    // method according to http://forrest.psych.unc.edu/teaching/p230/Torgerson.pdf
    class TogersonMetricLearner : public MahalanobisLearner {

    private:

        int dim;
        int sampleCount;

        arma::mat D;

        CustomSet xsSet;

        std::vector<arma::vec> expandedX1s;
        std::vector<arma::vec> expandedX2s;
        std::vector<double> expandedDistances;

        void generateD();
        void expandConstraints();

        int selectI();
        int compareArmadilloVec(arma::vec vec1, arma::vec vec2);

        std::vector<int> getDRowIdxs(arma::vec x);

        arma::mat generateA();
        arma::mat generateX();
        arma::mat generateB(int iIdx);
        arma::mat generateY(arma::mat B);
        arma::mat generateZ(arma::mat X, arma::mat Y);

    public:

        TogersonMetricLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances);

        Mahalanobis learnMetric();

    };

}

#endif
