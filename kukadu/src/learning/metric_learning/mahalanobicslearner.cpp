#include "mahalanobicslearner.hpp"

#include "../../types/kukadutypes.hpp"

using namespace arma;
using namespace std;

namespace kukadu {

    MahalanobisLearner::MahalanobisLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances) : metric(x1s.at(0).n_elem) {

        int vecSize = x1s.size();

        if(x1s.size() != x2s.size() || distances.size() != vecSize) {
            string error = "(MahalanobisLearner) sizes of x vector collections do not match";
            cerr << error << endl;
            throw KukaduException(error.c_str());
        }

        int xDimension = x1s.at(0).n_elem;
        for(int i = 1; i < x1s.size(); ++i) {
            if(x1s.at(i).n_elem != xDimension || x2s.at(i).n_elem != xDimension) {
                string error = "(MahalanobisLearner) sizes of x vectors do not match";
                cerr << error << endl;
                throw KukaduException(error.c_str());
            }
        }

        this->x1s = x1s;
        this->x2s = x2s;
        this->distances = distances;

    }

    void MahalanobisLearner::addSample(arma::vec x1, arma::vec x2, double distance) {

        if(x1s.size() > 0 && x1s.at(0).n_elem == x1.n_elem && x2s.at(0).n_elem == x2.n_elem) {
            string error = "(MahalanobisLearner) size of x vector does not match";
            cerr << error << endl;
            throw KukaduException(error.c_str());
        }

        distances.push_back(distance);

        x1s.push_back(x1);
        x2s.push_back(x2);

    }

    Mahalanobis MahalanobisLearner::getMetric() {
        return metric;
    }

    int MahalanobisLearner::getVectorDim() {
        return x1s.at(0).n_elem;
    }

    int MahalanobisLearner::getSampleCount() {
        return x1s.size();
    }

    double MahalanobisLearner::getSampleDistance(int idx) {
        return distances.at(idx);
    }

    arma::vec MahalanobisLearner::getX1(int idx) {
        return x1s.at(idx);
    }

    arma::vec MahalanobisLearner::getX2(int idx) {
        return x2s.at(idx);
    }

}
