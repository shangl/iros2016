#include "togersonmetriclearner.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    TogersonMetricLearner::TogersonMetricLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances) : MahalanobisLearner(x1s, x2s, distances) {
    }

    Mahalanobis TogersonMetricLearner::learnMetric() {

        dim = getVectorDim();
        sampleCount = getSampleCount();


        int iIdx = selectI();

        mat B = generateB(iIdx);
        mat X = generateX();
        mat Y = generateY(B);
        mat Z = generateZ(X, Y);
        mat A = Z.t() * Z;

        // normalize it for convenience
        A = 1 / A(0, 0) * A;

        return Mahalanobis(A);

    }

    int TogersonMetricLearner::selectI() {

        // if you change this, generateB and generateX have to be adjusted
        return 0;

    }

    std::vector<int> TogersonMetricLearner::getDRowIdxs(arma::vec x) {

        CustomSet foundXs;
        vector<int> idxs;

        for(int i = 0; i < expandedX1s.size(); ++i) {

            if(compareArmadilloVec(expandedX1s.at(i), x)) {

                if(foundXs.insert(expandedX2s.at(i)).second)
                    idxs.push_back(i);

            }
        }

        return idxs;
    }

    int TogersonMetricLearner::compareArmadilloVec(arma::vec vec1, arma::vec vec2) {

        int retVal = 1;

        if(vec1.n_elem != vec2.n_elem)
            retVal = 0;

        for(int i = 0; i < vec1.n_elem; ++i) {
            if(vec1(i) != vec2(i))
                retVal = 0;
        }

        return retVal;

    }

    void TogersonMetricLearner::expandConstraints() {

        expandedX1s.clear();
        expandedX2s.clear();
        expandedDistances.clear();

        expandedX1s.insert(expandedX1s.end(), x1s.begin(), x1s.end());
        expandedX1s.insert(expandedX1s.end(), x2s.begin(), x2s.end());

        expandedX2s.insert(expandedX2s.end(), x2s.begin(), x2s.end());
        expandedX2s.insert(expandedX2s.end(), x1s.begin(), x1s.end());

        expandedDistances.insert(expandedDistances.end(), distances.begin(), distances.end());

        for(int i = 0; i < expandedX1s.size(); ++i) {
            xsSet.insert(expandedX1s.at(i));
        }

        D = mat(xsSet.size(), xsSet.size());
        D.fill(0.0);

    }

    arma::mat TogersonMetricLearner::generateX() {

        mat X(dim, xsSet.size() - 1);

        for(int i = 1; i < xsSet.size(); ++i) {
            for(int j = 0; j < dim; ++j) {
                vec currentX = xsSet.at(i);
                X(j, i - 1) = currentX(j);
            }
        }

        return X;
    }

    void TogersonMetricLearner::generateD() {

        int xsCount = xsSet.size();
        D = mat(xsCount, xsCount);

        for(int i = 0; i < xsSet.size(); ++i) {

            vec x1 = xsSet.at(i);
            vector<int> idxs = getDRowIdxs(x1);

            if(xsCount != idxs.size()) {

                string msg = "(TogersonMetricLearner) provided distances not complete";
                cerr << msg << endl;
                throw KukaduException(msg.c_str());

            }

            int x1Idx = xsSet.find(x1).second - 1;

            for(int j = 0; j < idxs.size(); ++j) {

                int x2Idx = xsSet.find(expandedX1s.at(idxs.at(j))).second - 1;
                D(x1Idx, j) = distances.at(idxs.at(j));

            }

        }

    }

    arma::mat TogersonMetricLearner::generateB(int iIdx) {

        vec iVec = getX1(iIdx);

        expandConstraints();
        generateD();

        int i = iIdx;

        mat B(D.n_rows - 1, D.n_cols - 1);
        for(int j = 1; j < D.n_rows; ++j) {
            for(int k = 1; k < D.n_cols; ++k) {

                double costheta = ( pow(D(i, j), 2) + pow(D(i, k), 2) - pow(D(j, k), 2) ) / (2 * D(i, j) * D(i, k));
                B(j - 1, k - 1) = D(i, j) * D(i, k) * costheta;

            }
        }

        return B;

    }

    arma::mat TogersonMetricLearner::generateY(arma::mat B) {

        mat Y;
        mat U, V;
        vec s;

        U.fill(0.0);
        V.fill(0.0);
        s.fill(0.0);

        svd(U, s, V, B);

        for(int i = 0; i < s.n_elem; ++i)
            s(i) = sqrt(s(i));

        mat matS = diagmat(s);

        Y = U * matS;

        return Y;

    }

    arma::mat TogersonMetricLearner::generateZ(arma::mat X, arma::mat Y) {

        mat Z = Y.t() * X.t() * inv(X * X.t());
        return Z;

    }

    arma::mat TogersonMetricLearner::generateA() {
    }

}
