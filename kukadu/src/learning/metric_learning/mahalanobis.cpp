#include "mahalanobis.hpp"

#include "../../types/kukadutypes.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    Mahalanobis::Mahalanobis() {

        int dim = 1;
        vec dia(dim);
        dia.fill(1.0);

        M = mat(dim, dim);
        M.fill(0.0);
        M.diag() = dia;

    }

    // delivers Euclidean metric
    Mahalanobis::Mahalanobis(int dim) {

        vec dia(dim);
        dia.fill(1.0);

        M = mat(dim, dim);
        M.fill(0.0);
        M.diag() = dia;

    }

    Mahalanobis::Mahalanobis(arma::mat M) {
        setM(M / M(0,0));
    }

    Mahalanobis::Mahalanobis(const Mahalanobis& maha) {
        this->M = maha.M;
    }

    double Mahalanobis::computeSquaredDistance(arma::vec vec1, arma::vec vec2) {

        vec res = (vec1 - vec2).t() * M * (vec1 - vec2);
        return res(0);

    }

    arma::vec Mahalanobis::getCoefficients() {

        vec ret(M.n_cols * M.n_rows);

        // vectorise function is not there as documented, so i have to implement it by myself
        for(int i = 0, k = 0; i < M.n_cols; ++i) {
            for(int j = 0; j < M.n_rows; ++j, ++k) {
                ret(k) = M(i, j);
            }
        }

        return ret;

    }

    void Mahalanobis::setM(arma::mat M) {
        if(M.n_cols != M.n_rows) {
            string error = "(Mahalanobis) not a squared matrix";
            cerr << error << endl;
            throw KukaduException(error.c_str());
        }
        this->M = M;
    }

    arma::mat Mahalanobis::getM() const {
        return M;
    }

    arma::mat Mahalanobis::getDecomposition() {

        mat Z;
        mat U, V;
        vec s;

        svd_econ(U, s, V, M);

        for(int i = 0; i < s.n_elem; ++i)
            s(i) = sqrt(s(i));

        mat matS = diagmat(s);

        Z = U * matS;

        return Z;

    }

}
