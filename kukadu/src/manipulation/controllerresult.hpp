#ifndef CONTROLLER_RESULT_H
#define CONTROLLER_RESULT_H

#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <wordexp.h>
#include <memory>
#include <armadillo>

namespace kukadu {

    class ControllerResult {

    private:

        bool bored;
        bool success;

        arma::vec t;
        std::vector<arma::vec> y;

        void construct(arma::vec t, std::vector<arma::vec> ys, bool success, bool bored);

    public:

        ControllerResult(arma::vec t, std::vector<arma::vec> ys);
        ControllerResult(arma::vec t, std::vector<arma::vec> ys, bool success, bool bored);

        arma::vec getTimes();
        std::vector<arma::vec> getYs();

        void setSuccess(bool success);
        bool getSuccess();

        bool wasBored();

    };

}

#endif // CONTROLLER_RESULT_H
