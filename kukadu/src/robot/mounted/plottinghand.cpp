#include "plottinghand.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    PlottingHand::PlottingHand(std::string type, std::string hand) : RosSchunk(type, hand) {

    }

    void PlottingHand::connectHand() {
        // nothing to do
    }

    void PlottingHand::closeHand(double percentage, double velocity) {

    }

    void PlottingHand::disconnectHand() {

        // nothing to do

    }

    void PlottingHand::setGrasp(kukadu_grasps grasp) {

    }

    void PlottingHand::safelyDestroy() {

    }

    void PlottingHand::publishSingleJoint(int idx, double pos) {

    }

    void PlottingHand::publishSdhJoints(std::vector<double> positions) {

    }

    std::vector<arma::mat> PlottingHand::getTactileSensing() {

    }

    void PlottingHand::moveJoints(arma::vec joints) {

    }

}
