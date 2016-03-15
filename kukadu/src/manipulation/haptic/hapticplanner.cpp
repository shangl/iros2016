#include "hapticplanner.hpp"

using namespace std;

namespace kukadu {

    HapticPlanner::HapticPlanner() {

    }

    void HapticPlanner::addSensingController(std::string name, KUKADU_SHARED_PTR<SensingController> sensingController) {

        sensingNames.push_back(name);
        sensingControllers.push_back(sensingController);

    }

    void HapticPlanner::addPreparationController(std::string name, KUKADU_SHARED_PTR<Controller> prepController) {

        preparationNames.push_back(name);
        preparationControllers.push_back(prepController);

    }

    void HapticPlanner::addComplexController(std::string name, KUKADU_SHARED_PTR<Controller> complexController) {

        complexControllerNames.push_back(name);
        complexControllers.push_back(complexController);

    }

    void HapticPlanner::printSensingControllers() {
        printNamedVector(sensingNames);
    }

    void HapticPlanner::printPreparationControllers() {
        printNamedVector(preparationNames);
    }

    void HapticPlanner::printComplexControllers() {
        printNamedVector(complexControllerNames);
    }

    int HapticPlanner::pickSensingController() {

        int choice = -1;
        cout << "which sensing controller would you like to try?" << endl;
        printSensingControllers();
        cin >> choice;
        return choice;

    }

    int HapticPlanner::pickComplexController() {

        int choice = -1;
        cout << "which complex controller would you like to try?" << endl;
        printComplexControllers();
        cin >> choice;
        return choice;

    }

    int HapticPlanner::pickPreparationController(int sensingController) {

    }

    void HapticPlanner::printNamedVector(std::vector<std::string> names) {

        for(int i = 0; i < names.size(); ++i)
            cout << "(" << i << ")" << names.at(i) << endl;

    }

}
