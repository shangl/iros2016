#ifndef KUKADU_HAPTICPLANNER_H
#define KUKADU_HAPTICPLANNER_H

#include <cstdio>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <wordexp.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "../controller.hpp"
#include "../../utils/utils.hpp"
#include "../sensingcontroller.hpp"

namespace kukadu {

    class HapticPlanner {

    private:

        std::vector<std::string> sensingNames;
        std::vector<std::string> preparationNames;
        std::vector<std::string> complexControllerNames;

        std::vector<KUKADU_SHARED_PTR<Controller> > complexControllers;
        std::vector<KUKADU_SHARED_PTR<Controller> > preparationControllers;
        std::vector<KUKADU_SHARED_PTR<SensingController> > sensingControllers;

        void printNamedVector(std::vector<std::string> names);

        int pickSensingController();
        int pickComplexController();
        int pickPreparationController(int sensingController);

    public:

        HapticPlanner();

        void addComplexController(std::string name, KUKADU_SHARED_PTR<Controller> complexController);
        void addPreparationController(std::string name, KUKADU_SHARED_PTR<Controller> prepController);
        void addSensingController(std::string name, KUKADU_SHARED_PTR<SensingController> sensingController);

        void printSensingControllers();
        void printComplexControllers();
        void printPreparationControllers();

    };

}

#endif // HAPTICPLANNER_H
