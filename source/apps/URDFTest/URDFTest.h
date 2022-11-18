#pragma once

#include <lenny/urdf/Model.h>

#include <iostream>

void test() {
    const std::vector<std::string> paths = {LENNY_ROBOT_FOLDER "/data/abb_yumi/robot.urdf", LENNY_ROBOT_FOLDER "/data/bd_spot/base/robot.urdf",
                                            LENNY_ROBOT_FOLDER "/data/franka_panda/robot.urdf"};

    for (const std::string& path : paths) {
        lenny::urdf::Model model(path);
        std::cout << model << std::endl;
        std::cout << "------------------------------------------------------------------------------------------" << std::endl;
    }
}