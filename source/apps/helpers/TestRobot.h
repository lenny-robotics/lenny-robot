#pragma once

#include <lenny/robot/Robot.h>

#include "TestEndEffector.h"

namespace lenny {

class TestRobot : public robot::Robot {
public:
    TestRobot() : robot::Robot(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/robot.urdf", gui::Model::f_loadModel) {
        //Add endeffector
        endEffectors.insert(
            {"Gripper", TestEndEffector("wrist_3_link", tools::Transformation(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::QuaternionD::Identity()))});
    }

    Eigen::VectorXd getInitialState() const {
        return loadStateFromFile(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/default_state.json").value();
    }
};

}  // namespace lenny