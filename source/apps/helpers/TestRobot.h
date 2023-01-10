#pragma once

#include <lenny/gui/Model.h>
#include <lenny/robot/Robot.h>

namespace lenny {

class TestRobot : public robot::Robot {
public:
    TestRobot() : robot::Robot(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/robot.urdf", gui::Model::f_loadModel) {
        endEffectors.insert({"Gripper", robot::EndEffector("wrist_3_link", 1,
                                                           tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.250),
                                                                                 tools::utils::rotX(PI / 2.0) * tools::utils::rotY(-PI / 2.0)))});
        endEffectors.at("Gripper").visuals.emplace_back(
            LENNY_ROBOT_APP_FOLDER "/config/robotiq_gripper/Gripper.obj", gui::Model::f_loadModel,
            tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.094), tools::utils::rotZ(PI / 2.0) * tools::utils::rotX(-PI / 2.0)), Eigen::Vector3d::Ones(),
            std::nullopt);
    }

    Eigen::VectorXd getInitialState() const {
        return loadStateFromFile(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/default_state.json").value();
    }
};

}  // namespace lenny