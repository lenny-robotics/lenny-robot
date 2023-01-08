#pragma once

#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>
#include <lenny/rapt/Agent.h>

namespace lenny {

class AgentApp : public gui::Application {
public:
    AgentApp();
    ~AgentApp() = default;

    //--- Drawing
    void drawScene() const override;
    void drawGui() override;

public:
    class Robot : public robot::Robot {
    public:
        Robot() : robot::Robot(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/robot.urdf", gui::Model::f_loadModel) {
            endEffectors.insert(
                {"Gripper", std::make_shared<robot::EndEffector>(
                                "wrist_3_link", 1,
                                tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.250), tools::utils::rotX(PI / 2.0) * tools::utils::rotY(-PI / 2.0)))});
            endEffectors.at("Gripper")->visuals.emplace_back(
                LENNY_ROBOT_APP_FOLDER "/config/robotiq_gripper/Gripper.obj", gui::Model::f_loadModel,
                tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.094), tools::utils::rotZ(PI / 2.0) * tools::utils::rotX(-PI / 2.0)), Eigen::Vector3d::Ones(),
                std::nullopt);
        }
    } robot;
    std::function<Eigen::VectorXb()> getDofMask = [&]() -> Eigen::VectorXb {
        Eigen::VectorXb dofMask = Eigen::VectorXb::Ones(robot.getStateSize());
        dofMask.segment(0, 6).setZero();
        return dofMask;
    };
    rapt::Agent agent = rapt::Agent("Agent", robot, robot.loadStateFromFile(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/default_state.json").value(), getDofMask());
};

}  // namespace lenny