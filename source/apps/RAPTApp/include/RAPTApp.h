#pragma once

#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>
#include <lenny/rapt/Agent.h>

namespace lenny {

class RAPTApp : public gui::Application {
public:
    RAPTApp();
    ~RAPTApp() = default;

    //--- Drawing
    void drawScene() const override;
    void drawGui() override;

public:
    robot::Robot robot = robot::Robot(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/robot.urdf", gui::Model::f_loadModel);
    std::function<Eigen::VectorXb()> getDofMask = [&]() -> Eigen::VectorXb {
        Eigen::VectorXb dofMask = Eigen::VectorXb::Ones(robot.getStateSize());
        dofMask.segment(0, 6).setZero();
        return dofMask;
    };
    rapt::Agent agent = rapt::Agent("Agent", robot, robot.loadStateFromFile(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/default_state.json").value(), getDofMask());
};

}  // namespace lenny