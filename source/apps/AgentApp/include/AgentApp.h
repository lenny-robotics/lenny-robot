#pragma once

#include <lenny/gui/Application.h>
#include <lenny/rapt/Agent.h>

#include "TestRobot.h"

namespace lenny {

class AgentApp : public gui::Application {
public:
    AgentApp();
    ~AgentApp() = default;

    //--- Drawing
    void drawScene() const override;
    void drawGui() override;

public:
    TestRobot robot;
    std::function<Eigen::VectorXb()> getDofMask = [&]() -> Eigen::VectorXb {
        Eigen::VectorXb dofMask = Eigen::VectorXb::Ones(robot.getStateSize());
        dofMask.segment(0, 6).setZero();
        return dofMask;
    };
    rapt::Agent agent = rapt::Agent("Agent", robot, robot.getInitialState(), getDofMask());
};

}  // namespace lenny