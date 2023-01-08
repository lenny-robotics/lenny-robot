#pragma once

#include <lenny/gui/Application.h>

#include "TestRobot.h"

namespace lenny {

class RobotApp : public gui::Application {
public:
    RobotApp();
    ~RobotApp() = default;

    //--- Drawing
    void drawScene() const override;
    void drawGui() override;

    //--- Interaction
    void mouseMoveCallback(double xPos, double yPos) override;

public:
    TestRobot robot;
    Eigen::VectorXd state = robot.getInitialState();
    std::optional<std::pair<std::string, Eigen::Vector3d>> rayIntersection = std::nullopt;
};

}  // namespace lenny