#pragma once

#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>
#include <lenny/robot/Robot.h>

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
    robot::Robot robot = robot::Robot(LENNY_ROBOT_FOLDER "/data/abb_yumi/robot.urdf", gui::Model::f_loadModel);
    Eigen::VectorXd state = Eigen::VectorXd::Zero(robot.getStateSize());
    std::optional<std::pair<std::string, Eigen::Vector3d>> rayIntersection = std::nullopt;
};

}  // namespace lenny