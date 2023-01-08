#pragma once

#include <lenny/control/BasicTrajectoryTracker.h>
#include <lenny/control/EmulatorControlInterface.h>
#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>
#include <lenny/gui/Plot.h>

namespace lenny {

class ControlApp : public gui::Application {
public:
    ControlApp();
    ~ControlApp() = default;

    void setTrajectory();

    //--- Process
    void restart() override;
    void process() override;

    //--- Drawing
    void drawScene() const override;
    void drawGui() override;

public:
    class Robot : public robot::Robot {
    public:
        Robot() : robot::Robot(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/robot.urdf", gui::Model::f_loadModel) {
            endEffectors.insert(
                {"Gripper", std::make_shared<robot::EndEffector>(
                                "wrist_3_link", 0,
                                tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.250), tools::utils::rotX(PI / 2.0) * tools::utils::rotY(-PI / 2.0)))});
            endEffectors.at("Gripper")->visuals.emplace_back(
                LENNY_ROBOT_APP_FOLDER "/config/robotiq_gripper/Gripper.obj", gui::Model::f_loadModel,
                tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.094), tools::utils::rotZ(PI / 2.0) * tools::utils::rotX(-PI / 2.0)), Eigen::Vector3d::Ones(),
                std::nullopt);
        }
    } robot;
    Eigen::VectorXd initialRobotState = robot.loadStateFromFile(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/default_state.json").value();
    Eigen::VectorXd finalRobotState = initialRobotState;
    control::EmulatorControlInterface rci = control::EmulatorControlInterface(robot, Eigen::VectorXb::Ones(robot.getStateSize()),
                                                                              gui::Plot<control::EmulatorControlInterface::PlotType>::f_addPlot);
    control::BasicTrajectoryTracker btt = control::BasicTrajectoryTracker(rci);
    tools::TrajectoryXd trajectory;
    uint numSteps = 60;
    bool isRecedingHorizon = false;
};

}  // namespace lenny