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
    robot::Robot robot = robot::Robot(LENNY_ROBOT_APP_FOLDER "/config/ur_5e/robot.urdf", gui::Model::f_loadModel);
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