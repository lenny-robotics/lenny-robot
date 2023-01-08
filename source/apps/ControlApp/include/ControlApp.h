#pragma once

#include <lenny/control/BasicTrajectoryTracker.h>
#include <lenny/control/EmulatorControlInterface.h>
#include <lenny/gui/Application.h>
#include <lenny/gui/Plot.h>

#include "TestRobot.h"

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
    TestRobot robot;
    Eigen::VectorXd initialRobotState = robot.getInitialState();
    Eigen::VectorXd finalRobotState = initialRobotState;
    control::EmulatorControlInterface rci = control::EmulatorControlInterface(robot, Eigen::VectorXb::Ones(robot.getStateSize()),
                                                                              gui::Plot<control::EmulatorControlInterface::PlotType>::f_addPlot);
    control::BasicTrajectoryTracker btt = control::BasicTrajectoryTracker(rci);
    tools::TrajectoryXd trajectory;
    uint numSteps = 60;
    bool isRecedingHorizon = false;
};

}  // namespace lenny