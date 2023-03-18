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
    void prepareToDraw() override;
    void drawScene() const;
    void drawGui();

public:
    TestRobot robot;
    Eigen::VectorXd initialRobotState = robot.getInitialState();
    Eigen::VectorXd finalRobotState = initialRobotState;
    std::function<Eigen::VectorXb()> getDofMask = [&]() -> Eigen::VectorXb {
        Eigen::VectorXb dofMask = Eigen::VectorXb::Ones(robot.getStateSize());
        dofMask.segment(0, 6).setZero();
        return dofMask;
    };
    control::EmulatorControlInterface rci =
        control::EmulatorControlInterface(robot, getDofMask(), gui::Plot<control::EmulatorControlInterface::PlotType>::f_addPlot);
    control::BasicTrajectoryTracker btt = control::BasicTrajectoryTracker(rci);
    tools::TrajectoryXd trajectory;
    uint numSteps = 60;
    bool isRecedingHorizon = false;
};

}  // namespace lenny