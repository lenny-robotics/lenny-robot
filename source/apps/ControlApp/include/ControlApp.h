#pragma once

#include <lenny/control/BasicTrajectoryTracker.h>
#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>
#include <lenny/gui/Plot.h>

#include "EmulatorControlInterface.h"

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
    robot::Robot robot = robot::Robot(LENNY_ROBOT_FOLDER "/data/floating_base/robot.urdf", gui::Model::f_loadModel);
    Eigen::VectorXd initialRobotState = Eigen::VectorXd::Zero(robot.getStateSize());
    EmulatorControlInterface rci =
        EmulatorControlInterface(robot, Eigen::VectorXb::Ones(robot.getStateSize()), gui::Plot<EmulatorControlInterface::PlotType>::f_addPlot);
    control::BasicTrajectoryTracker btt = control::BasicTrajectoryTracker(rci);
    tools::Transformation targetBasePose;
    tools::TrajectoryXd trajectory;
    uint numSteps = 100;
    bool isRecedingHorizon = false;
};

}  // namespace lenny