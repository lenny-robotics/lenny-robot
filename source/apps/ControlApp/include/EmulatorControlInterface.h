#pragma once

#include <lenny/control/RobotControlInterface.h>

#include "PhysicalRobotEmulator.h"

namespace lenny {

class EmulatorControlInterface : public control::RobotControlInterface {
public:
    EmulatorControlInterface(const robot::Robot& robot, const Eigen::VectorXb& dofMask, const tools::Plot<PlotType>::F_addPlot f_addPlot);
    ~EmulatorControlInterface();

    bool isConnected() const override;
    void stop() override;

private:
    void sendControlCommandsToPhysicalRobot(double dt) override;
    void readPhysicalRobotValues(double dt) override;

    void connect() override;
    void disconnect() override;

    void drawAdditionalGuiContent() override;

private:
    PhysicalRobotEmulator emulator;
};

}  // namespace lenny