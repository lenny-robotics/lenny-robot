#pragma once

#include <lenny/control/PhysicalRobotEmulator.h>
#include <lenny/control/RobotControlInterface.h>

namespace lenny::control {

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

public:
    PhysicalRobotEmulator emulator;
};

}  // namespace lenny::control