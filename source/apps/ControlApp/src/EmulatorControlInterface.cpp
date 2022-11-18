#include "EmulatorControlInterface.h"

#include <lenny/tools/Gui.h>

namespace lenny {

EmulatorControlInterface::EmulatorControlInterface(const robot::Robot& robot, const Eigen::VectorXb& dofMask, const tools::Plot<PlotType>::F_addPlot f_addPlot)
    : control::RobotControlInterface(robot, dofMask, f_addPlot), emulator(currentPosition) {
    k_P = 1.0;
    k_D = 0.001;
    positionReachedTolerances = 0.001 * Eigen::VectorXd::Ones(getStateSize());
}

EmulatorControlInterface::~EmulatorControlInterface() {
    if (isConnected())
        terminateCommunication();
}

bool EmulatorControlInterface::isConnected() const {
    return emulator.isConnected();
}

void EmulatorControlInterface::stop() {
    control::RobotControlInterface::stop();
    emulator.stop();
}

void EmulatorControlInterface::sendControlCommandsToPhysicalRobot(double dt) {
    emulator.setTargetState(commandPosition);
}

void EmulatorControlInterface::readPhysicalRobotValues(double dt) {
    Eigen::VectorXd currentState;
    emulator.getCurrentState(currentState);
    currentVelocity = estimateControlVelocity(currentState, currentPosition, dt);
    currentPosition = currentState;
}

void EmulatorControlInterface::connect() {
    emulator.connect();
}

void EmulatorControlInterface::disconnect() {
    emulator.disconnect();
}

void EmulatorControlInterface::drawAdditionalGuiContent() {
    using tools::Gui;
    if (isConnected()) {
        Gui::I->Slider("Time Delay", emulator.timeDelay, 0.0, 1.0);
        Gui::I->Slider("Noise Factor", emulator.noiseFactor, 0.0, 0.01, "%.6f");
    }
}
}  // namespace lenny
