#pragma once

#include <lenny/tools/Definitions.h>

#include <thread>

namespace lenny::control {

class PhysicalRobotEmulator {
public:
    PhysicalRobotEmulator(const Eigen::VectorXd& initialState);
    ~PhysicalRobotEmulator() {}

    //Connection
    void connect();
    void disconnect();
    bool isConnected() const;

    //Get and send
    void getCurrentState(Eigen::VectorXd& currentState) const;
    void setTargetState(const Eigen::VectorXd& targetState);
    void stop();

protected:
    void applyControlThread();

public:
    bool printDebugInfo = false;
    double timeDelay = 0.1;  //[s]
    double noiseFactor = 1e-3;

protected:
    Eigen::VectorXd currentState, targetState;

    struct Command {
        Eigen::VectorXd state;
        double timeStamp;
    };
    std::vector<Command> commandBuffer;

    bool controlThreadIsRunning = false;
    std::thread controlThread;
    double controlFrameRate = 100.0;

    bool sendCommands = false;
};

}  // namespace lenny::control
