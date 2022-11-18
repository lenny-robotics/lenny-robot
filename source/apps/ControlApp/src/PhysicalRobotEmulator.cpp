#include "PhysicalRobotEmulator.h"

#include <lenny/tools/Logger.h>
#include <lenny/tools/Timer.h>
#include <lenny/tools/Trajectory.h>

#include <random>

namespace lenny {

PhysicalRobotEmulator::PhysicalRobotEmulator(const Eigen::VectorXd& initialState) : currentState(initialState), targetState(initialState) {}

void PhysicalRobotEmulator::connect() {
    controlThreadIsRunning = true;
    controlThread = std::thread(&PhysicalRobotEmulator::applyControlThread, this);
}

void PhysicalRobotEmulator::disconnect() {
    controlThreadIsRunning = false;
    controlThread.join();
}

bool PhysicalRobotEmulator::isConnected() const {
    return controlThreadIsRunning;
}

void PhysicalRobotEmulator::getCurrentState(Eigen::VectorXd& currentState) const {
    currentState = this->currentState;
}

void PhysicalRobotEmulator::setTargetState(const Eigen::VectorXd& targetState) {
    if (targetState.size() != this->targetState.size())
        LENNY_LOG_ERROR("Wrong input size");
    this->targetState = targetState;
    sendCommands = true;
}

void PhysicalRobotEmulator::stop() {
    sendCommands = false;
    commandBuffer.clear();
}

void PhysicalRobotEmulator::applyControlThread() {
    auto getRandomGaussian = [](const double& mean, const double& standardDeviation) -> double {
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> d{mean, standardDeviation};
        return d(gen);
    };

    LENNY_LOG_INFO("Starting control thread...")

    tools::Timer frameRateTimer, commandTimer;
    commandTimer.restart();
    commandBuffer.clear();

    double currentFrameRate = controlFrameRate;
    while (controlThreadIsRunning) {
        //Restart time
        frameRateTimer.restart();

        //Do control schtuff
        if (sendCommands) {
            //Add command to command buffer
            double commandTime = commandTimer.time();
            commandBuffer.push_back({targetState, commandTime});

            //Now find command that is closest to delayed time
            double delayedTime = commandTime - timeDelay;

            //And add a bit of noise to it
            delayedTime += noiseFactor * getRandomGaussian(0.0, 1.0);

            //Figure out current state
            if (commandBuffer.front().timeStamp > delayedTime) {
                //Buffer does not have enough entries yet
                currentState = commandBuffer.front().state;
            } else {
                //Find the lower index of the buffer
                int lowerIndex = 0;
                for (int i = 0; i < (int)commandBuffer.size(); i++) {
                    if (commandBuffer[i].timeStamp > delayedTime)
                        break;
                    lowerIndex = i;
                }

                if (lowerIndex == (int)commandBuffer.size() - 1) {
                    //Lower index corresponds to last buffer entry, so we take the last entry
                    currentState = commandBuffer.back().state;
                } else {
                    //Linear interpolation between the two closest states
                    tools::TrajectoryXd trajectory;
                    trajectory.addEntry(commandBuffer[lowerIndex].timeStamp, commandBuffer[lowerIndex].state);
                    trajectory.addEntry(commandBuffer[lowerIndex + 1].timeStamp, commandBuffer[lowerIndex + 1].state);
                    currentState = trajectory.getLinearInterpolation(delayedTime);
                }
            }

            //Erase command entries if they are no longer relevant
            for (int i = 0; i < commandBuffer.size(); i++) {
                if (commandBuffer[i].timeStamp < delayedTime)
                    commandBuffer.erase(commandBuffer.begin() + i);
                else
                    break;
            }
        }

        //Frame rate
        double waitTime = (1.0 / controlFrameRate) - frameRateTimer.time();
        if (waitTime > 0.0)
            frameRateTimer.sleep(waitTime);

        currentFrameRate = (1.0 / frameRateTimer.time());
        if (printDebugInfo && (controlFrameRate - currentFrameRate) > 1.0)
            LENNY_LOG_WARNING("Frame rate dropped: %lf", currentFrameRate);
    }

    commandBuffer.clear();
    LENNY_LOG_INFO("Control thread terminated");
}

}  // namespace lenny