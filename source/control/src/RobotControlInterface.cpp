#include <lenny/control/RobotControlInterface.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Logger.h>
#include <lenny/tools/Timer.h>

namespace lenny::control {

RobotControlInterface::RobotControlInterface(const robot::Robot& robot, const Eigen::VectorXb& dofMask, const tools::Plot<PlotType>::F_addPlot f_addPlot)
    : robot(robot), dofMask(dofMask) {
    if (dofMask.size() != robot.getStateSize())
        LENNY_LOG_ERROR("Invalid dof mask input")
    initializeValues();
    if (f_addPlot)
        initializePlots(f_addPlot);
}

void RobotControlInterface::initializeCommunication() {
    //Check if communication has already been established
    if (comThreadIsRunning)
        return;

    //Establish connection to robot
    connect();

    //Check if connection is established
    if (!isConnected()) {
        LENNY_LOG_WARNING("Connection to robot `%s` could not be established", robot.name.c_str());
        return;
    }

    //Sync simulated with physical robot values and initialize
    readPhysicalRobotValues(1.0);
    currentVelocity.setZero();
    targetPosition = currentPosition;
    targetVelocity = currentVelocity;
    fullTargetState = getRobotStateFromControlState(targetPosition, fullTargetState);
    commandPosition = currentPosition;
    commandVelocity = currentVelocity;

    //Start communication thread
    comThreadIsRunning = true;
    comThread = std::thread(&RobotControlInterface::applyComThread, this);
    LENNY_LOG_INFO("Connection to robot '%s` successfully established", robot.name.c_str());
}

void RobotControlInterface::terminateCommunication() {
    if (!comThreadIsRunning)
        return;

    comThreadIsRunning = false;
    comThread.join();
    disconnect();

    if (!isConnected())
        LENNY_LOG_INFO("Connection to robot `%s` successfully terminated", robot.name.c_str())
    else
        LENNY_LOG_WARNING("Connection to robot `%s` could not be terminated", robot.name.c_str())
}

void RobotControlInterface::getCurrentValues(Eigen::VectorXd& currentRobotPosition, Eigen::VectorXd& currentRobotVelocity) const {
    currentRobotPosition = getRobotStateFromControlState(currentPosition, fullTargetState);
    currentRobotVelocity = getRobotStateFromControlState(currentVelocity, Eigen::VectorXd::Zero(robot.getStateSize()));
}

void RobotControlInterface::setTargetValues(const Eigen::VectorXd& targetRobotPosition, const Eigen::VectorXd& targetRobotVelocity) {
    if (targetRobotPosition.size() != robot.getStateSize() || targetRobotVelocity.size() != robot.getStateSize())
        LENNY_LOG_ERROR("Invalid inputs")

    fullTargetState = targetRobotPosition;
    targetPosition = getControlStateFromRobotState(targetRobotPosition);
    targetVelocity = getControlStateFromRobotState(targetRobotVelocity);
}

void RobotControlInterface::setTargetValues(const Eigen::VectorXd& targetRobotPosition, const double& dt) {
    if (targetRobotPosition.size() != robot.getStateSize() || dt < 1e-6)
        LENNY_LOG_ERROR("Invalid inputs")

    fullTargetState = targetRobotPosition;
    const Eigen::VectorXd previousTargetPosition = targetPosition;
    targetPosition = getControlStateFromRobotState(targetRobotPosition);
    targetVelocity = estimateControlVelocity(targetPosition, previousTargetPosition, dt);
}

void RobotControlInterface::stop() {
    sendCommands = false;
    commandPosition = currentPosition;
    commandVelocity.setZero();
    sendControlCommandsToPhysicalRobot(1.0);
}

bool RobotControlInterface::positionReached(const Eigen::VectorXd& currentRobotPosition, const Eigen::VectorXd& targetRobotPosition) const {
    if (currentRobotPosition.size() != robot.getStateSize() || targetRobotPosition.size() != robot.getStateSize())
        LENNY_LOG_ERROR("Invalid inputs")

    const Eigen::VectorXd currentControlPosition = getControlStateFromRobotState(currentRobotPosition);
    const Eigen::VectorXd targetControlPosition = getControlStateFromRobotState(targetRobotPosition);
    const Eigen::VectorXd difference = estimateControlVelocity(targetControlPosition, currentControlPosition, 1.0);
    for (int i = 0; i < difference.size(); i++)
        if (fabs(difference[i]) > positionReachedTolerances[i])
            return false;
    return true;
}

/*
 * COMMENT: Estimates delay based on positionPlots (CURRENT_VALUE VS TARGET_VALUE)
 */
void RobotControlInterface::estimateDelay(std::map<std::string, double>& estimatePerDof) const {
    //Clear input
    estimatePerDof.clear();

    //Check if data is initialized
    if (positionPlots.size() != getStateSize() || getStateSize() <= 0) {
        LENNY_LOG_WARNING("Plot data does not seem to be initialized. Therefore, delay estimation does not work...")
        return;
    }

    //Check if enough data has been recorded
    const int indexWindow = 0.5 / averageDeltaT;  // seconds / seconds
    if (positionPlots.at(0)->getData().size() < 2 * indexWindow) {
        LENNY_LOG_WARNING("Not enought data recorded for estimation...")
        return;
    }

    //Collect delays for individual dofs
    const double deltaThreshold = 0.001;  //Ignore measurement if there is not enough change
    for (int i = 0; i < positionPlots.size(); i++) {
        double averageDofDelay = 0.0;
        double counter = 0.0;
        const auto& data = positionPlots.at(i)->getData();
        for (int j = indexWindow; j < data.size() - indexWindow; j++) {
            const double targetTime = data.at(j).first;
            const double targetValue = data.at(j).second.at(TARGET_VALUE);

            if (fabs(targetValue - data.at(j + 1).second.at(TARGET_VALUE)) < deltaThreshold)
                continue;

            double currentTime = data.at(j).first;
            double currentValue = data.at(j).second.at(CURRENT_VALUE);

            double valueDifference = fabs(targetValue - currentValue);
            double timeDifference = currentTime - targetTime;

            const int maxIndex = std::min((int)data.size(), j + 1 + indexWindow);
            for (int k = j + 1; k < maxIndex; k++) {  //Starting from target value, go forward and check current value
                currentTime = data.at(k).first;
                currentValue = data.at(k).second.at(CURRENT_VALUE);

                const double diff = fabs(targetValue - currentValue);
                if (diff < valueDifference) {
                    valueDifference = diff;
                    timeDifference = currentTime - targetTime;
                }
            }
            averageDofDelay += timeDifference;
            counter += 1.0;
        }
        if (counter > 0.0) {
            averageDofDelay /= counter;
            estimatePerDof.insert({positionPlots.at(i)->getTitle(), averageDofDelay});
            LENNY_LOG_DEBUG("Average delay for Dof %d: %lf", i, averageDofDelay)
        } else {
            LENNY_LOG_WARNING("Not enough relevant data to estimate delay for Dof %d", i)
        }
    }
}

void RobotControlInterface::drawScene(double alpha) const {
    if (!isConnected())
        return;

    //ToDo: End-effectors...
    if (showCurrentRobot && readCommands) {
        const Eigen::VectorXd robotState = getRobotStateFromControlState(currentPosition, fullTargetState);
        robot.drawVisuals(robotState, {}, currentDrawColor, alpha);
    }

    if (showTargetRobot && sendCommands) {
        const Eigen::VectorXd robotState = getRobotStateFromControlState(targetPosition, fullTargetState);
        robot.drawVisuals(robotState, {}, targetDrawColor, alpha);
    }

    if (showCommandRobot && sendCommands) {
        const Eigen::VectorXd robotState = getRobotStateFromControlState(commandPosition, fullTargetState);
        robot.drawVisuals(robotState, {}, commandDrawColor, alpha);
    }
}

void RobotControlInterface::drawGui() {
    using tools::Gui;
    if (Gui::I->TreeNode(("Control Interface - " + robot.name).c_str())) {
        if (isConnected()) {
            if (sendCommands)
                Gui::I->TextColored(Eigen::Vector4d(0.75, 0.0, 0.0, 1.0), "SENDING COMMANDS");
            else
                Gui::I->TextColored(Eigen::Vector4d(0.0, 0.75, 0.0, 1.0), "IDLE");

            if (Gui::I->Button("STOP"))
                stop();

            if (Gui::I->Button("Disconnect"))
                terminateCommunication();

            if (Gui::I->TreeNode("Control Settings")) {
                Gui::I->Input("P Gain", k_P);
                Gui::I->Input("D Gain", k_D);

                Gui::I->TreePop();
            }

            if (Gui::I->TreeNode("Draw Settings")) {
                Gui::I->Checkbox("Show Current Robot", showCurrentRobot);
                Gui::I->Checkbox("Show Target Robot", showTargetRobot);
                Gui::I->Checkbox("Show Command Robot", showCommandRobot);

                Gui::I->TreePop();
            }

            if (Gui::I->TreeNode("Tolerances")) {
                if (Gui::I->TreeNode("Position Reached")) {
                    for (int iter = 0, i = 0; i < robot.getStateSize(); i++)
                        if (dofMask[i])
                            Gui::I->Input(robot.getDescriptionForDofIndex(i).c_str(), positionReachedTolerances[iter++]);

                    Gui::I->TreePop();
                }

                if (Gui::I->TreeNode("Emergency Stop")) {
                    for (int iter = 0, i = 0; i < robot.getStateSize(); i++)
                        if (dofMask[i])
                            Gui::I->Input(robot.getDescriptionForDofIndex(i).c_str(), emergencyStopTolerances[iter++]);

                    Gui::I->TreePop();
                }

                Gui::I->TreePop();
            }

            if (Gui::I->TreeNode("Framerate Interface")) {
                Gui::I->Input("Target Framerate", targetFramerate);
                Gui::I->Text("Current Framerate:	%lf", currentFramerate);
                Gui::I->Text("Dt Average:		%lf", averageDeltaT);

                Gui::I->TreePop();
            }

            if (Gui::I->TreeNode("Plots")) {
                if (Gui::I->Button("Estimate Delay")) {
                    std::map<std::string, double> estimatePerDof;
                    estimateDelay(estimatePerDof);
                }

                if (Gui::I->Button("Clear"))
                    clearPlots();

                if (Gui::I->TreeNode("Positions")) {
                    for (auto& plot : positionPlots)
                        plot->draw();
                    Gui::I->TreePop();
                }

                if (Gui::I->TreeNode("Velocities")) {
                    for (auto& plot : velocityPlots)
                        plot->draw();
                    Gui::I->TreePop();
                }

                Gui::I->TreePop();
            }

            if (Gui::I->TreeNode("Save To File")) {
                if (Gui::I->Button("Save Current Robot State")) {
                    const std::string path = LENNY_PROJECT_FOLDER "/logs/RobotState_" + tools::utils::getCurrentDateAndTime() + ".json";
                    const Eigen::VectorXd robotState = getRobotStateFromControlState(currentPosition, fullTargetState);
                    robot.saveStateToFile(robotState, path);
                }

                Gui::I->TreePop();
            }

        } else {
            if (Gui::I->Button("Connect")) {
                initializeCommunication();
            }
        }

        drawAdditionalGuiContent();

        Gui::I->TreePop();
    }
}

void RobotControlInterface::initializeValues() {
    fullTargetState = Eigen::VectorXd::Zero(robot.getStateSize());

    const uint size = getStateSize();
    currentPosition = Eigen::VectorXd::Zero(size);
    currentVelocity = Eigen::VectorXd::Zero(size);
    targetPosition = Eigen::VectorXd::Zero(size);
    targetVelocity = Eigen::VectorXd::Zero(size);
    commandPosition = Eigen::VectorXd::Zero(size);
    commandVelocity = Eigen::VectorXd::Zero(size);

    positionReachedTolerances = Eigen::VectorXd::Ones(size);
    emergencyStopTolerances = Eigen::VectorXd::Ones(size);
    int iter = 0;
    for (int i = 0; i < dofMask.size(); i++) {
        if (!dofMask[i])
            continue;
        if (i < 3) {  //Base position
            positionReachedTolerances[iter] = 0.03;
            emergencyStopTolerances[iter] = 1.0;
        } else if (i >= 3 && i < 6) {  //Base orientation
            positionReachedTolerances[iter] = 0.15;
            emergencyStopTolerances[iter] = PI / 4.0;
        } else {  //Joint angles
            positionReachedTolerances[iter] = 1e-2;
            emergencyStopTolerances[iter] = PI / 8.0;
        }

        iter++;
    }
}

uint RobotControlInterface::getStateSize() const {
    uint size = 0;
    for (int i = 0; i < dofMask.size(); i++)
        size += (uint)dofMask[i];
    return size;
}

Eigen::VectorXd RobotControlInterface::getControlStateFromRobotState(const Eigen::VectorXd& robotState) const {
    Eigen::VectorXd controlState(getStateSize());
    int iter = 0;
    for (int i = 0; i < dofMask.size(); i++) {
        if (dofMask[i])
            controlState[iter++] = robotState[i];
    }
    return controlState;
}

Eigen::VectorXd RobotControlInterface::getRobotStateFromControlState(const Eigen::VectorXd& controlState, const Eigen::VectorXd& fullRobotState) const {
    Eigen::VectorXd robotState(fullRobotState);
    int iter = 0;
    for (int i = 0; i < dofMask.size(); i++) {
        if (dofMask[i])
            robotState[i] = controlState[iter++];
    }
    return robotState;
}

Eigen::VectorXd RobotControlInterface::estimateControlVelocity(const Eigen::VectorXd& position1, const Eigen::VectorXd& position0, const double& dt) const {
    const int size = getStateSize();
    if (position1.size() != size || position0.size() != size || dt < 1e-6)
        LENNY_LOG_ERROR("Invalid input(s)")

    int iter = 0;
    Eigen::VectorXd velocity(size);
    for (int i = 0; i < dofMask.size(); i++) {
        if (!dofMask[i])
            continue;

        if (i < 3)
            velocity[iter] = (position1[iter] - position0[iter]) / dt;
        else
            velocity[iter] = robot::Robot::estimateAngularVelocity(position1[iter], position0[iter], dt);

        iter++;
    }
    return velocity;
}

void RobotControlInterface::computeCommandsUsingPDControl() {
    //P gain
    const Eigen::VectorXd positionError = estimateControlVelocity(targetPosition, currentPosition, 1.0);

    //D gain
    const Eigen::VectorXd velocityError = targetVelocity - currentVelocity;

    //For velocity control
    commandVelocity = k_P * positionError + k_D * velocityError;

    //For position control
    commandPosition = currentPosition + commandVelocity;
}

void RobotControlInterface::applyEmergencyStopCheck() {
    const Eigen::VectorXd difference = estimateControlVelocity(targetPosition, currentPosition, 1.0);
    for (int i = 0; i < difference.size(); i++) {
        if (fabs(difference[i]) > emergencyStopTolerances[i]) {
            stop();
            LENNY_LOG_WARNING("Emergency stop for dof index `%d` triggered: Current: %lf. Target: %lf. Difference: %lf", i, currentPosition[i],
                              targetPosition[i], difference[i]);
        }
    }
}

void RobotControlInterface::applyComThread() {
    //Initialize
    LENNY_LOG_INFO("Starting communication thread...")
    averageDeltaT = 1.0 / targetFramerate;
    tools::Timer timer;

    //Run loop
    while (comThreadIsRunning && isConnected()) {
        //Restart timer
        timer.restart();

        //Read values
        if (readCommands) {
            readPhysicalRobotValues(averageDeltaT);
        } else {
            currentPosition = targetPosition;
            currentVelocity = targetVelocity;
        }

        //Send values
        if (sendCommands) {
            //Compute commands
            computeCommandsUsingPDControl();

            //Send commands
            sendControlCommandsToPhysicalRobot(averageDeltaT);

            //Check emergency stop
            applyEmergencyStopCheck();

            //Update plots
            updatePlots();
        }

        //Handle framerate
        double waitTime = (1.0 / targetFramerate) - timer.time();
        if (waitTime > 0.0)
            timer.sleep(waitTime);
        currentFramerate = 1.0 / timer.time();
        averageDeltaT = 0.5 * (averageDeltaT + timer.time());
        if (printDebugInfo && (targetFramerate - currentFramerate > 1.0))
            LENNY_LOG_WARNING("Frame rate dropped: %lf VS %lf", currentFramerate, targetFramerate)
    }
    LENNY_LOG_INFO("Terminated communication thread...")
}

void RobotControlInterface::initializePlots(const tools::Plot<PlotType>::F_addPlot f_addPlot) {
    //enum PLOT_ENTRY { LOWER_LIMIT, UPPER_LIMIT, CURRENT_VALUE, TARGET_VALUE, COMMAND_VALUE };
    constexpr auto plotEntryNames = magic_enum::enum_names<PLOT_ENTRY>();
    static const std::vector<std::array<float, 3>> plotColors = {{0.5f, 0.f, 0.f},
                                                                 {0.5f, 0.f, 0.f},
                                                                 {(float)currentDrawColor[0], (float)currentDrawColor[1], (float)currentDrawColor[2]},
                                                                 {(float)targetDrawColor[0], (float)targetDrawColor[1], (float)targetDrawColor[2]},
                                                                 {(float)commandDrawColor[0], (float)commandDrawColor[1], (float)commandDrawColor[2]}};

    for (uint i = 0; i < robot.getStateSize(); i++) {
        if (!dofMask[i])
            continue;
        const std::string description = robot.getDescriptionForDofIndex(i);
        f_addPlot(positionPlots, description, "time", "value", 1000);
        f_addPlot(velocityPlots, description, "time", "value", 1000);

        positionPlots.back()->addLineSpec({std::string(plotEntryNames.at(0)), [](const PlotType& d) { return (float)d.at(0); }, plotColors.at(0)});
        velocityPlots.back()->addLineSpec({std::string(plotEntryNames.at(0)), [](const PlotType& d) { return (float)d.at(0); }, plotColors.at(0)});

        positionPlots.back()->addLineSpec({std::string(plotEntryNames.at(1)), [](const PlotType& d) { return (float)d.at(1); }, plotColors.at(1)});
        velocityPlots.back()->addLineSpec({std::string(plotEntryNames.at(1)), [](const PlotType& d) { return (float)d.at(1); }, plotColors.at(1)});

        positionPlots.back()->addLineSpec({std::string(plotEntryNames.at(2)), [](const PlotType& d) { return (float)d.at(2); }, plotColors.at(2)});
        velocityPlots.back()->addLineSpec({std::string(plotEntryNames.at(2)), [](const PlotType& d) { return (float)d.at(2); }, plotColors.at(2)});

        positionPlots.back()->addLineSpec({std::string(plotEntryNames.at(3)), [](const PlotType& d) { return (float)d.at(3); }, plotColors.at(3)});
        velocityPlots.back()->addLineSpec({std::string(plotEntryNames.at(3)), [](const PlotType& d) { return (float)d.at(3); }, plotColors.at(3)});

        positionPlots.back()->addLineSpec({std::string(plotEntryNames.at(4)), [](const PlotType& d) { return (float)d.at(4); }, plotColors.at(4)});
        velocityPlots.back()->addLineSpec({std::string(plotEntryNames.at(4)), [](const PlotType& d) { return (float)d.at(4); }, plotColors.at(4)});
    }
}

void RobotControlInterface::clearPlots() {
    for (auto& plot : positionPlots)
        plot->clearData();
    for (auto& plot : velocityPlots)
        plot->clearData();
}

void RobotControlInterface::updatePlots() {
    if (positionPlots.size() != getStateSize() || velocityPlots.size() != getStateSize())
        return;

    static float time = 0.f;
    int iter = 0;
    for (int i = 0; i < robot.getStateSize(); i++) {
        if (!dofMask[i])
            continue;

        {  //--- Position plots
            PlotType plotData;
            const auto& limit = robot.getLimitsForDofIndex(i, robot::Robot::POSITION);
            if (limit.has_value()) {
                plotData.at(LOWER_LIMIT) = limit->first;
                plotData.at(UPPER_LIMIT) = limit->second;
            } else {
                plotData.at(LOWER_LIMIT) = currentPosition[iter];
                plotData.at(UPPER_LIMIT) = currentPosition[iter];
            }
            plotData.at(CURRENT_VALUE) = currentPosition[iter];
            plotData.at(TARGET_VALUE) = targetPosition[iter];
            plotData.at(COMMAND_VALUE) = commandPosition[iter];

            positionPlots.at(iter)->addData(time, plotData);
        }

        {  //--- Velocity plots
            PlotType plotData;
            const auto& limit = robot.getLimitsForDofIndex(i, robot::Robot::VELOCITY);
            if (limit.has_value()) {
                plotData.at(LOWER_LIMIT) = limit->first;
                plotData.at(UPPER_LIMIT) = limit->second;
            } else {
                plotData.at(LOWER_LIMIT) = currentVelocity[iter];
                plotData.at(UPPER_LIMIT) = currentVelocity[iter];
            }
            plotData.at(CURRENT_VALUE) = currentVelocity[iter];
            plotData.at(TARGET_VALUE) = targetVelocity[iter];
            plotData.at(COMMAND_VALUE) = commandVelocity[iter];

            velocityPlots.at(iter)->addData(time, plotData);
        }

        iter++;
    }
    time += (float)averageDeltaT;
}

}  // namespace lenny::control