#include "ControlApp.h"

#include <lenny/gui/ImGui.h>
#include <lenny/gui/ImGuizmo.h>

namespace lenny {

ControlApp::ControlApp() : gui::Application("ControlApp") {
    showOrigin = false;
    setTrajectory();
}

void ControlApp::setTrajectory() {
    trajectory.clear();
    trajectory.addEntry(0.0, initialRobotState);
    trajectory.addEntry((double)numSteps * getDt(), finalRobotState);
}

void ControlApp::restart() {
    initialRobotState.setZero();
}

void ControlApp::process() {
    setTrajectory();
    if (isRecedingHorizon)
        initialRobotState = trajectory.getLinearInterpolation(getDt());

    if (rci.isConnected() && !btt.isTrackingRunning() && isRecedingHorizon)
        rci.setTargetValues(initialRobotState, 1.0 / targetFramerate);
}

void ControlApp::drawScene() const {
    robot.drawScene(initialRobotState);
    rci.drawScene();
    for (uint i = 0; i < numSteps; i++) {
        const double time = (double)i * getDt();
        robot.drawVisuals(trajectory.getLinearInterpolation(time), Eigen::Vector3d(0.75, 0.75, 0.75), 0.5);
    }
}

void ControlApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");

    if (ImGui::TreeNode("App Settings")) {
        ImGui::InputUInt("Num Steps", &numSteps);
        ImGui::Checkbox("Is Receding Horizon", &isRecedingHorizon);
        ImGui::TreePop();
    }

    robot.drawGui(true);
    robot.drawFKGui(finalRobotState, "Final Robot State");
    rci.drawGui();
    btt.drawGui();

    if (rci.isConnected() && !btt.isTrackingRunning()) {
        if (ImGui::TreeNode("Control")) {
            auto controlGui = [&](std::function<void()> f_positionReached) -> void {
                Eigen::VectorXd currentPosition, currentVelocity;
                rci.getCurrentValues(currentPosition, currentVelocity);
                if (rci.positionReached(currentPosition, initialRobotState)) {
                    f_positionReached();
                } else {
                    if (ImGui::Button("Sync"))
                        btt.gotoStateInTime(initialRobotState, 3.0);
                }
            };

            if (isRecedingHorizon) {
                if (!rci.sendCommands)
                    controlGui([&]() { ImGui::Checkbox("Send commands", &rci.sendCommands); });
                else
                    ImGui::Checkbox("Send commands", &rci.sendCommands);
            } else {
                controlGui([&]() {
                    if (ImGui::Button("Execute")) {
                        btt.executeTrajectory(trajectory, (double)numSteps * getDt());
                    }
                });
            }

            ImGui::TreePop();
        }
    }

    ImGui::End();
}

}  // namespace lenny