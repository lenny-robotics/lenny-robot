#include "ControlApp.h"

#include <lenny/gui/ImGui.h>

namespace lenny {

ControlApp::ControlApp() : gui::Application("ControlApp") {
    //Setup scene
    const auto [width, height] = getCurrentWindowSize();
    scenes.emplace_back(std::make_shared<gui::Scene>("Scene-1", width, height));
    scenes.back()->f_drawScene = [&]() -> void { drawScene(); };
    scenes.back()->showOrigin = false;
}

void ControlApp::setTrajectory() {
    trajectory.clear();
    trajectory.addEntry(0.0, initialRobotState);
    trajectory.addEntry((double)numSteps * getDt(), finalRobotState);
}

void ControlApp::prepareToDraw() {
    setTrajectory();

    if (isRecedingHorizon)
        initialRobotState = trajectory.getLinearInterpolation(getDt());

    if (rci.isConnected() && !btt.isTrackingRunning() && isRecedingHorizon)
        rci.setTargetValues(initialRobotState, 1.0 / targetFramerate);
}

void ControlApp::drawScene() const {
    robot.drawScene(initialRobotState, {});
    rci.drawScene();
    robot.drawVisuals(finalRobotState, {}, std::nullopt, 0.5);
}

void ControlApp::drawGui() {
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