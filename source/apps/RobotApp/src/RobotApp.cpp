#include "RobotApp.h"

#include <lenny/gui/ImGui.h>
#include <lenny/gui/Renderer.h>
#include <lenny/tools/Logger.h>

namespace lenny {

RobotApp::RobotApp() : gui::Application("RobotApp") {
    showOrigin = false;
    showGround = false;
}

void RobotApp::drawScene() const {
    robot.drawScene(state);

    if (rayIntersection.has_value()) {
        const auto& [linkName, globalIntersectionPoint] = rayIntersection.value();
        LENNY_LOG_PRINT(tools::Logger::DEFAULT, "%s\n", linkName.c_str());
        gui::Renderer::I->drawSphere(globalIntersectionPoint, 0.01, Eigen::Vector4d(0.75, 0.0, 0.0, 1.0));
    }
}

void RobotApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");

    robot.drawGui(true);
    robot.drawFKGui(state, "Forward Kinematics");

    ImGui::End();
}

void RobotApp::mouseMoveCallback(double xPos, double yPos) {
    rayIntersection = robot.getFirstLinkHitByRay(state, camera.getRayFromScreenCoordinates(xPos, yPos), robot.showVisuals, robot.showSkeleton);
    gui::Application::mouseMoveCallback(xPos, yPos);
}

}  // namespace lenny