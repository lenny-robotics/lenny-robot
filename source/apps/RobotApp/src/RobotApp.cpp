#include "RobotApp.h"

#include <lenny/gui/ImGui.h>
#include <lenny/gui/Renderer.h>
#include <lenny/tools/Logger.h>

namespace lenny {

RobotApp::RobotApp() : gui::Application("RobotApp") {
    //Setup scene
    const auto [width, height] = getCurrentWindowSize();
    scenes.emplace_back(std::make_shared<gui::Scene>("Scene-1", width, height));
    scenes.back()->f_drawScene = [&]() -> void { drawScene(); };
    scenes.back()->f_mouseMoveCallback = [&](double xPos, double yPos, Ray ray) -> void { mouseMoveCallback(xPos, yPos, ray); };
}

void RobotApp::drawScene() const {
    robot.drawScene(robotState, {{"Gripper", endEffectorState}});

    if (rayIntersection.has_value()) {
        const auto& [linkName, globalIntersectionPoint] = rayIntersection.value();
        LENNY_LOG_PRINT(tools::Logger::DEFAULT, "%s\n", linkName.c_str());
        gui::Renderer::I->drawSphere(globalIntersectionPoint, 0.01, Eigen::Vector4d(0.75, 0.0, 0.0, 1.0));
    }
}

void RobotApp::drawGui() {
    ImGui::Begin("Main Menu");

    robot.drawGui(true);
    robot.drawFKGui(robotState, "Forward Kinematics");
    ImGui::SliderDouble("EndEffector State", &endEffectorState[0], 0.0, 1.0);

    ImGui::End();
}

void RobotApp::mouseMoveCallback(double xPos, double yPos, Ray ray) {
    rayIntersection = robot.getFirstLinkHitByRay(robotState, ray);
}

}  // namespace lenny