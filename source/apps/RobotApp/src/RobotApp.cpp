#include "RobotApp.h"

#include <lenny/gui/ImGui.h>
#include <lenny/gui/Renderer.h>
#include <lenny/tools/Logger.h>

namespace lenny {

RobotApp::RobotApp() : gui::Application("RobotApp") {
    showOrigin = false;
    showGround = false;

    robot.endEffectors.insert(
        {"Gripper", std::make_unique<robot::EndEffector>("wrist_3_link", tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.250),
                                                                                               tools::utils::rotX(PI / 2.0) * tools::utils::rotY(-PI / 2.0)))});
    robot.endEffectors.at("Gripper")->visuals.emplace_back(
        LENNY_ROBOT_APP_FOLDER "/config/robotiq_gripper/Gripper.obj", gui::Model::f_loadModel,
        tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.094), tools::utils::rotZ(PI / 2.0) * tools::utils::rotX(-PI / 2.0)), Eigen::Vector3d::Ones(),
        std::nullopt);
}

void RobotApp::drawScene() const {
    robot.drawScene(state, {{"Gripper", Eigen::VectorXd::Zero(0)}});

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
    rayIntersection = robot.getFirstLinkHitByRay(state, camera.getRayFromScreenCoordinates(xPos, yPos));
    gui::Application::mouseMoveCallback(xPos, yPos);
}

}  // namespace lenny