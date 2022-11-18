#include "KineLocoApp.h"

#include <lenny/gui/ImGui.h>
#include <lenny/gui/ImGuizmo.h>
#include <lenny/tools/Trajectory.h>

namespace lenny {

KineLocoApp::KineLocoApp() : gui::Application("KineLocoApp") {
    showOrigin = false;
    animator.run = true;
}

void KineLocoApp::restart() {
    targetBasePose = robot.base->getTransformationFromState(initialSpotState.segment(0, 6));
}

void KineLocoApp::process() {
    const bool isInStand = false;

    tools::Trajectory<Eigen::Vector6d> genericTrajectory;
    genericTrajectory.addEntry(0.0, initialSpotState.segment(0, 6));
    genericTrajectory.addEntry((double)numSteps * deltaT, robot.base->getStateFromTransformation(targetBasePose));

    std::vector<std::pair<Eigen::Vector6d, double>> baseTrajectory;
    for (uint i = 0; i < numSteps; i++) {
        const double time = (double)i * deltaT;
        const Eigen::Vector6d basePose = genericTrajectory.getLinearInterpolation(time);
        baseTrajectory.emplace_back(std::make_pair(basePose, time));
    }
    locomotionController.computeRobotStateForTime(currentSpotState, baseTrajectory, animator.currentTime, isInStand);
}

void KineLocoApp::drawScene() const {
    animator.update();
    robot.drawScene(currentSpotState);
    locomotionController.drawScene();
}

void KineLocoApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");

    if (ImGui::TreeNode("App Settings")) {
        ImGui::InputUInt("Num Steps", &numSteps);
        ImGui::InputDouble("DeltaT", &deltaT);

        ImGui::TreePop();
    }

    robot.drawGui(true);
    locomotionController.drawGui();
    animator.drawGui();

    ImGui::SetNextItemOpen(true);
    if (ImGui::TreeNode("ImGuizmo")) {
        static Eigen::Vector3d scale = Eigen::Vector3d::Ones();
        ImGuizmo::useWidget(targetBasePose.position, targetBasePose.orientation, scale, camera.getViewMatrix(), camera.getProjectionMatrix());
        ImGui::TreePop();
    }

    ImGui::End();
}

}  // namespace lenny