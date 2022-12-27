#include "AgentsApp.h"

#include <lenny/gui/ImGui.h>

namespace lenny {

AgentsApp::AgentsApp() : gui::Application("AgentsApp") {
    //Setup drawing
    showOrigin = false;

    //Initialize base position
    for (rapt::Agent::SPtr agent : agents) {
        Eigen::VectorXd initialRobotState = agent->getInitialRobotState();
        initialRobotState[0] = tools::utils::getRandomNumberInRange({-2.0, 2.0});
        initialRobotState[2] = tools::utils::getRandomNumberInRange({-2.0, 2.0});
        initialRobotState[5] = tools::utils::getRandomNumberInRange({-PI / 2.0, PI / 2.0});
        agent->setInitialRobotStateFromRobotState(initialRobotState);

        for (rapt::Gripper::UPtr& gripper : agent->grippers)
            gripper->showGripLocation = true;
    }
}

void AgentsApp::drawScene() const {
    for (rapt::Agent::SPtr agent : agents)
        agent->drawScene(rapt::Agent::MotionTrajectory(agent->getInitialAgentState(), 1, 1.0), 0.0, false);
}

void AgentsApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");

    for (rapt::Agent::SPtr agent : agents)
        agent->drawGui(true);

    if (ImGui::Begin("Mesh Simplification")) {
        static float threshold = 0.8f;
        static float targetError = 0.01;
        static bool saveToFile = false;

        ImGui::SliderFloat("Threshold", &threshold, 0.f, 1.f);
        ImGui::SliderFloat("Target Error", &targetError, 0.f, 1.f);
        ImGui::Checkbox("Save To File", &saveToFile);

        auto simplyVisual = [&](const robot::Visual& visual) -> void {
            if (gui::Model* model = dynamic_cast<gui::Model*>(visual.model.get()))
                model->simplify(threshold, targetError, saveToFile);
        };

        auto simplifyAgent = [&](rapt::Agent::SPtr agent) -> void {
            //Loop over robot visuals
            for (auto& [linkName, link] : agent->robot.links)
                for (auto& visual : link.visuals)
                    simplyVisual(visual);

            //Loop over gripper visuals
            for (auto& gripper : agent->grippers)
                for (auto& visual : gripper->visuals)
                    simplyVisual(visual);
        };

        for (rapt::Agent::SPtr agent : agents) {
            if (ImGui::Button(agent->name.c_str()))
                simplifyAgent(agent);
        }
    }

    ImGui::End();
}

}  // namespace lenny