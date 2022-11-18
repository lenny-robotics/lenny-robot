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
    }
}

void AgentsApp::drawScene() const {
    for (rapt::Agent::SPtr agent : agents)
        agent->drawScene(agent->getInitialAgentState());
}

void AgentsApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");

    for (rapt::Agent::SPtr agent : agents)
        agent->drawGui(true);

    spotBaseRobot.drawGui();

    ImGui::End();
}

}  // namespace lenny