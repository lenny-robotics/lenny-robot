#include "AgentApp.h"

#include <lenny/gui/ImGui.h>

namespace lenny {

AgentApp::AgentApp() : gui::Application("AgentApp") {
    showOrigin = false;
    showGround = false;
}

void AgentApp::drawScene() const {
    agent.drawScene(agent.getInitialAgentState());
}

void AgentApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");

    agent.drawGui(true);

    ImGui::End();
}

}  // namespace lenny