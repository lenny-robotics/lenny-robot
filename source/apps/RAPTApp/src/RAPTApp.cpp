#include "RAPTApp.h"

#include <lenny/gui/ImGui.h>

namespace lenny {

RAPTApp::RAPTApp() : gui::Application("RAPTApp") {
    showOrigin = false;
    showGround = false;
}

void RAPTApp::drawScene() const {
    agent.drawScene(agent.getInitialAgentState());
}

void RAPTApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");

    agent.drawGui(true);

    ImGui::End();
}

}  // namespace lenny