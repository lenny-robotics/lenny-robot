#include "AgentApp.h"

#include <lenny/gui/ImGui.h>

namespace lenny {

AgentApp::AgentApp() : gui::Application("AgentApp") {
    //Setup scene
    const auto [width, height] = getCurrentWindowSize();
    scenes.emplace_back(std::make_shared<gui::Scene>("Scene-1", width, height));
    scenes.back()->f_drawScene = [&]() -> void { drawScene(); };
}

void AgentApp::drawScene() const {
    agent.drawScene(agent.getInitialAgentState());
}

void AgentApp::drawGui() {
    ImGui::Begin("Main Menu");
    agent.drawGui(true);
    ImGui::End();
}

}  // namespace lenny