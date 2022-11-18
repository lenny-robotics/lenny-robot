#include <lenny/rapt/Gripper.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Renderer.h>

namespace lenny::rapt {

Gripper::Gripper(const robot::Robot& robot, const std::string& linkName, const tools::Transformation& localTrafo, const std::string& description)
    : linkName(linkName), localTrafo(localTrafo), description(description) {
    if (robot.links.find(linkName) == robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong robot with name `%s`", linkName.c_str(), robot.name.c_str());
}

void Gripper::drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const {
    if (showGripLocation) {
        const tools::Transformation gripLocation = globalLinkPose * localTrafo;
        tools::Renderer::I->drawCoordinateSystem(gripLocation.position, gripLocation.orientation, 0.05, 0.005);
    }
}

void Gripper::drawGui() {
    using tools::Gui;
    if (Gui::I->TreeNode(description.c_str())) {
        Gui::I->Text("Link: %s", linkName.c_str());
        Gui::I->Input("Local Transformation", localTrafo);
        Gui::I->Slider("Finger Percentage", fingerPercentage, 0.0, 1.0);
        for (uint iter = 0; robot::Visual & visual : visuals)
            visual.drawGui(("Visual - " + std::to_string(iter++)).c_str());

        drawAdditionalGuiContent();

        Gui::I->TreePop();
    }
}

}  // namespace lenny::rapt