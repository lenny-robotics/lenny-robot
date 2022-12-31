#include <lenny/rapt/Gripper.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Renderer.h>

namespace lenny::rapt {

Gripper::Gripper(const robot::Robot& robot, const std::string& linkName, const tools::Transformation& localTrafo, const std::string& description)
    : linkName(linkName), localTrafo(localTrafo), description(description) {
    if (robot.links.find(linkName) == robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong robot with name `%s`", linkName.c_str(), robot.name.c_str());
    timer.restart();
}

void Gripper::close() {
    setFingerPosition(0.0);
}

void Gripper::open() {
    setFingerPosition(1.0);
}

void Gripper::setFingerPosition(const double& fingerPercentage) {
    targetFingerPercentage = fingerPercentage;
    tools::utils::boundToRange(targetFingerPercentage, 0.0, 1.0);
}

double Gripper::getFingerPosition() const {
    return currentFingerPercentage;
}

void Gripper::drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const {
    updateCurrentFingerPercentage();
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

        Gui::I->Input("Finger Velocity", fingerVelocity);
        if (Gui::I->Button("Close"))
            close();
        Gui::I->SameLine();
        if (Gui::I->Button("Open"))
            open();
        Gui::I->SameLine();
        if (Gui::I->Slider("Position", targetFingerPercentage, 0.0, 1.0))
            setFingerPosition(targetFingerPercentage);

        for (uint iter = 0; robot::Visual & visual : visuals)
            visual.drawGui(("Visual - " + std::to_string(iter++)).c_str());
        Gui::I->Checkbox("Show Grip Location", showGripLocation);

        drawAdditionalGuiContent();

        Gui::I->TreePop();
    }
}

void Gripper::updateCurrentFingerPercentage() const {
    const double dt = timer.time();
    timer.restart();
    if (currentFingerPercentage > targetFingerPercentage) {
        currentFingerPercentage -= fingerVelocity * dt;
        tools::utils::boundToRange(currentFingerPercentage, targetFingerPercentage, 1.0);
    } else if (currentFingerPercentage < targetFingerPercentage) {
        currentFingerPercentage += fingerVelocity * dt;
        tools::utils::boundToRange(currentFingerPercentage, 0.0, targetFingerPercentage);
    }
}

}  // namespace lenny::rapt