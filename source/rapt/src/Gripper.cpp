#include <lenny/rapt/Gripper.h>
#include <lenny/tools/Gui.h>

namespace lenny::rapt {

Gripper::Gripper(const robot::EndEffector::CSPtr endEffector) : endEffector(endEffector) {}

void Gripper::close() {
    setTargetFingerPercentage(0.0);
}

void Gripper::open() {
    setTargetFingerPercentage(1.0);
}

void Gripper::setTargetFingerPercentage(const double& newTargetFingerPercentage) {
    targetFingerPercentage = newTargetFingerPercentage;
    tools::utils::boundToRange(targetFingerPercentage, 0.0, 1.0);
}

double Gripper::getCurrentFingerPercentage() const {
    return currentFingerPercentage;
}

void Gripper::drawGui(const std::string& description) {
    using tools::Gui;

    if (Gui::I->TreeNode(description.c_str())) {
        Gui::I->Input("Finger Velocity", fingerVelocity);
        if (Gui::I->Button("Close"))
            close();
        Gui::I->SameLine();
        if (Gui::I->Button("Open"))
            open();
        Gui::I->SameLine();
        if (Gui::I->Slider("Target Finger Percentage", targetFingerPercentage, 0.0, 1.0))
            setTargetFingerPercentage(targetFingerPercentage);

        Gui::I->TreePop();
    }
}

void Gripper::updateFingerPercentage() const {
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