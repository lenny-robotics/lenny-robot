#include <lenny/robot/Joint.h>
#include <lenny/tools/Gui.h>

namespace lenny::robot {

Eigen::Vector3d Joint::skeletonColor = Eigen::Vector3d(0.0, 0.0, 0.9);

void Joint::drawGui(const std::string& name) {
    using tools::Gui;
    if (Gui::I->TreeNode(name.c_str())) {
        Gui::I->Text("Parent: %s", parentName.c_str());
        Gui::I->Text("Child: %s", childName.c_str());
        Gui::I->Text("Local Position: (%lf, %lf, %lf)", pJPos.x(), pJPos.y(), pJPos.z());
        Gui::I->Text("Axis: (%lf, %lf, %lf)", axis.x(), axis.y(), axis.z());

        drawLimitsGui("Angle Limits", angleLimits);
        drawLimitsGui("Velocity Limits", velLimits);

        Gui::I->TreePop();
    }
}

}  // namespace lenny::robot