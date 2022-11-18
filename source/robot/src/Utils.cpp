#include <lenny/robot/Utils.h>
#include <lenny/tools/Gui.h>

namespace lenny::robot {

void drawLimitsGui(const std::string& description, Limits& limits) {
    using tools::Gui;
    if (Gui::I->TreeNode(description.c_str())) {
        if (limits.has_value()) {
            Gui::I->PushItemWidth(75.f);
            Gui::I->Input("Lower", limits->first);
            Gui::I->SameLine();
            Gui::I->Input("Upper", limits->second);
            Gui::I->SameLine();
            if (Gui::I->Button("Deactivate"))
                limits = std::nullopt;
            Gui::I->PopItemWidth();
        } else {
            if (Gui::I->Button("Activate"))
                limits = std::make_pair<double, double>(-PI, PI);
        }
        Gui::I->TreePop();
    }
}

Visual::Visual(const std::string& filePath) : filePath(filePath) {
    model = std::make_unique<tools::Model>("dummy");
}

void Visual::drawGui(const std::string& description) {
    using tools::Gui;
    if (model) {
        if (Gui::I->TreeNode(description.c_str())) {
            Gui::I->Input("Local Trafo", localTrafo);
            Gui::I->Input("Scale", scale);
            if (color.has_value())
                Gui::I->ColorPicker3("Color", color.value());

            Gui::I->TreePop();
        }
    }
}

}  // namespace lenny::robot