#include <lenny/robot/EndEffector.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Renderer.h>

namespace lenny::robot {

EndEffector::EndEffector(const std::string& linkName, const uint& stateSize, const tools::Transformation& localGraspTrafo)
    : linkName(linkName), stateSize(stateSize), localGraspTrafo(localGraspTrafo) {
    f_drawVisuals = [](const std::vector<robot::Visual>& visuals, const Eigen::VectorXd& state, const tools::Transformation& globalLinkPose,
                       const std::optional<Eigen::Vector3d>& color, const double& alpha) -> void {
        for (const auto& visual : visuals) {
            const std::optional<Eigen::Vector3d> col = color.has_value() ? color : visual.color;
            visual.drawScene(globalLinkPose, col, alpha);
        }
    };
}

void EndEffector::checkState(const Eigen::VectorXd& state) const {
    if (state.size() != stateSize)
        LENNY_LOG_ERROR("Invalid state input (Size: %d VS %d)", state.size(), stateSize)
}

void EndEffector::drawScene(const Eigen::VectorXd& state, const tools::Transformation& globalLinkPose, const std::optional<Eigen::Vector3d>& color,
                            const double& alpha) const {
    checkState(state);
    if (f_drawVisuals)
        f_drawVisuals(visuals, state, globalLinkPose, color, alpha);
}

void EndEffector::drawGraspLocation(const tools::Transformation& globalLinkPose) const {
    const tools::Transformation gripLocation = globalLinkPose * localGraspTrafo;
    tools::Renderer::I->drawCoordinateSystem(gripLocation.position, gripLocation.orientation, 0.05, 0.005);
}

void EndEffector::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(description.c_str())) {
        Gui::I->Text("Link: %s", linkName.c_str());
        Gui::I->Input("Local Grasp Trafo", localGraspTrafo);

        for (uint iter = 0; robot::Visual & visual : visuals)
            visual.drawGui(("Visual - " + std::to_string(iter++)).c_str());

        if (f_drawAdditionalGuiContent)
            f_drawAdditionalGuiContent();

        Gui::I->TreePop();
    }
}

}  // namespace lenny::robot