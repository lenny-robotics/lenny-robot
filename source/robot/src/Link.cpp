#include <lenny/robot/Link.h>
#include <lenny/tools/Gui.h>

namespace lenny::robot {

Eigen::Vector3d Link::skeletonColor = Eigen::Vector3d(0.5, 0.5, 0.5);

void Link::drawGui(const std::string& name) {
    using tools::Gui;
    if (Gui::I->TreeNode(name.c_str())) {
        for (uint index = 0; Visual & visual : visuals)
            visual.drawGui("Visual - " + std::to_string(index++));
        Gui::I->TreePop();
    }
}

}  // namespace lenny::robot