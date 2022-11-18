#include <lenny/collision/Primitives.h>
#include <lenny/rapt/WorldCollisionHandler.h>
#include <lenny/tools/Gui.h>

#include <fstream>

namespace lenny::rapt {

WorldCollisionParent::SPtr WorldCollisionHandler::parent = std::make_shared<WorldCollisionParent>();

void WorldCollisionHandler::drawScene() const {
    for (const auto& [primitive, parentState] : primitives)
        primitive->drawScene(parentState, Eigen::Vector4d(0.0, 0.0, 0.75, 0.75));
}

void WorldCollisionHandler::drawGui() {
    using tools::Gui;
    if (Gui::I->TreeNode("World Collision Handler")) {
        int iter = 0;
        for (auto& [primitive, state] : primitives)
            primitive->drawGui(primitive->parent->description + " - " + std::to_string(iter++));

        if (Gui::I->TreeNode("Save & Load")) {
            if (Gui::I->Button("Save To File"))
                saveCollisionPrimitivesToFile(LENNY_PROJECT_FOLDER "/logs/WorldCollisionPrimitives-" + tools::utils::getCurrentDateAndTime() + ".json");

            if (Gui::I->Button("Load From File"))
                loadCollisionPrimitivesFromFile(nullptr);

            Gui::I->TreePop();
        }

        Gui::I->TreePop();
    }
}

tools::Transformation WorldCollisionHandler::convertPose(const Eigen::Vector6d& state) const {
    return parent->rigidBody.getTransformationFromState(state);
}

Eigen::Vector6d WorldCollisionHandler::convertPose(const tools::Transformation& trafo) const {
    return parent->rigidBody.getStateFromTransformation(trafo);
}

void WorldCollisionHandler::addCollisionSphere(const Eigen::Vector6d& state, const double& radius) {
    primitives.emplace_back(std::make_pair(std::make_shared<collision::Sphere>(parent, Eigen::Vector3d::Zero(), radius), state));
}

void WorldCollisionHandler::addCollisionCapsule(const Eigen::Vector6d& state, const double& length, const double& radius) {
    primitives.emplace_back(std::make_pair(
        std::make_shared<collision::Capsule>(parent, 0.5 * length * Eigen::Vector3d::UnitX(), -0.5 * length * Eigen::Vector3d::UnitX(), radius), state));
}

void WorldCollisionHandler::addCollisionRectangle(const Eigen::Vector6d& state, const Eigen::Vector2d& dimensions, const double& safetyMargin) {
    primitives.emplace_back(std::make_pair(
        std::make_shared<collision::Rectangle>(parent, Eigen::Vector3d::Zero(), Eigen::QuaternionD::Identity(), dimensions, safetyMargin), state));
}

void WorldCollisionHandler::addCollisionBox(const Eigen::Vector6d& state, const Eigen::Vector3d& dimensions, const double& safetyMargin) {
    primitives.emplace_back(
        std::make_pair(std::make_shared<collision::Box>(parent, Eigen::Vector3d::Zero(), Eigen::QuaternionD::Identity(), dimensions, safetyMargin), state));
}

bool WorldCollisionHandler::saveCollisionPrimitivesToFile(const std::string& filePath) const {
    //Check file extensions
    if (!tools::utils::checkFileExtension(filePath, "json"))
        LENNY_LOG_ERROR("File `%s` should be a `json` file", filePath.c_str())

    //Open file
    std::ofstream file(filePath);
    if (!file.is_open()) {
        LENNY_LOG_WARNING("File `%s` could not be opened\n", filePath.c_str());
        return false;
    }

    //To json
    json js;
    for (const auto& [primitive, parentState] : primitives) {
        json js_tmp;
        primitive->to_json(js_tmp);
        js_tmp["parentState"] = parentState;
        js.push_back(js_tmp);
    }

    //Stream to file
    file << std::setw(2) << js << std::endl;

    //Close file
    file.close();

    //Wrap up
    LENNY_LOG_INFO("Successfully saved collision primitives into file `%s`", filePath.c_str());
    return true;
}

bool WorldCollisionHandler::loadCollisionPrimitivesFromFile(const char* fP) {
    //Initialize file path
    std::string filePath = fP ? std::string(fP) : tools::utils::browseFile();

    //Check file extensions
    if (!tools::utils::checkFileExtension(filePath, "json"))
        LENNY_LOG_ERROR("File `%s` should be a `json` file", filePath.c_str())

    //Open file
    std::ifstream file(filePath);
    if (!file.is_open())
        LENNY_LOG_ERROR("File `%s` could not be opened\n", filePath.c_str());

    //Load json from file
    json js;
    file >> js;

    //From json
    primitives.clear();
    enum PRIMITIVE_DESCRIPTIONS { Sphere, Capsule, Rectangle, Box };
    const Eigen::Vector6d parentState_tmp = Eigen::Vector6d::Zero();
    for (auto& js_tmp : js) {
        const std::string description = js_tmp.value("primitive-description", std::string());
        auto enum_descr = magic_enum::enum_cast<PRIMITIVE_DESCRIPTIONS>(description);
        if (!enum_descr.has_value()) {
            LENNY_LOG_WARNING("Unknown primitive description `%s`. Skipping...", description.c_str());
        } else {
            switch (enum_descr.value()) {
                case Sphere: {
                    addCollisionSphere(parentState_tmp, 0.1);
                    break;
                }
                case Capsule: {
                    addCollisionCapsule(parentState_tmp, 0.1, 0.1);
                    break;
                }
                case Rectangle: {
                    addCollisionRectangle(parentState_tmp, Eigen::Vector2d::Ones(), 0.01);
                    break;
                }
                case Box: {
                    addCollisionBox(parentState_tmp, Eigen::Vector3d::Ones(), 0.01);
                    break;
                }
                default:
                    LENNY_LOG_ERROR("Primitive description `%s` is not handled!", description.c_str());
            }
            primitives.back().first->from_json(js_tmp);
            primitives.back().second = js_tmp.value("parentState", Eigen::Vector6d());
        }
    }

    //Close file
    file.close();

    //Wrap up
    LENNY_LOG_INFO("Collision primitives successfully loaded from file `%s`", filePath.c_str());
    return true;
}

}  // namespace lenny::rapt