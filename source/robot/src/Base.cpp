#include <lenny/robot/Base.h>
#include <lenny/tools/Gui.h>

#include <fstream>

namespace lenny::robot {

Base::Base(const std::string& linkName)
    : tools::EulerAngleRigidBody(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()), linkName(linkName) {}

void Base::drawGui() {
    using tools::Gui;
    if (Gui::I->TreeNode("Base")) {
        Gui::I->Text("Link: %s", linkName.c_str());
        if (Gui::I->TreeNode("Position Limits")) {
            for (int i = 0; i < 6; i++)
                drawLimitsGui(std::string(dofNames[i]).c_str(), posLimitsList.at(i));
            Gui::I->TreePop();
        }

        if (Gui::I->TreeNode("Velocity Limits")) {
            for (int i = 0; i < 6; i++)
                drawLimitsGui(std::string(dofNames[i]).c_str(), velLimitsList.at(i));
            Gui::I->TreePop();
        }

        if (Gui::I->Button("Save limits to file"))
            saveLimitsToFile(LENNY_PROJECT_FOLDER "/logs/RobotBaseLimits-" + tools::utils::getCurrentDateAndTime() + ".json");
        if (Gui::I->Button("Load limits from file"))
            loadLimitsFromFile(nullptr);

        Gui::I->TreePop();
    }
}

bool Base::saveLimitsToFile(const std::string& filePath) const {
    //Check file extensions
    if (!tools::utils::checkFileExtension(filePath, "json")) {
        LENNY_LOG_WARNING("File `%s` should be a `json` file... Abort!", filePath.c_str())
        return false;
    }

    //Open file
    std::ofstream file(filePath);
    if (!file.is_open()) {
        LENNY_LOG_WARNING("File `%s` could not be opened... Abort!", filePath.c_str());
        return false;
    }

    //Setup json
    json js;

    //Pose limits
    auto saveToFile = [&](const std::array<Limits, 6>& limitList, const std::string& description) -> void {
        for (int i = 0; i < 6; i++) {
            json js_tmp;
            if (limitList[i].has_value()) {
                js_tmp[description]["dof"] = std::string(dofNames[i]);
                js_tmp[description]["lower"] = limitList[i].value().first;
                js_tmp[description]["upper"] = limitList[i].value().second;
                js.push_back(js_tmp);
            }
        }
    };
    saveToFile(posLimitsList, "PoseLimits");
    saveToFile(velLimitsList, "VelocityLimits");

    //Stream to file
    file << std::setw(2) << js << std::endl;

    //Close file
    file.close();

    //Wrap up
    LENNY_LOG_INFO("Successfully saved base limits into file `%s`", filePath.c_str());
    return true;
}

bool Base::loadLimitsFromFile(const char* fP) {
    //Initialize file path
    std::string filePath = fP ? std::string(fP) : tools::utils::browseFile();

    //Check file extensions
    if (!tools::utils::checkFileExtension(filePath, "json")) {
        LENNY_LOG_WARNING("File `%s` should be a `json` file... Abort!", filePath.c_str())
        return false;
    }

    //Open file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        LENNY_LOG_WARNING("File `%s` could not be opened... Abort!", filePath.c_str());
        return false;
    }

    //Load json from file
    json js;
    file >> js;

    //Reset limits
    for (int i = 0; i < 6; i++) {
        posLimitsList[i] = std::nullopt;
        velLimitsList[i] = std::nullopt;
    }

    //Load limits
    auto loadFromFile = [&](std::array<Limits, 6>& limitList, json& j_element) -> void {
        for (auto& j_limits : j_element) {
            const std::string dofName = j_limits.value("dof", decltype(dofName)());
            const double lower = j_limits.value("lower", decltype(lower)());
            const double upper = j_limits.value("upper", decltype(upper)());

            const auto dof_enum = magic_enum::enum_cast<Base::DOFS>(dofName);
            const auto index = magic_enum::enum_integer(dof_enum.value());
            limitList[index] = std::pair<double, double>{lower, upper};
        }
    };

    for (auto& j_element : js) {
        if (j_element.contains("PoseLimits"))
            loadFromFile(posLimitsList, j_element);
        else if (j_element.contains("VelocityLimits"))
            loadFromFile(velLimitsList, j_element);
        else
            LENNY_LOG_ERROR("Unknown element `%s`", j_element.dump().c_str())
    }

    //Close file
    file.close();

    //Wrap up
    LENNY_LOG_INFO("Base limits successfully loaded from file `%s`", filePath.c_str());
    return true;
}

}  // namespace lenny::robot