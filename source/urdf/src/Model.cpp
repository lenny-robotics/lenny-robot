#include <lenny/tools/Logger.h>
#include <lenny/urdf/Model.h>

namespace lenny::urdf {

Model::Model(const std::string& filePath) {
    loadFromFile(filePath);
}

void Model::loadFromFile(const std::string& filePath) {
    //Create document and load file
    tinyxml2::XMLDocument document;
    document.LoadFile(filePath.c_str());
    if (document.Error()) {
        LENNY_LOG_ERROR("File with path `%s` could not be loaded (Error message: `%s`)", filePath.c_str(), document.ErrorStr());
    }

    //Find robot element
    tinyxml2::XMLElement* robotElement = document.RootElement();
    if (!robotElement || std::string(robotElement->Value()) != "robot") {
        LENNY_LOG_ERROR("Could not find the `robot` element in the xml file");
    }

    //Load name
    this->name = Utils::parseStringValueFromAttribute(robotElement, "name", "Model", Utils::EXPECTED);

    //Load materials (outside of links)
    for (tinyxml2::XMLElement* materialElement = robotElement->FirstChildElement("material"); materialElement != nullptr;
         materialElement = materialElement->NextSiblingElement("material"))
        Material::addNewMaterialToList(this->materials, materialElement);

    //Load links and materials
    for (tinyxml2::XMLElement* linkElement = robotElement->FirstChildElement("link"); linkElement != nullptr;
         linkElement = linkElement->NextSiblingElement("link")) {
        //Links
        this->links.emplace_back(Link::parseFromXmlElement(linkElement));

        //Materials (inside of links)
        for (tinyxml2::XMLElement* visualElement = linkElement->FirstChildElement("visual"); visualElement != nullptr;
             visualElement = visualElement->NextSiblingElement("visual"))
            if (tinyxml2::XMLElement* materialElement = visualElement->FirstChildElement("material"))
                Material::addNewMaterialToList(this->materials, materialElement);
    }

    //Load joints
    for (tinyxml2::XMLElement* jointElement = robotElement->FirstChildElement("joint"); jointElement != nullptr;
         jointElement = jointElement->NextSiblingElement("joint"))
        this->joints.emplace_back(Joint::parseFromXmlElement(jointElement));

    //Apply sanity checks
    this->applySanityChecks();
}

void Model::applySanityChecks() const {
    //--- Check if there is at least one link
    if (links.size() == 0) {
        LENNY_LOG_ERROR("No links found... There should be at least one link");
    }

    //--- Check for duplicate names
    for (int i = 0; i < joints.size(); i++) {
        for (int j = i + 1; j < joints.size(); j++) {
            if (joints[i].name == joints[j].name) {
                LENNY_LOG_ERROR("Joint name `%s` seems to appear more than once", joints[i].name.c_str());
            }
        }
    }

    for (int i = 0; i < links.size(); i++) {
        for (int j = i + 1; j < links.size(); j++) {
            if (links[i].name == links[j].name) {
                LENNY_LOG_ERROR("Link name `%s` seems to appear more than once", links[i].name.c_str());
            }
        }
    }

    for (int i = 0; i < materials.size(); i++) {
        for (int j = i + 1; j < materials.size(); j++) {
            if (materials[i].name == materials[j].name) {
                LENNY_LOG_ERROR("Material name `%s` seems to appear more than once", materials[i].name.c_str());
            }
        }
    }

    //--- Check if all joint child and parent link names exist
    auto checkLinkExistence = [&](const std::string& relative_link_name) -> void {
        bool exists = false;
        for (const Link& link : links) {
            if (link.name == relative_link_name) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            LENNY_LOG_ERROR("Child or parent link with name `%s` does not seem to exist", relative_link_name.c_str());
        }
    };

    for (const Joint& joint : joints) {
        checkLinkExistence(joint.child_link_name);
        checkLinkExistence(joint.parent_link_name);
    }

    //--- Check if one root link exists (root can never be a child)
    std::vector<uint> baseLinkCandidates;
    for (uint i = 0; i < links.size(); i++) {
        bool isAChild = false;
        for (const Joint& joint : joints) {
            if (joint.child_link_name == links.at(i).name) {
                isAChild = true;
                break;
            }
        }
        if (!isAChild)
            baseLinkCandidates.push_back(i);
    }
    if (baseLinkCandidates.size() != 1) {
        LENNY_LOG_ERROR("Invalid number of roots found: `%d`, but there should be 1", baseLinkCandidates.size());
    }
    const urdf::Link& baseLink = links.at(baseLinkCandidates.at(0));

    //--- Check if child and parent names of the same link are not the same
    for (const Joint& joint : joints) {
        if (joint.child_link_name == joint.parent_link_name) {
            LENNY_LOG_ERROR("Child link name and parent link name for link `%s` seems to be the same", joint.name.c_str());
        }
    }

    //--- Check if there is more than one link with the same child
    for (int i = 0; i < joints.size(); i++) {
        for (int j = i + 1; j < joints.size(); j++) {
            if (joints[i].child_link_name == joints[j].child_link_name) {
                LENNY_LOG_ERROR("There seems to be more than one link with the same child `%s`", joints[i].child_link_name.c_str());
            }
        }
    }

    //--- Check if all links are in use
    for (const Link& link : links) {
        bool isInUse = false;
        if (link.name == baseLink.name) {
            isInUse = true;
        } else {
            for (const Joint& joint : joints) {
                if (joint.parent_link_name == link.name || joint.child_link_name == link.name) {
                    isInUse = true;
                    break;
                }
            }
        }
        if (!isInUse) {
            LENNY_LOG_WARNING("Link `%s` does not seem to be used in the kinematic chain", link.name.c_str())
        }
    }

    //--- Check if all required materials are present
    for (const Link& link : links) {
        for (const Link::Visual& visual : link.visuals) {
            if (visual.material_name.has_value()) {
                bool materialExists = false;
                for (const Material& material : materials) {
                    if (material.name == visual.material_name.value()) {
                        materialExists = true;
                        break;
                    }
                }
                if (!materialExists) {
                    LENNY_LOG_ERROR("Material with name `%s` does not exist", visual.material_name.value().c_str());
                }
            }
        }
    }
}

}  // namespace lenny::urdf
