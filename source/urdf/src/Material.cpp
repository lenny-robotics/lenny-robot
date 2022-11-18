#include <lenny/tools/Logger.h>
#include <lenny/urdf/Material.h>

namespace lenny::urdf {

Material Material::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    if (!xmlElement) {
        LENNY_LOG_ERROR("Invalid xml element");
    }

    Material material;
    material.name = Utils::parseStringValueFromAttribute(xmlElement, "name", "Link::Material", Utils::MANDATORY).value();
    if (tinyxml2::XMLElement* textureElement = xmlElement->FirstChildElement("texture"))
        material.texture_filename = Utils::parseStringValueFromAttribute(textureElement, "filename", "Link::Material", Utils::OPTIONAL);
    if (tinyxml2::XMLElement* colorElement = xmlElement->FirstChildElement("color"))
        if (const char* color_str = colorElement->Attribute("rgba"))
            material.color = Color::parseFromString(color_str);
    return material;
}

void Material::addNewMaterialToList(std::vector<Material>& materials, tinyxml2::XMLElement* xmlElement) {
    if (!xmlElement) {
        LENNY_LOG_ERROR("Invalid xml element");
    }

    const Material newMaterial = Material::parseFromXmlElement(xmlElement);
    bool materialAlreadyExists = false;
    for (const Material& oldMaterial : materials) {
        if (newMaterial.name == oldMaterial.name) {
            materialAlreadyExists = true;
            break;
        }
    }
    if (!materialAlreadyExists && (newMaterial.color.has_value() || newMaterial.texture_filename.has_value()))
        materials.push_back(newMaterial);
}

}  // namespace lenny::urdf