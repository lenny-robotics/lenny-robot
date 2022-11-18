#pragma once

#include <lenny/urdf/Utils.h>

namespace lenny::urdf {

struct Material {
    std::string name;
    std::optional<std::string> texture_filename = std::nullopt;
    std::optional<Color> color = std::nullopt;

    static Material parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
    static void addNewMaterialToList(std::vector<Material>& materials, tinyxml2::XMLElement* xmlElement);
};

inline std::ostream& operator<<(std::ostream& out, const Material& m) {
    out << "Material" << std::endl;
    Utils::print(out, m.name, "\tname");
    Utils::print(out, m.texture_filename, "\ttexture_filename");
    Utils::print(out, m.color, "\tcolor");
    return out;
}

}  // namespace lenny::urdf