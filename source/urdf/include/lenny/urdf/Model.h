#pragma once

#include <lenny/urdf/Joint.h>
#include <lenny/urdf/Link.h>
#include <lenny/urdf/Material.h>

namespace lenny::urdf {

class Model {
public:
    Model(const std::string& filePath);
    ~Model() = default;

private:
    void loadFromFile(const std::string& filePath);
    void applySanityChecks() const;

public:
    std::optional<std::string> name;
    std::vector<Link> links;
    std::vector<Joint> joints;
    std::vector<Material> materials;
};

inline std::ostream& operator<<(std::ostream& out, const Model& m) {
    Utils::print(out, m.name, "Name");
    out << std::endl;
    for (const Link& l : m.links)
        out << l << std::endl;
    for (const Joint& j : m.joints)
        out << j << std::endl;
    for (const Material& m : m.materials)
        out << m << std::endl;
    return out;
}

}  // namespace lenny::urdf