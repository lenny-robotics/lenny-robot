#pragma once

#include <lenny/urdf/Geometry.h>
#include <lenny/urdf/Joint.h>

namespace lenny::urdf {

class Link {
public:
    //--- Member structs
    struct Inertial {
        Transform origin;
        double mass, ixx, ixy, ixz, iyy, iyz, izz;

        static Inertial parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
    };

    struct Visual {
        std::optional<std::string> name = std::nullopt, material_name = std::nullopt;
        std::optional<Transform> origin = std::nullopt;
        std::optional<GeometryType> geometry = std::nullopt;

        static Visual parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
    };

    struct Collision {
        std::optional<std::string> name = std::nullopt;
        std::optional<Transform> origin = std::nullopt;
        std::optional<GeometryType> geometry = std::nullopt;

        static Collision parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
    };

    //--- Constructor
    Link() = default;
    ~Link() = default;

    //--- Parser
    static Link parseFromXmlElement(tinyxml2::XMLElement* xmlElement);

public:
    //--- Members
    std::string name;
    std::optional<Inertial> inertial = std::nullopt;
    std::vector<Visual> visuals = {};
    std::vector<Collision> collisions = {};
};

inline std::ostream& operator<<(std::ostream& out, const Link::Inertial& i) {
    Utils::print(out, i.origin, "\t\torigin");
    Utils::print(out, i.mass, "\t\tmass");
    Utils::print(out, i.ixx, "\t\tixx");
    Utils::print(out, i.ixy, "\t\tixy");
    Utils::print(out, i.ixz, "\t\tixz");
    Utils::print(out, i.iyy, "\t\tiyy");
    Utils::print(out, i.iyz, "\t\tiyz");
    Utils::print(out, i.izz, "\t\tizz");
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const Link::Visual& v) {
    Utils::print(out, v.name, "\t\tname");
    Utils::print(out, v.material_name, "\t\tmaterial_name");
    Utils::print(out, v.origin, "\t\torigin");
    Utils::print(out, v.geometry, "\t\tgeometry");
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const Link::Collision& c) {
    Utils::print(out, c.name, "\t\tname");
    Utils::print(out, c.origin, "\t\torigin");
    Utils::print(out, c.geometry, "\t\tgeometry");
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const Link& l) {
    out << "Link" << std::endl;
    Utils::print(out, l.name, "\tname");
    Utils::print(out, l.inertial, "\tinertial", true);
    for (const Link::Visual& visual : l.visuals)
        out << "\tvisual: " << std::endl << visual;
    for (const Link::Collision& collision : l.collisions)
        out << "\tcollision: " << std::endl << collision;
    return out;
}

}  // namespace lenny::urdf