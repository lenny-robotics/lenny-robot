#pragma once

#include <lenny/urdf/Utils.h>

#include <variant>

namespace lenny::urdf {

/**
 * Sphere
 */
struct Sphere {
public:
    double radius = 0.0;

    static Sphere parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
};

inline std::ostream& operator<<(std::ostream& out, const Sphere& s) {
    out << "sphere" << std::endl;
    out << "\t\t\tradius : " << s.radius << std::endl;
    return out;
}

/**
 * Box
 */
struct Box {
    Vector3 dim;

    static Box parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
};

inline std::ostream& operator<<(std::ostream& out, const Box& b) {
    out << "box" << std::endl;
    out << "\t\t\tdim: " << b.dim;
    return out;
}

/**
 * Cylinder
 */
struct Cylinder {
    double length = 0.0, radius = 0.0;

    static Cylinder parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
};

inline std::ostream& operator<<(std::ostream& out, const Cylinder& c) {
    out << "cylinder" << std::endl;
    out << "\t\t\tlength: " << c.length << std::endl;
    out << "\t\t\tradius: " << c.radius;
    return out;
}

/**
 * Capsule
 */
struct Capsule {
    double length = 0.0, radius = 0.0;

    static Capsule parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
};

inline std::ostream& operator<<(std::ostream& out, const Capsule& c) {
    out << "capsule" << std::endl;
    out << "\t\t\tlength: " << c.length << std::endl;
    out << "\t\t\tradius: " << c.radius;
    return out;
}

/**
 * Mesh
 */
struct Mesh {
    std::string filePath;
    Vector3 scale;

    Mesh() {
        scale.x = 1.0;
        scale.y = 1.0;
        scale.z = 1.0;
    }

    static Mesh parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
};

inline std::ostream& operator<<(std::ostream& out, const Mesh& m) {
    out << "mesh" << std::endl;
    out << "\t\t\tfilePath: " << m.filePath << std::endl;
    out << "\t\t\tscale: " << m.scale;
    return out;
}

/**
 * variant
 */
typedef std::variant<Sphere, Box, Cylinder, Capsule, Mesh> GeometryType;

/**
 * Geometry
 */
struct Geometry {
    static GeometryType parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
};

inline std::ostream& operator<<(std::ostream& out, const GeometryType& g) {
    std::visit([&](const auto& geometry) { out << geometry; }, g);
    return out;
}

}  // namespace lenny::urdf