#pragma once

#include <lenny/urdf/Utils.h>

#include <array>

namespace lenny::urdf {

class Joint {
public:
    //--- Member structs
    struct Dynamics {
        std::optional<double> damping = std::nullopt, friction = std::nullopt;

        static Dynamics parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
    };

    struct Limits {
        std::optional<double> lower = std::nullopt, upper = std::nullopt, effort = std::nullopt, velocity = std::nullopt, acceleration = std::nullopt;

        static Limits parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
    };

    struct Safety {
        std::optional<double> upper_limit = std::nullopt, lower_limit = std::nullopt, k_position = std::nullopt, k_velocity = std::nullopt;

        static Safety parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
    };

    struct Calibration {
        std::optional<double> rising = std::nullopt, falling = std::nullopt;

        static Calibration parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
    };

    struct Mimic {
        std::string joint_name;
        std::optional<double> offset = std::nullopt, multiplier = std::nullopt;

        static Mimic parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
    };

    //--- Types
    enum TYPE { REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED };
    const std::array<std::string, 6> TYPE_STRING = {"revolute", "continuous", "prismatic", "floating", "planar", "fixed"};

    //--- Constructor
    Joint() = default;
    ~Joint() = default;

    //--- Parser
    static Joint parseFromXmlElement(tinyxml2::XMLElement* xmlElement);

public:
    //--- Members
    std::string name, child_link_name, parent_link_name;
    Transform parent_to_joint_transform;
    TYPE type;
    std::optional<Vector3> axis = std::nullopt;
    std::optional<Dynamics> dynamics = std::nullopt;
    std::optional<Limits> limits = std::nullopt;
    std::optional<Safety> safety = std::nullopt;
    std::optional<Calibration> calibration = std::nullopt;
    std::optional<Mimic> mimic = std::nullopt;
};

inline std::ostream& operator<<(std::ostream& out, const Joint::Dynamics& d) {
    Utils::print(out, d.damping, "\t\tdamping");
    Utils::print(out, d.friction, "\t\tfriction");
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const Joint::Limits& l) {
    Utils::print(out, l.lower, "\t\tlower");
    Utils::print(out, l.upper, "\t\tupper");
    Utils::print(out, l.effort, "\t\teffort");
    Utils::print(out, l.velocity, "\t\tvelocity");
    Utils::print(out, l.acceleration, "\t\tacceleration");
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const Joint::Safety& s) {
    Utils::print(out, s.upper_limit, "\t\tupper_limit");
    Utils::print(out, s.lower_limit, "\t\tlower_limit");
    Utils::print(out, s.k_position, "\t\tk_position");
    Utils::print(out, s.k_velocity, "\t\tk_velocity");
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const Joint::Calibration& c) {
    Utils::print(out, c.rising, "\t\trising");
    Utils::print(out, c.falling, "\t\tfalling");
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const Joint::Mimic& m) {
    Utils::print(out, m.joint_name, "\t\tjoint_name");
    Utils::print(out, m.offset, "\t\toffset");
    Utils::print(out, m.multiplier, "\t\tmultiplier");
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const Joint& j) {
    out << "Joint" << std::endl;
    Utils::print(out, j.name, "\tname");
    Utils::print(out, j.child_link_name, "\tchild_link_name");
    Utils::print(out, j.parent_link_name, "\tparent_link_name");
    out << "\ttype: " << j.TYPE_STRING[j.type] << std::endl;
    Utils::print(out, j.axis, "\taxis");
    Utils::print(out, j.parent_to_joint_transform, "\tparent_to_joint_transform");
    Utils::print(out, j.dynamics, "\tdynamics", true);
    Utils::print(out, j.limits, "\tlimits", true);
    Utils::print(out, j.safety, "\tsafety", true);
    Utils::print(out, j.calibration, "\tcalibration", true);
    Utils::print(out, j.mimic, "\tmimic", true);
    return out;
}

}  // namespace lenny::urdf