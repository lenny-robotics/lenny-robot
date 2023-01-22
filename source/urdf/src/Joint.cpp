#include <lenny/tools/Logger.h>
#include <lenny/urdf/Joint.h>

namespace lenny::urdf {

Joint::Dynamics Joint::Dynamics::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Joint::Dynamics dynamics;
    dynamics.damping = Utils::parseDoubleValueFromAttribute(xmlElement, "damping", "Joint::Dynamics", Utils::OPTIONAL);
    dynamics.friction = Utils::parseDoubleValueFromAttribute(xmlElement, "friction", "Joint::Dynamics", Utils::OPTIONAL);
    return dynamics;
}

Joint::Limits Joint::Limits::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Joint::Limits limits;
    limits.lower = Utils::parseDoubleValueFromAttribute(xmlElement, "lower", "Joint::Limits", Utils::OPTIONAL);
    limits.upper = Utils::parseDoubleValueFromAttribute(xmlElement, "upper", "Joint::Limits", Utils::OPTIONAL);
    limits.effort = Utils::parseDoubleValueFromAttribute(xmlElement, "effort", "Joint::Limits", Utils::OPTIONAL);
    limits.velocity = Utils::parseDoubleValueFromAttribute(xmlElement, "velocity", "Joint::Limits", Utils::OPTIONAL);
    limits.acceleration = Utils::parseDoubleValueFromAttribute(xmlElement, "acceleration", "Joint::Limits", Utils::OPTIONAL);
    return limits;
}

Joint::Safety Joint::Safety::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Joint::Safety safety;
    safety.upper_limit = Utils::parseDoubleValueFromAttribute(xmlElement, "upper_limit", "Joint::Safety", Utils::OPTIONAL);
    safety.lower_limit = Utils::parseDoubleValueFromAttribute(xmlElement, "lower_limit", "Joint::Safety", Utils::OPTIONAL);
    safety.k_position = Utils::parseDoubleValueFromAttribute(xmlElement, "k_position", "Joint::Safety", Utils::OPTIONAL);
    safety.k_velocity = Utils::parseDoubleValueFromAttribute(xmlElement, "k_velocity", "Joint::Safety", Utils::OPTIONAL);
    return safety;
}

Joint::Calibration Joint::Calibration::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Joint::Calibration calibration;
    calibration.rising = Utils::parseDoubleValueFromAttribute(xmlElement, "rising", "Joint::Calibration", Utils::OPTIONAL);
    calibration.falling = Utils::parseDoubleValueFromAttribute(xmlElement, "falling", "Joint::Calibration", Utils::OPTIONAL);
    return calibration;
}

Joint::Mimic Joint::Mimic::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Joint::Mimic mimic;
    mimic.joint_name = Utils::parseStringValueFromAttribute(xmlElement, "joint", "Joint::Mimic", Utils::MANDATORY).value();
    mimic.offset = Utils::parseDoubleValueFromAttribute(xmlElement, "offset", "Joint::Mimic", Utils::OPTIONAL);
    mimic.multiplier = Utils::parseDoubleValueFromAttribute(xmlElement, "multiplier", "Joint::Mimic", Utils::OPTIONAL);
    return mimic;
}

Joint Joint::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Joint joint;

    //name
    joint.name = Utils::parseStringValueFromAttribute(xmlElement, "name", "Joint", Utils::MANDATORY).value();

    //parent_to_joint_transform
    if (tinyxml2::XMLElement* originElement = xmlElement->FirstChildElement("origin")) {
        joint.parent_to_joint_transform = Transform::parseFromXmlElement(originElement);
    } else {
        LENNY_LOG_ERROR("No `origin` element in joint `%s`", joint.name.c_str());
    }

    //parent_link_name (root node might not have a parent...)
    if (tinyxml2::XMLElement* parentElement = xmlElement->FirstChildElement("parent")) {
        joint.parent_link_name = Utils::parseStringValueFromAttribute(parentElement, "link", "Joint::parent_link_name", Utils::MANDATORY).value();
    } else {
        LENNY_LOG_ERROR("No `parent` element in joint `%s`", joint.name.c_str());
    }

    //child_link_name (end effector node might not have a child...)
    if (tinyxml2::XMLElement* childElement = xmlElement->FirstChildElement("child")) {
        joint.child_link_name = Utils::parseStringValueFromAttribute(childElement, "link", "Joint::child_link_name", Utils::MANDATORY).value();
    } else {
        LENNY_LOG_ERROR("No `child` element in joint `%s`", joint.name.c_str());
    }

    //Type
    const std::optional<std::string> type_str = Utils::parseStringValueFromAttribute(xmlElement, "type", "Joint", Utils::MANDATORY);
    bool typeFound = false;
    for (int i = 0; i < joint.TYPE_STRING.size(); i++) {
        if (type_str.value() == joint.TYPE_STRING[i]) {
            joint.type = static_cast<TYPE>(i);
            typeFound = true;
            break;
        }
    }
    if (!typeFound) {
        LENNY_LOG_ERROR("Invalid joint type `%s`", type_str.value().c_str());
    }

    //Axis
    if (joint.type != FLOATING && joint.type != FIXED) {
        if (tinyxml2::XMLElement* axisElement = xmlElement->FirstChildElement("axis")) {
            const std::optional<std::string> axis_str = Utils::parseStringValueFromAttribute(axisElement, "xyz", "Joint::axis", Utils::MANDATORY);
            joint.axis = Vector3::parseFromString(axis_str.value());
        } else {
            LENNY_LOG_WARNING("No `axis` element in joint `%s`", joint.name.c_str());
        }
    }

    //Dynamics
    if (tinyxml2::XMLElement* dynamicsElement = xmlElement->FirstChildElement("dynamics"))
        joint.dynamics = Dynamics::parseFromXmlElement(dynamicsElement);

    //Limits
    if (tinyxml2::XMLElement* limitsElement = xmlElement->FirstChildElement("limit"))
        joint.limits = Limits::parseFromXmlElement(limitsElement);

    //Safety
    if (tinyxml2::XMLElement* safetyElement = xmlElement->FirstChildElement("safety_controller"))
        joint.safety = Safety::parseFromXmlElement(safetyElement);

    //Calibration
    if (tinyxml2::XMLElement* calibrationElement = xmlElement->FirstChildElement("calibration"))
        joint.calibration = Calibration::parseFromXmlElement(calibrationElement);

    //Mimic
    if (tinyxml2::XMLElement* mimicElement = xmlElement->FirstChildElement("mimic"))
        joint.mimic = Mimic::parseFromXmlElement(mimicElement);

    return joint;
}

}  // namespace lenny::urdf