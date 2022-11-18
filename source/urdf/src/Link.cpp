#include <lenny/tools/Logger.h>
#include <lenny/urdf/Link.h>

namespace lenny::urdf {

Link::Inertial Link::Inertial::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Link::Inertial inertial;
    if (tinyxml2::XMLElement* originElement = xmlElement->FirstChildElement("origin"))
        inertial.origin = Transform::parseFromXmlElement(originElement);
    else {
        LENNY_LOG_ERROR("`origin` element not available")
    }

    if (tinyxml2::XMLElement* massElement = xmlElement->FirstChildElement("mass")) {
        inertial.mass = Utils::parseDoubleValueFromAttribute(massElement, "value", "Link::Inertial", Utils::MANDATORY).value();
    } else {
        LENNY_LOG_ERROR("`mass` element not available");
    }

    if (tinyxml2::XMLElement* inertiaElement = xmlElement->FirstChildElement("inertia")) {
        inertial.ixx = Utils::parseDoubleValueFromAttribute(inertiaElement, "ixx", "Link::Inertial", Utils::MANDATORY).value();
        inertial.ixy = Utils::parseDoubleValueFromAttribute(inertiaElement, "ixy", "Link::Inertial", Utils::MANDATORY).value();
        inertial.ixz = Utils::parseDoubleValueFromAttribute(inertiaElement, "ixz", "Link::Inertial", Utils::MANDATORY).value();
        inertial.iyy = Utils::parseDoubleValueFromAttribute(inertiaElement, "iyy", "Link::Inertial", Utils::MANDATORY).value();
        inertial.iyz = Utils::parseDoubleValueFromAttribute(inertiaElement, "iyz", "Link::Inertial", Utils::MANDATORY).value();
        inertial.izz = Utils::parseDoubleValueFromAttribute(inertiaElement, "izz", "Link::Inertial", Utils::MANDATORY).value();
    } else {
        LENNY_LOG_ERROR("Inertia element not available");
    }

    return inertial;
}

Link::Visual Link::Visual::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Link::Visual visual;
    visual.name = Utils::parseStringValueFromAttribute(xmlElement, "name", "Link::Visual", Utils::OPTIONAL);
    if (tinyxml2::XMLElement* originElement = xmlElement->FirstChildElement("origin"))
        visual.origin = Transform::parseFromXmlElement(originElement);
    if (tinyxml2::XMLElement* geometryElement = xmlElement->FirstChildElement("geometry"))
        visual.geometry = Geometry::parseFromXmlElement(geometryElement);
    if (tinyxml2::XMLElement* materialElement = xmlElement->FirstChildElement("material"))
        visual.material_name = Utils::parseStringValueFromAttribute(materialElement, "name", "Link::Visual::material_name", Utils::OPTIONAL);
    return visual;
}

Link::Collision Link::Collision::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Link::Collision collision;
    collision.name = Utils::parseStringValueFromAttribute(xmlElement, "name", "Link::Collision", Utils::OPTIONAL);
    if (tinyxml2::XMLElement* originElement = xmlElement->FirstChildElement("origin"))
        collision.origin = Transform::parseFromXmlElement(originElement);
    if (tinyxml2::XMLElement* geometryElement = xmlElement->FirstChildElement("geometry"))
        collision.geometry = Geometry::parseFromXmlElement(geometryElement);
    return collision;
}

Link Link::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Link link;
    link.name = Utils::parseStringValueFromAttribute(xmlElement, "name", "Link", Utils::MANDATORY).value();
    if (tinyxml2::XMLElement* inertialElement = xmlElement->FirstChildElement("inertial"))
        link.inertial = Inertial::parseFromXmlElement(inertialElement);
    for (tinyxml2::XMLElement* visualElement = xmlElement->FirstChildElement("visual"); visualElement != nullptr;
         visualElement = visualElement->NextSiblingElement("visual"))
        link.visuals.emplace_back(Visual::parseFromXmlElement(visualElement));
    for (tinyxml2::XMLElement* collisionElement = xmlElement->FirstChildElement("collision"); collisionElement != nullptr;
         collisionElement = collisionElement->NextSiblingElement("collision"))
        link.collisions.emplace_back(Collision::parseFromXmlElement(collisionElement));
    return link;
}

}  // namespace lenny::urdf