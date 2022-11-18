#include <lenny/tools/Logger.h>
#include <lenny/urdf/Geometry.h>

namespace lenny::urdf {

Sphere Sphere::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Sphere sphere;
    sphere.radius = Utils::parseDoubleValueFromAttribute(xmlElement, "radius", "Geometry::Sphere", Utils::MANDATORY).value();
    return sphere;
}

Box Box::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Box box;
    if (const char* dim_str = xmlElement->Attribute("size")) {
        box.dim = Vector3::parseFromString(dim_str);
    } else {
        LENNY_LOG_ERROR("Attribute `size` does not exist");
    }
    return box;
}

Cylinder Cylinder::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Cylinder cylinder;
    cylinder.length = Utils::parseDoubleValueFromAttribute(xmlElement, "length", "Geometry::Cylinder", Utils::MANDATORY).value();
    cylinder.radius = Utils::parseDoubleValueFromAttribute(xmlElement, "radius", "Geometry::Cylinder", Utils::MANDATORY).value();
    return cylinder;
}

Capsule Capsule::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Capsule capsule;
    capsule.length = Utils::parseDoubleValueFromAttribute(xmlElement, "length", "Geometry::Capsule", Utils::MANDATORY).value();
    capsule.radius = Utils::parseDoubleValueFromAttribute(xmlElement, "radius", "Geometry::Capsule", Utils::MANDATORY).value();
    return capsule;
}

Mesh Mesh::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Mesh mesh;
    mesh.filePath = Utils::parseStringValueFromAttribute(xmlElement, "filename", "Geometry::Mesh", Utils::MANDATORY).value();
    if (const char* scale_str = xmlElement->Attribute("scale"))
        mesh.scale = Vector3::parseFromString(scale_str);
    else {
        mesh.scale.x = 1.0;
        mesh.scale.y = 1.0;
        mesh.scale.z = 1.0;
    }
    return mesh;
}

GeometryType Geometry::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    if (!xmlElement) {
        LENNY_LOG_ERROR("Passed invalid xml element");
    }

    if (tinyxml2::XMLElement* shapeElement = xmlElement->FirstChildElement()) {
        const std::string type_str = shapeElement->Value();
        if (type_str == "sphere")
            return Sphere::parseFromXmlElement(shapeElement);
        else if (type_str == "box")
            return Box::parseFromXmlElement(shapeElement);
        else if (type_str == "cylinder")
            return Cylinder::parseFromXmlElement(shapeElement);
        else if (type_str == "capsule")
            return Capsule::parseFromXmlElement(shapeElement);
        else if (type_str == "mesh")
            return Mesh::parseFromXmlElement(shapeElement);
        else {
            LENNY_LOG_ERROR("Unknown shape type `%s`", type_str.c_str());
        }
    } else {
        LENNY_LOG_ERROR("Geometry does not contain any shape information");
    }
    return Mesh();  //Dummy
}

}  // namespace lenny::urdf