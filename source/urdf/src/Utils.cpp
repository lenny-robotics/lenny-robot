#include <lenny/tools/Logger.h>
#include <lenny/urdf/Utils.h>

#include <iterator>
#include <sstream>

namespace lenny::urdf {

/**
 * Utils
 */
void Utils::splitStringBySpace(std::vector<std::string>& list, const std::string& text) {
    std::stringstream ss(text);
    std::istream_iterator<std::string> begin(ss), end;
    list = std::vector<std::string>(begin, end);
}

double Utils::convertStringToDouble(const std::string& value, const std::string& description) {
    double dVal = 0.0;
    try {
        dVal = std::stod(value);
    } catch (const std::invalid_argument& error) {
        LENNY_LOG_ERROR("(%s): Unable to parse component `%s` to a double -> %s", description.c_str(), value.c_str(), error.what());
    } catch (const std::out_of_range& error) {
        LENNY_LOG_ERROR("(%s): Unable to parse component `%s` to a double -> %s", description.c_str(), value.c_str(), error.what());
    }
    return dVal;
}

void Utils::parseDoubleListFromString(std::vector<double>& values, const std::string& text, const std::string& description, const int expectedNumValues) {
    //--- Split test into list
    std::vector<std::string> valueStringList;
    splitStringBySpace(valueStringList, text);

    //--- Parse values
    values.clear();
    for (const std::string& valueString : valueStringList)
        if (valueString != "")
            values.push_back(convertStringToDouble(valueString, description));

    //--- Check size
    if (values.size() != expectedNumValues) {
        LENNY_LOG_ERROR("(%s): Expected %d elements, but instead found %d", description.c_str(), expectedNumValues, values.size());
    }
}

bool Utils::showExpectedWarnings = true;

std::optional<std::string> Utils::parseStringValueFromAttribute(tinyxml2::XMLElement* xmlElement, const char* attribute, const char* description,
                                                                const LEVEL level) {
    if (!xmlElement) {
        LENNY_LOG_ERROR("Passed invalid xml element");
    }

    std::optional<std::string> value = std::nullopt;
    if (const char* str = xmlElement->Attribute(attribute)) {
        value = std::string(str);
    } else {
        if (level == EXPECTED && showExpectedWarnings) {
            LENNY_LOG_WARNING("(%s) `%s` attribute not available", description, attribute);
        } else if (level == MANDATORY) {
            LENNY_LOG_ERROR("(%s) `%s` attribute not available", description, attribute);
        }
    }
    return value;
}

std::optional<double> Utils::parseDoubleValueFromAttribute(tinyxml2::XMLElement* xmlElement, const char* attribute, const char* description,
                                                           const LEVEL level) {
    if (!xmlElement) {
        LENNY_LOG_ERROR("Passed invalid xml element");
    }

    std::optional<double> value = std::nullopt;
    if (const char* str = xmlElement->Attribute(attribute)) {
        value = Utils::convertStringToDouble(str, attribute);
    } else {
        if (level == EXPECTED && showExpectedWarnings) {
            LENNY_LOG_WARNING("(%s) `%s` attribute not available", description, attribute);
        } else if (level == MANDATORY) {
            LENNY_LOG_ERROR("(%s) `%s` attribute not available", description, attribute);
        }
    }
    return value;
}

/**
 * Vector3
 */
Vector3 Vector3::parseFromString(const std::string& text) {
    std::vector<double> values;
    Utils::parseDoubleListFromString(values, text, "Vector3", 3);
    Vector3 v;
    v.x = values[0];
    v.y = values[1];
    v.z = values[2];
    return v;
}

/**
 * Rotation
 */
Rotation Rotation::parseFromString(const std::string& text) {
    std::vector<double> values;
    Utils::parseDoubleListFromString(values, text, "Rotation", 3);
    Rotation rotation;
    rotation.roll = values[0];
    rotation.pitch = values[1];
    rotation.yaw = values[2];
    return rotation;
}

/**
 * Transform
 */
Transform Transform::parseFromXmlElement(tinyxml2::XMLElement* xmlElement) {
    Transform transform;
    if (xmlElement) {
        //Position
        const char* xyz_str = xmlElement->Attribute("xyz");
        if (xyz_str != nullptr)
            transform.position = Vector3::parseFromString(xyz_str);

        //Rotation
        const char* rpy_str = xmlElement->Attribute("rpy");
        if (rpy_str != NULL)
            transform.rotation = Rotation::parseFromString(rpy_str);
    } else {
        LENNY_LOG_ERROR("Passed invalid xml element");
    }
    return transform;
}

/**
 * Color
 */
Color Color::parseFromString(const std::string& text) {
    std::vector<double> values;
    Utils::parseDoubleListFromString(values, text, "Color", 4);
    Color color;
    color.r = values[0];
    color.g = values[1];
    color.b = values[2];
    color.a = values[3];
    return color;
}

}  // namespace lenny::urdf