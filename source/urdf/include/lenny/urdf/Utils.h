#pragma once
#pragma GCC diagnostic ignored "-Wformat-security"

#include <tinyxml2.h>

#include <ostream>
#include <optional>
#include <string>
#include <vector>

namespace lenny::urdf {

/**
 * Utils
 */
class Utils {
private:  //Private, since this is a purely static class
    Utils() = default;
    ~Utils() = default;

public:
    template <class Type>
    static void print(std::ostream& out, const Type& value, const std::string& description, const bool newLineBefore = false) {
        if (newLineBefore)
            out << description << ": " << std::endl << value;
        else
            out << description << ": " << value << std::endl;
    }

    template <class Type>
    static void print(std::ostream& out, const std::optional<Type>& value, const std::string& description, const bool newLineBefore = false) {
        if (value.has_value())
            print(out, value.value(), description, newLineBefore);
    }

    static void splitStringBySpace(std::vector<std::string>& list, const std::string& text);
    static double convertStringToDouble(const std::string& value, const std::string& description);
    static void parseDoubleListFromString(std::vector<double>& values, const std::string& text, const std::string& description, const int expectedNumValues);

    static bool showExpectedWarnings;
    enum LEVEL { OPTIONAL, EXPECTED, MANDATORY };
    static std::optional<std::string> parseStringValueFromAttribute(tinyxml2::XMLElement* xmlElement, const char* attribute, const char* description,
                                                                    const LEVEL level);
    static std::optional<double> parseDoubleValueFromAttribute(tinyxml2::XMLElement* xmlElement, const char* attribute, const char* description,
                                                               const LEVEL level);
};

/**
 * Vector3d
 */
struct Vector3 {
    double x = 0.0, y = 0.0, z = 0.0;

    static Vector3 parseFromString(const std::string& text);
};

inline std::ostream& operator<<(std::ostream& out, const Vector3& v) {
    out << "x: " << v.x << ", y: " << v.y << ", z: " << v.z;
    return out;
}

/**
 * Rotation
 */
struct Rotation {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    static Rotation parseFromString(const std::string& text);
};

inline std::ostream& operator<<(std::ostream& out, const Rotation& r) {
    out << "roll: " << r.roll << ", pitch: " << r.pitch << ", yaw: " << r.yaw;
    return out;
}

/**
 * Transform
 */
struct Transform {
    Vector3 position;
    Rotation rotation;

    static Transform parseFromXmlElement(tinyxml2::XMLElement* xmlElement);
};

inline std::ostream& operator<<(std::ostream& out, const Transform& t) {
    out << t.position << ", " << t.rotation;
    return out;
}

/**
 * Color
 */
struct Color {
    double r = 0.0, g = 0.0, b = 0.0, a = 0.0;

    static Color parseFromString(const std::string& text);
};

inline std::ostream& operator<<(std::ostream& out, const Color& c) {
    out << "r: " << c.r << ", g: " << c.g << ", b: " << c.b << ", a: " << c.a;
    return out;
}

}  // namespace lenny::urdf