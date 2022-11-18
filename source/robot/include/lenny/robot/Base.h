#pragma once

#include <lenny/robot/Utils.h>
#include <lenny/tools/EulerAngleRigidBody.h>

#include <array>

namespace lenny::robot {

class Base : public tools::EulerAngleRigidBody {
public:
    typedef std::unique_ptr<Base> UPtr;

    Base(const std::string& linkName);
    ~Base() = default;

    void drawGui();

    bool saveLimitsToFile(const std::string& filePath) const;
    bool loadLimitsFromFile(const char* filePath);

public:
    const std::string linkName;  //Base name

    enum DOFS { X, Y, Z, A, B, C };       //x, y, z, alpha, beta, gamma
    std::array<Limits, 6> posLimitsList;  //Pos limits
    std::array<Limits, 6> velLimitsList;  //Vel limits
    static constexpr auto dofNames = magic_enum::enum_names<Base::DOFS>();
};

}  // namespace lenny::robot