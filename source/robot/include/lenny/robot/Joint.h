#pragma once

#include <lenny/robot/Utils.h>

namespace lenny::robot {

class Joint {
public:
    Joint(const std::string& parentName, const std::string& childName) : parentName(parentName), childName(childName) {}
    ~Joint() = default;

    void drawGui(const std::string& name);

public:
    const std::string parentName, childName;          //Parent and child link name (child position is always zero)
    Eigen::Vector3d pJPos = Eigen::Vector3d::Zero();  //Location of the joint on the parent expressed in the parent's local coordinates
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();  //Local coordinates of rotation axis (same for child and parent)
    Limits angleLimits, velLimits;                    //Joint limits
    static Eigen::Vector3d skeletonColor;             //Drawing color of skeleton
};

}  // namespace lenny::robot