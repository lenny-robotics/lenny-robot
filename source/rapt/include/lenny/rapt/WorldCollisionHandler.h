#pragma once

#include <lenny/rapt/WorldCollisionParent.h>
#include <lenny/collision/Primitive.h>

namespace lenny::rapt {

class WorldCollisionHandler {
public:
    //--- Constructor
    WorldCollisionHandler() = default;
    ~WorldCollisionHandler() = default;

    //--- Drawing
    void drawScene() const;
    void drawGui();

    //--- Helpers
    tools::Transformation convertPose(const Eigen::Vector6d& state) const;
    Eigen::Vector6d convertPose(const tools::Transformation& trafo) const;

    //--- Adders
    void addCollisionSphere(const Eigen::Vector6d& state, const double& radius);
    void addCollisionCapsule(const Eigen::Vector6d& state, const double& length, const double& radius);
    void addCollisionRectangle(const Eigen::Vector6d& state, const Eigen::Vector2d& dimensions, const double& safetyMargin);
    void addCollisionBox(const Eigen::Vector6d& state, const Eigen::Vector3d& dimensions, const double& safetyMargin);

    //--- Save & Load
    bool saveCollisionPrimitivesToFile(const std::string& filePath) const;
    bool loadCollisionPrimitivesFromFile(const char* filePath);

public:
    typedef std::pair<collision::Primitive::SPtr, Eigen::Vector6d> PrimitiveInfo;  //[primitive, parentState]
    typedef std::vector<PrimitiveInfo> PrimitiveList;
    PrimitiveList primitives;

private:
    static WorldCollisionParent::SPtr parent;
};

}  // namespace lenny::agent
