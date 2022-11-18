#pragma once

#include <lenny/robot/Utils.h>

namespace lenny::robot {

class Link {
public:
    Link() = default;
    Link(Link&&) = default;
    ~Link() = default;

    void drawGui(const std::string& name);

public:
    std::vector<Visual> visuals;  //List of visuals

    static Eigen::Vector3d skeletonColor;  //Drawing color of skeleton
};

}  // namespace lenny::robot