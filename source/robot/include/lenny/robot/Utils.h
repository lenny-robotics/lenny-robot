#pragma once

#include <lenny/tools/Model.h>
#include <lenny/tools/Transformation.h>

//--- Dof mask
namespace Eigen {
typedef Eigen::Matrix<bool, -1, 1> VectorXb;
}  // namespace Eigen

namespace lenny::robot {
//--- Limits
typedef std::optional<std::pair<double, double>> Limits;  //[lower, upper]
void drawLimitsGui(const std::string& description, Limits& limits);

//--- Visual
class Visual {
public:
    Visual(const std::string& filePath);
    Visual(Visual&&) = default;
    ~Visual() = default;

    void drawGui(const std::string& description);

public:
    const std::string filePath;
    tools::Model::UPtr model;
    tools::Transformation localTrafo;
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    std::optional<Eigen::Vector3d> color;
};

}  // namespace lenny::robot