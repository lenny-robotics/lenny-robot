#pragma once

#include <lenny/tools/Model.h>
#include <lenny/tools/Transformation.h>

//--- Dof mask
namespace Eigen {
typedef Eigen::Matrix<bool, -1, 1> VectorXb;
}  // namespace Eigen

//------------------------------------------------------------------------------------------------------------

namespace lenny::robot {
//--- Limits
typedef std::optional<std::pair<double, double>> Limits;  //[lower, upper]
void drawLimitsGui(const std::string& description, Limits& limits);

//------------------------------------------------------------------------------------------------------------

//--- Visual
class Visual {
public:
    Visual(const std::string& filePath, const tools::Model::F_loadModel& f_loadModel, const tools::Transformation& localTrafo, const Eigen::Vector3d& scale,
           const std::optional<Eigen::Vector3d>& color);
    Visual(Visual&&) = default;
    ~Visual() = default;

    void drawScene(const tools::Transformation& globalPose, const std::optional<Eigen::Vector3d>& color, const double& alpha) const;
    void drawGui(const std::string& description);

public:
    const std::string filePath;
    tools::Model::UPtr model;
    tools::Transformation localTrafo;
    Eigen::Vector3d scale;
    std::optional<Eigen::Vector3d> color;
};

}  // namespace lenny::robot