#pragma once

#include <lenny/robot/Robot.h>

namespace lenny::kineloco {

class TrackerAgenda {
public:
    struct LimbTarget {
        const std::string linkName;
        const Eigen::Vector3d localCoordinates;
        Eigen::Vector3d globalCoordinates;
    };

    TrackerAgenda(const robot::Robot& robot) : robot(robot) {}
    ~TrackerAgenda() = default;

    uint getJointSize() const;
    void checkJoints(const Eigen::VectorXd& joints) const;
    Eigen::VectorXd getRobotState(const Eigen::VectorXd& joints) const;
    void setBasePose(const tools::Transformation& baseTrafo);

    Eigen::Vector3d computeGlobalPoint(const Eigen::VectorXd& joints, const LimbTarget& linkTarget) const;
    void computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& joints, const LimbTarget& linkTarget) const;
    const robot::Limits& getAngleLimitsForJointIndex(const uint& jointIndex) const;

public:
    const robot::Robot& robot;
    Eigen::Vector6d basePose;
    std::vector<LimbTarget> limbTargets;
};

}  // namespace lenny::kineloco