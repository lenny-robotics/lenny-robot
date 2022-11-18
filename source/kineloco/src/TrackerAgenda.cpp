#include <lenny/kineloco/TrackerAgenda.h>

namespace lenny::kineloco {

uint TrackerAgenda::getJointSize() const {
    return robot.joints.size();
}

void TrackerAgenda::checkJoints(const Eigen::VectorXd& joints) const {
    if (joints.size() != getJointSize())
        LENNY_LOG_ERROR("Invalid joints input")
}

Eigen::VectorXd TrackerAgenda::getRobotState(const Eigen::VectorXd& joints) const {
    checkJoints(joints);
    Eigen::VectorXd robotState(robot.getStateSize());
    robotState << basePose, joints;
    return robotState;
}

void TrackerAgenda::setBasePose(const tools::Transformation& baseTrafo) {
    basePose = robot.base->getStateFromTransformation(baseTrafo);
}

Eigen::Vector3d TrackerAgenda::computeGlobalPoint(const Eigen::VectorXd& joints, const LimbTarget& limbTarget) const {
    return robot.computeGlobalPoint(getRobotState(joints), limbTarget.localCoordinates, limbTarget.linkName);
}

void TrackerAgenda::computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& joints, const LimbTarget& limbTarget) const {
    Eigen::VectorXb dofMask = Eigen::VectorXb::Ones(robot.getStateSize());
    dofMask.segment(0, 6).setZero();
    Eigen::MatrixXd fullJacobian(3, robot.getStateSize());
    robot.computePointJacobian(fullJacobian, getRobotState(joints), limbTarget.localCoordinates, limbTarget.linkName, dofMask);
    jacobian = fullJacobian.block(0, 6, 3, robot.getStateSize() - 6);
}

const robot::Limits& TrackerAgenda::getAngleLimitsForJointIndex(const uint& jointIndex) const {
    return robot.getLimitsForDofIndex(jointIndex + 6, robot::Robot::POSITION);
}

}  // namespace lenny::kineloco