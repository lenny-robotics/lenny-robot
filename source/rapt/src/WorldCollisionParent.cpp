#include <lenny/rapt/WorldCollisionParent.h>

namespace lenny::rapt {

tools::EulerAngleRigidBody WorldCollisionParent::rigidBody;

WorldCollisionParent::WorldCollisionParent() : collision::Parent("World Collision Parent") {}

Eigen::Vector3d WorldCollisionParent::computePoint(const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    return rigidBody.computeGlobalPoint(state, p_primitive);
}

void WorldCollisionParent::computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    //We don't need the derivatives for this
    jacobian.resize(3, state.size());
    jacobian.setZero();
}

void WorldCollisionParent::computePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    //We don't need the derivatives for this
    tensor.resize(Eigen::Vector3i(3, state.size(), state.size()));
    tensor.setZero();
}

Eigen::Vector3d WorldCollisionParent::computeVector(const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    return rigidBody.computeGlobalVector(state, v_primitive);
}

void WorldCollisionParent::computeVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    //We don't need the derivatives for this
    jacobian.resize(3, state.size());
    jacobian.setZero();
}

void WorldCollisionParent::computeVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    //We don't need the derivatives for this
    tensor.resize(Eigen::Vector3i(3, state.size(), state.size()));
    tensor.setZero();
}

int WorldCollisionParent::getStateDimension() const {
    return rigidBody.STATE_SIZE;
}

}  // namespace lenny::rapt