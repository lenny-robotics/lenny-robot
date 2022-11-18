#include <lenny/rapt/AgentCollisionParent.h>

namespace lenny::rapt {

AgentCollisionParent::AgentCollisionParent(const Agent& agent, const std::string& linkName) : collision::Parent(linkName), agent(agent) {
    if (agent.robot.links.find(linkName) == agent.robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent.name.c_str());
}

Eigen::Vector3d AgentCollisionParent::computePoint(const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    return agent.computeGlobalPoint(state, p_primitive, description);
}

void AgentCollisionParent::computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    agent.computePointJacobian(jacobian, state, p_primitive, description);
}

void AgentCollisionParent::computePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const {
    if (useTensor) {
        agent.computePointTensor(tensor, state, p_primitive, description);
    } else {
        tensor.resize(Eigen::Vector3i(3, state.size(), state.size()));
        tensor.setZero();
    }
}

Eigen::Vector3d AgentCollisionParent::computeVector(const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    return agent.computeGlobalVector(state, v_primitive, description);
}

void AgentCollisionParent::computeVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    agent.computeVectorJacobian(jacobian, state, v_primitive, description);
}

void AgentCollisionParent::computeVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const {
    if (useTensor) {
        agent.computeVectorTensor(tensor, state, v_primitive, description);
    } else {
        tensor.resize(Eigen::Vector3i(3, state.size(), state.size()));
        tensor.setZero();
    }
}

int AgentCollisionParent::getStateDimension() const {
    return agent.getStateSize();
}

}  // namespace lenny::rapt