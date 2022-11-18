#pragma once

#include <lenny/collision/Parent.h>
#include <lenny/rapt/Agent.h>

/**
 * COMMENT: linkName is the parent description
 */

namespace lenny::rapt {

class AgentCollisionParent : public collision::Parent {
public:
    //--- Constructors
    AgentCollisionParent(const Agent& agent, const std::string& linkName);
    ~AgentCollisionParent() = default;

    //--- Compute point and its derivatives from primitive point
    Eigen::Vector3d computePoint(const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const override;
    void computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const override;
    void computePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& p_primitive) const override;

    //--- Compute vector and its derivatives from primitive vector
    Eigen::Vector3d computeVector(const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const override;
    void computeVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const override;
    void computeVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& v_primitive) const override;

    //--- Helpers
    int getStateDimension() const override;

private:
    const Agent& agent;
};

}  // namespace lenny::rapt