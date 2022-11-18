#pragma once

#include <lenny/kineloco/TrackerAgenda.h>
#include <lenny/optimization/InequalityConstraint.h>

namespace lenny::kineloco {

class JointAngleConstraint : public optimization::InequalityConstraint {
public:
    JointAngleConstraint(const TrackerAgenda& agenda);
    ~JointAngleConstraint() = default;

    uint getConstraintNumber() const override;
    void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const override;
    void computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const override;
    void computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const override;

public:
    double testFactor = 0.1;

private:
    const TrackerAgenda& agenda;
};

}  // namespace lenny::kineloco