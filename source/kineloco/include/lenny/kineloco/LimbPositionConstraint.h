#pragma once

#include <lenny/kineloco/TrackerAgenda.h>
#include <lenny/optimization/EqualityConstraint.h>

namespace lenny::kineloco {

class LimbPositionConstraint : public optimization::EqualityConstraint {
public:
    LimbPositionConstraint(const TrackerAgenda& agenda);
    ~LimbPositionConstraint() = default;

    uint getConstraintNumber() const override;
    void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const override;
    void computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const override;
    void computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const override;

private:
    const TrackerAgenda& agenda;
};

}  // namespace lenny::kineloco