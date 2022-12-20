#include <lenny/kineloco/JointAngleConstraint.h>
#include <lenny/tools/Gui.h>

namespace lenny::kineloco {

JointAngleConstraint::JointAngleConstraint(const TrackerAgenda& agenda) : optimization::InequalityConstraint("Joint Angle Limits"), agenda(agenda) {
    useTensorForHessian = false;
    barrier.setEpsilon(0.0);
    barrier.setStiffness(1.0);
    softificationWeights.setOnes(2 * 3 * 4);  //Initialize this with 2 constraints x 3 dofs x 4 legs
}

uint JointAngleConstraint::getConstraintNumber() const {
    uint numActiveLimits = 0;
    for (uint i = 0; i < agenda.getJointSize(); i++)
        if (agenda.getAngleLimitsForJointIndex(i))
            numActiveLimits += 2;
    return numActiveLimits;
}

void JointAngleConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    C.setZero();  //Important!
    if (C.size() == 0)
        return;

    uint iter = 0;
    for (uint i = 0; i < agenda.getJointSize(); i++) {
        const robot::Limits& limit = agenda.getAngleLimitsForJointIndex(i);
        if (!limit.has_value())
            continue;
        C[iter++] = limit->first - q[i];
        C[iter++] = q[i] - limit->second;
    }
}

void JointAngleConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    pCpQ.setZero();
    if (pCpQ.rows() == 0)
        return;

    Eigen::TripletDList tripletDList;
    uint iter = 0;
    for (uint i = 0; i < agenda.getJointSize(); i++) {
        const robot::Limits& limit = agenda.getAngleLimitsForJointIndex(i);
        if (!limit.has_value())
            continue;
        tools::utils::addTripletDToList(tripletDList, iter++, i, -1.0);
        tools::utils::addTripletDToList(tripletDList, iter++, i, 1.0);
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void JointAngleConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();
}

}  // namespace lenny::kineloco