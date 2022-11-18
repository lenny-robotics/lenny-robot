#include <lenny/kineloco/LimbPositionConstraint.h>

namespace lenny::kineloco {

LimbPositionConstraint::LimbPositionConstraint(const TrackerAgenda& agenda) : optimization::EqualityConstraint("Limb Position"), agenda(agenda) {
    useTensorForHessian = false;
}

uint LimbPositionConstraint::getConstraintNumber() const {
    uint numC = 0;
    for (const TrackerAgenda::LimbTarget& target : agenda.limbTargets)
        numC += 3;
    return numC;
}

void LimbPositionConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    int iter = 0;
    for (const TrackerAgenda::LimbTarget& target : agenda.limbTargets) {
        C.segment(iter, 3) = agenda.computeGlobalPoint(q, target) - target.globalCoordinates;
        iter += 3;
    }
}

void LimbPositionConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    Eigen::TripletDList tripletDList;
    int iter = 0;
    for (const TrackerAgenda::LimbTarget& target : agenda.limbTargets) {
        Eigen::MatrixXd jacobian;
        agenda.computePointJacobian(jacobian, q, target);
        for (int k = 0; k < jacobian.outerSize(); ++k)
            for (Eigen::MatrixXd::InnerIterator it(jacobian, k); it; ++it)
                tools::utils::addTripletDToList(tripletDList, iter + it.row(), it.col(), it.value());
        iter += 3;
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

//COMMENT: Not implemented, since we don't use it in the optimization anyway
void LimbPositionConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();
}

}  // namespace lenny::kineloco