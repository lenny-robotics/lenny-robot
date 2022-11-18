#include <lenny/kineloco/JointAngleConstraint.h>
#include <lenny/kineloco/LimbPositionConstraint.h>
#include <lenny/kineloco/TrajectoryTracker.h>

namespace lenny::kineloco {

TrajectoryTracker::TrajectoryTracker(const robot::Robot& robot, const Eigen::VectorXd& initialRobotState)
    : agenda(robot), objective("KineLoco Objective"), optimizer("KineLoco Optimizer") {
    //--- Initialize objectives
    objective.subObjectives.clear();
    objective.subObjectives.emplace_back(std::make_pair(std::make_unique<LimbPositionConstraint>(agenda), 1.0));
    objective.subObjectives.emplace_back(std::make_pair(std::make_unique<JointAngleConstraint>(agenda), 10.0));

    //--- Initialize vectors
    agenda.basePose = initialRobotState.segment(0, 6);
    joints = initialRobotState.segment(6, robot.getStateSize() - 6);
}

bool TrajectoryTracker::solve(const uint& numSteps) {
    if (checkDerivatives) {
        objective.testIndividualFirstDerivatives(joints);
        objective.testIndividualSecondDerivatives(joints);
    }
    return optimizer.optimize(joints, objective, numSteps);
}

}  // namespace lenny::kineloco