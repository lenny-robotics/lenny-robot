#pragma once

#include <lenny/kineloco/TrackerAgenda.h>
#include <lenny/optimization/NewtonOptimizer.h>
#include <lenny/optimization/TotalObjective.h>

namespace lenny::kineloco {

class TrajectoryTracker {
public:
    TrajectoryTracker(const robot::Robot& robot, const Eigen::VectorXd& initialRobotState);
    ~TrajectoryTracker() = default;

    bool solve(const uint& numSteps);

public:
    TrackerAgenda agenda;
    optimization::TotalObjective objective;
    optimization::NewtonOptimizer optimizer;
    Eigen::VectorXd joints;  //This is what we optimize
    bool checkDerivatives = false;
};

}  // namespace lenny::kineloco