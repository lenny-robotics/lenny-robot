#pragma once

#include <lenny/kineloco/TrajectoryPlanner.h>
#include <lenny/kineloco/TrajectoryTracker.h>
#include <lenny/tools/Typedefs.h>

namespace lenny::kineloco {

class LocomotionController {
public:
    //--- Typedefs
    LENNY_GENERAGE_TYPEDEFS(LocomotionController)

    //--- Constructor
    LocomotionController(const robot::Robot& robot, const Eigen::VectorXd& initialRobotState, const std::vector<LimbInfo>& limbInfos);
    ~LocomotionController() = default;

    //--- Solver
    void computeRobotStateForTime(Eigen::VectorXd& robotState, const TrajectoryPlanner::BaseTrajectory& baseTrajectory, const double& time,
                                  const bool& isInStand);

    //--- Drawing & Gui
    void drawScene() const;
    void drawGui();

    //--- Setter
    void setStrideDuration(const double& strideDuration);
    void setSwingFootHeight(const double& swingFootHeight);

public:
    bool automaticallyComputeStrideDuration = true;
    bool showTrajectories = true;

private:
    TrajectoryPlanner planner;
    TrajectoryTracker tracker;
};

}  // namespace lenny::kineloco