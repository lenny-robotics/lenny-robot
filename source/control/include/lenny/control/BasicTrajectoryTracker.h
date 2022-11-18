#pragma once

#include <lenny/control/RobotControlInterface.h>
#include <lenny/tools/Trajectory.h>

namespace lenny::control {

class BasicTrajectoryTracker {
public:
    BasicTrajectoryTracker(RobotControlInterface& rci) : rci(rci) {}
    ~BasicTrajectoryTracker() = default;

    void executeTrajectory(const tools::TrajectoryXd& trajectory, const double& totalTrajectoryTime);
    void stopTrajectoryExecution();
    void gotoStateInTime(const Eigen::VectorXd& state, const double& time);
    bool isTrackingRunning() const;

    void drawGui();

private:
    void applyTrajectoryExecution(const tools::TrajectoryXd& trajectory);

public:
    double targetFramerate = 100.0;
    bool printDebugInfo = true;

private:
    //Control interface
    RobotControlInterface& rci;

    //Trajectory (for gui)
    double totalTrajectoryTime = -1.0;
    double currentTime = -1.0;

    //Thread
    bool trajectoryIsBeingExecuted = false;
    double currentFramerate = targetFramerate;
};
}  // namespace lenny::control