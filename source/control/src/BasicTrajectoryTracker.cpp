#include <lenny/control/BasicTrajectoryTracker.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Logger.h>
#include <lenny/tools/Timer.h>

#include <thread>

namespace lenny::control {

void BasicTrajectoryTracker::executeTrajectory(const tools::TrajectoryXd& trajectory, const double& totalTrajectoryTime) {
    //Check inputs
    if (trajectory.getLinearInterpolation(totalTrajectoryTime).size() != rci.robot.getStateSize() || totalTrajectoryTime < 1e-5)
        LENNY_LOG_ERROR("Invalid inputs")

    //Apply trajectory execution
    if (rci.isConnected()) {
        this->totalTrajectoryTime = totalTrajectoryTime;
        std::thread(&BasicTrajectoryTracker::applyTrajectoryExecution, this, trajectory).detach();
    }
}

void BasicTrajectoryTracker::stopTrajectoryExecution() {
    trajectoryIsBeingExecuted = false;
    rci.stop();
}

void BasicTrajectoryTracker::gotoStateInTime(const Eigen::VectorXd& state, const double& time) {
    //Check connection
    if (!rci.isConnected())
        return;

    //Check inputs
    if (state.size() != rci.robot.getStateSize())
        LENNY_LOG_ERROR("Invalid inputs")

    //Get current values
    Eigen::VectorXd currentPosition, currentVelocity;
    rci.getCurrentValues(currentPosition, currentVelocity);

    //Compute delta
    const Eigen::VectorXd delta = rci.robot.estimateVelocity(state, currentPosition, 1.0);

    //Fill trajectory
    const double dt = 1.0 / 30.0;
    const uint n = uint(time / dt);
    tools::TrajectoryXd trajectory;
    for (uint i = 0; i <= n; i++)
        trajectory.addEntry((double)i * dt, currentPosition + (double)i / (double)n * delta);

    //Execute
    executeTrajectory(trajectory, time);
}

bool BasicTrajectoryTracker::isTrackingRunning() const {
    return trajectoryIsBeingExecuted;
}

void BasicTrajectoryTracker::drawGui() {
    using tools::Gui;
    if (Gui::I->TreeNode(("Trajectory Tracker - " + rci.robot.name).c_str())) {
        if (trajectoryIsBeingExecuted)
            Gui::I->TextColored(Eigen::Vector4d(0.75, 0.0, 0.0, 1.0), "EXECUTION ONGOING");

        Gui::I->Text("Time: %lf / %lf", currentTime, totalTrajectoryTime);
        Gui::I->Text("Framerate: %lf VS %lf", currentFramerate, targetFramerate);
        Gui::I->Input("Target Framerate", targetFramerate);

        Gui::I->TreePop();
    }
}

void BasicTrajectoryTracker::applyTrajectoryExecution(const tools::TrajectoryXd& trajectory) {
    //Initialize
    LENNY_LOG_INFO("Trajectory execution started...")
    trajectoryIsBeingExecuted = true;
    currentTime = 0.0;

    //Get current values ...
    Eigen::VectorXd currentPosition, currentVelocity;
    rci.getCurrentValues(currentPosition, currentVelocity);

    //... and initialize target values with them. Then start the syncing process
    rci.setTargetValues(currentPosition, currentVelocity);
    rci.sendCommands = true;

    //Find final pose to end execution
    const Eigen::VectorXd finalPosition = trajectory.getLinearInterpolation(totalTrajectoryTime + 1.0);

    //Set timers
    tools::Timer trajectoryTimer, framerateTimer;
    trajectoryTimer.restart();
    double averageDeltaT = 1.0 / targetFramerate;

    //Execute trajectory loop
    while (trajectoryIsBeingExecuted && rci.sendCommands && trajectoryTimer.time() < 2.0 * totalTrajectoryTime) {
        //Update time
        framerateTimer.restart();
        currentTime = trajectoryTimer.time();

        //Set target values
        rci.setTargetValues(trajectory.getLinearInterpolation(currentTime), averageDeltaT);

        //Update current values
        rci.getCurrentValues(currentPosition, currentVelocity);

        //Check if final position has been reached
        if (currentTime > totalTrajectoryTime && rci.positionReached(currentPosition, finalPosition))
            break;

        //Handle framerate
        const double waitTime = (1.0 / targetFramerate) - framerateTimer.time();
        if (waitTime > 0.0)
            framerateTimer.sleep(waitTime, true);
        currentFramerate = 1.0 / framerateTimer.time();
        averageDeltaT = 0.5 * (averageDeltaT + framerateTimer.time());
        if (printDebugInfo && (targetFramerate - currentFramerate > 1.0))
            LENNY_LOG_WARNING("Frame rate dropped: %lf VS %lf", currentFramerate, targetFramerate)
    }

    //Wrap up
    rci.stop();
    tools::Timer::sleep(0.1, false);
    trajectoryIsBeingExecuted = false;
    LENNY_LOG_INFO("Trajectory execution finished")
}

}  // namespace lenny::control