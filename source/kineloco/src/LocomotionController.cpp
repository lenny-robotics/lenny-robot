#include <lenny/kineloco/LocomotionController.h>
#include <lenny/tools/Gui.h>

namespace lenny::kineloco {

LocomotionController::LocomotionController(const robot::Robot& robot, const Eigen::VectorXd& initialRobotState, const std::vector<LimbInfo>& limbInfos)
    : planner(robot, initialRobotState, limbInfos), tracker(robot, initialRobotState) {
    //Setup limb targets
    for (const LimbInfo& limbInfo : limbInfos) {
        const Eigen::Vector3d globalPosition = robot.computeGlobalPoint(initialRobotState, limbInfo.localPosition, limbInfo.linkName);
        tracker.agenda.limbTargets.emplace_back(TrackerAgenda::LimbTarget{limbInfo.linkName, limbInfo.localPosition, globalPosition});
    }

    //Setup optimizer
    tracker.optimizer.printInfos = false;
}

void LocomotionController::computeRobotStateForTime(Eigen::VectorXd& robotState, const TrajectoryPlanner::BaseTrajectory& baseTrajectory, const double& time,
                                                    const bool& isInStand) {
    //Update planner
    planner.setBaseTrajectory(baseTrajectory);
    if (automaticallyComputeStrideDuration)
        planner.updateStrideDuration();
    planner.setupPeriodicGait();
    planner.generateLimbTrajectories(isInStand);

    //Update tracker
    tracker.agenda.setBasePose(planner.getTargetBaseTransformationAtTime(time));
    for (int i = 0; i < planner.limbs.size(); i++)
        tracker.agenda.limbTargets.at(i).globalCoordinates = planner.limbs.at(i).trajectory.getLinearInterpolation(time);

    //Solve
    tracker.solve(100);

    //Get state
    robotState = tracker.agenda.getRobotState(tracker.joints);
}

void LocomotionController::drawScene() const {
    if (showTrajectories)
        planner.drawScene();
}

void LocomotionController::drawGui() {
    using tools::Gui;

    if (Gui::I->TreeNode("Locomotion Controller")) {
        Gui::I->Checkbox("Show Trajectories", showTrajectories);
        Gui::I->Checkbox("Autocompute Stride Duration", automaticallyComputeStrideDuration);
        if (!automaticallyComputeStrideDuration)
            Gui::I->Slider("Stride Duration", planner.strideDuration, 0.0, 1.0);
        planner.drawGui();
        
        Gui::I->TreePop();
    }
}

void LocomotionController::setStrideDuration(const double& strideDuration) {
    automaticallyComputeStrideDuration = false;
    planner.strideDuration = strideDuration;
}

void LocomotionController::setSwingFootHeight(const double& swingFootHeight) {
    planner.swingFootHeight = swingFootHeight;
}

}  // namespace lenny::kineloco