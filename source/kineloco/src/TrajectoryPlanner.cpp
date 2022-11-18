#include <lenny/kineloco/TrajectoryPlanner.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Renderer.h>

namespace lenny::kineloco {

TrajectoryPlanner::TrajectoryPlanner(const robot::Robot& robot, const Eigen::VectorXd& initialRobotState, const std::vector<LimbInfo>& limbInfos)
    : robot(robot), initialRobotState(initialRobotState) {
    //Setup limbs
    limbs.clear();
    const tools::Transformation basePose = robot.computeGlobalPose(initialRobotState, tools::Transformation(), robot.base->linkName);
    for (const LimbInfo& limbInfo : limbInfos) {
        const Eigen::Vector3d globalLinkPosition = robot.computeGlobalPoint(initialRobotState, limbInfo.localPosition, limbInfo.linkName);
        limbs.emplace_back(limbInfo, basePose.getLocalCoordinatesForPoint(globalLinkPosition));
    }

    //Setup trajectories
    swingFootHeightTrajectory.addEntry(0.0, 0.0);
    swingFootHeightTrajectory.addEntry(0.5, 1.0);
    swingFootHeightTrajectory.addEntry(1.0, 0.0);

    bodyForwardLeanOffset.clear();
    bodyForwardLeanOffset.addEntry(0.0, 0.0);
    bodyForwardLeanOffset.addEntry(1.0, 0.0);

    bodySidewaysLeanOffset.clear();
    bodySidewaysLeanOffset.addEntry(0.0, 0.0);
    bodySidewaysLeanOffset.addEntry(1.0, 0.0);

    bodyYawOffset.clear();
    bodyYawOffset.addEntry(0.0, 0.0);
    bodyYawOffset.addEntry(0.25, -0.05);
    bodyYawOffset.addEntry(0.5, 0.0);
    bodyYawOffset.addEntry(0.75, 0.05);
    bodyYawOffset.addEntry(1.0, 0.0);
}

void TrajectoryPlanner::setBaseTrajectory(const std::vector<std::pair<Eigen::Vector6d, double>>& trajectory) {
    auto convertAngleIntoPIRange = [](double& angle) -> void {
        if (angle > PI)
            angle -= 2.0 * PI;
        else if (angle < -PI)
            angle += 2.0 * PI;
    };

    baseTrajectory.clear();
    Eigen::Vector6d basePose_old = trajectory.front().first;
    for (auto [basePose_new, time] : trajectory) {
        //Handle PI range problem for orientation
        for (int j = 3; j < 6; j++) {
            double angle1 = basePose_new[j];
            double angle0 = basePose_old[j];
            convertAngleIntoPIRange(angle1);
            convertAngleIntoPIRange(angle0);
            double diff = angle1 - angle0;
            convertAngleIntoPIRange(diff);
            basePose_new[j] = basePose_old[j] + diff;
        }

        //Add to trajectory
        baseTrajectory.addEntry(time, basePose_new);
        basePose_old = basePose_new;
    }
}

void TrajectoryPlanner::updateStrideDuration() {
    const auto& trajEntries = baseTrajectory.getEntries();
    if (trajEntries.size() > 0) {
        const Eigen::Vector6d firstPose = trajEntries.front().first;
        const Eigen::Vector6d lastPose = trajEntries.back().first;
        const double distance = (firstPose.segment(0, 3) - lastPose.segment(0, 3)).norm() + 2.0 * fabs(firstPose[4] - lastPose[4]);
        const int numSteps = (int)std::ceil(distance / 0.5);
        strideDuration = std::min(0.5, trajEntries.back().second / (double)numSteps);
    }
}

void TrajectoryPlanner::setupPeriodicGait() {
    //Reset
    timeForLastCSUpdate = 0.0;
    strideUpdates.clear();
    for (Limb& limb : limbs)
        limb.reset();

    //Add periodic gaits, starting from simulation time 0.0
    while (timeForLastCSUpdate < getPlanningHorizon()) {
        for (Limb& limb : limbs)
            addSwingPhaseForLimb(limb, {timeForLastCSUpdate + limb.timeInterval.first * strideDuration,
                                        timeForLastCSUpdate + limb.timeInterval.second * strideDuration});
        strideUpdates.emplace_back(std::make_pair(timeForLastCSUpdate, timeForLastCSUpdate + strideDuration));
        timeForLastCSUpdate += strideDuration;
    }
}

void TrajectoryPlanner::generateLimbTrajectories(const bool& isInStand) {
    const double planningHorizon = getPlanningHorizon();
    const double dt = strideDuration / 50.0;  //25.0;
    Eigen::VectorXd robotStartState = initialRobotState;
    if (baseTrajectory.getEntries().size() > 0)
        robotStartState.segment(0, 6) = baseTrajectory.getEntries().front().first;
    for (Limb& limb : limbs) {
        double t = 0.0;  //Start at zero time
        limb.trajectory.clear();
        const Eigen::Vector3d worldPosition = robot.computeGlobalPoint(robotStartState, limb.localPosition, limb.linkName);
        limb.trajectory.addEntry(t, worldPosition);

        if (isInStand) {
            limb.trajectory.addEntry(planningHorizon, worldPosition);
            continue;
        }

        while (t < planningHorizon) {
            t += dt;
            const ContactPhase cp = limb.getContactPhaseForTime(t);

            if (!cp.isInSwing()) {
                //Keep foot on the ground
                const Eigen::Vector3d globalLimbPosition = limb.trajectory.getEntries().back().first;
                limb.trajectory.addEntry(t, globalLimbPosition);
            } else {
                //At the end of every swing phase, we want the foot to be planted such that in the middle of the stance phase,
                //it is in its default/zeroStepLength configuration (e.g. right under the hip/shoulder)

                //First determine the moment in time that corresponds to the middle of the following stance phase
                const ContactPhase cp_stance = limb.getContactPhaseForTime(t + cp.getTimeLeft() + 0.01);
                if (cp_stance.isInSwing())
                    LENNY_LOG_ERROR("After a swing phase, a stance phase should follow, but this is not the case")

                const double tEndOfSwing = t + cp.getTimeLeft();
                const double tMidStance = tEndOfSwing + cp_stance.getDuration() * ffStancePhaseForDefaultStepLength;

                //Now compute the location of the BODY frame at that moment in time
                const auto [bFramePos, bFrameHeadingAngle] =
                    computeBodyFrameCoordinatesFromBaseFrame(getTargetBaseTransformationAtTime(tMidStance), tMidStance);

                Eigen::Vector3d aAxis, bAxis, cAxis;
                robot.base->getRotationAxes(aAxis, bAxis, cAxis);
                const Eigen::Vector3d finalLimbPosition =
                    bFramePos + tools::utils::getRotationQuaternion(bFrameHeadingAngle, cAxis) * limb.defaultOffset;

                //Plan motion for the entire swing phase
                while (t < tEndOfSwing && t < planningHorizon) {
                    const ContactPhase cp_swing = limb.getContactPhaseForTime(t);
                    const Eigen::Vector3d oldLimbPosition = limb.trajectory.getEntries().back().first;

                    //We have the remainder of the swing phase to go from the old step position to the final stepping location.
                    //Based on this we know how much we should be travelling over a time window dt...
                    double dTimeStep = dt / cp_swing.getTimeLeft();
                    if (dTimeStep > 1.0)
                        dTimeStep = 1.0;

                    //This is now in the body coordinate frame
                    const Eigen::Vector3d deltaStep = dTimeStep * (finalLimbPosition - oldLimbPosition);

                    Eigen::Vector3d currentLimbPosition = oldLimbPosition + deltaStep;
                    currentLimbPosition.y() = swingFootHeightTrajectory.getSplineInterpolation(cp_swing.getPercentageOfTimeEllapsed()) * swingFootHeight;
                    limb.trajectory.addEntry(t, currentLimbPosition);

                    if (t + dt < tEndOfSwing)
                        t += dt;
                    else
                        break;
                }
            }
        }
    }
}

tools::Transformation TrajectoryPlanner::getTargetBaseTransformationAtTime(const double& time) const {
    return robot.base->getTransformationFromState(baseTrajectory.getLinearInterpolation(time));
}

void TrajectoryPlanner::drawScene() const {
    //Base trajectory
    const auto& baseTrajectryEntries = baseTrajectory.getEntries();
    for (const auto& [pose, time] : baseTrajectryEntries) {
        const tools::Transformation trafo = robot.base->getTransformationFromState(pose);
        tools::Renderer::I->drawCoordinateSystem(trafo.position, trafo.orientation, 0.1, 0.01);
    }

    //Limb trajectories
    for (const auto& limb : limbs) {
        const auto& limbTrajectoryEntries = limb.trajectory.getEntries();
        for (const auto& [position, time] : limbTrajectoryEntries)
            tools::Renderer::I->drawSphere(position, 0.01, Eigen::Vector4d(1.0, 1.0, 0.0, 1.0));
    }
}

void TrajectoryPlanner::drawGui() {
    using tools::Gui;
    Gui::I->Text("Planning Horizon: %lf", getPlanningHorizon());
    Gui::I->Slider("Swing Foot Height", swingFootHeight, 0.0, 1.0);
    Gui::I->Slider("FF Stance Phase", ffStancePhaseForDefaultStepLength, 0.0, 1.0);

    for (Limb& limb : limbs)
        limb.drawGui();
}

double TrajectoryPlanner::getPlanningHorizon() const {
    if (baseTrajectory.getEntries().size() > 0)
        return baseTrajectory.getEntries().back().second;
    return 0.0;
}

//[position, headingAngle]
std::pair<Eigen::Vector3d, double> TrajectoryPlanner::computeBodyFrameCoordinatesFromBaseFrame(const tools::Transformation& baseFrame,
                                                                                               const double& time) const {
    const double stridePhase = getStridePhaseForTime(time);
    const Eigen::Vector6d basePose = robot.base->getStateFromTransformation(baseFrame);
    const double bFrameHeadingAngle = basePose[5] - bodyYawOffset.getSplineInterpolation(stridePhase);
    Eigen::Vector3d aAxis, bAxis, cAxis;
    robot.base->getRotationAxes(aAxis, bAxis, cAxis);
    const Eigen::QuaternionD bFrameHeading = tools::utils::getRotationQuaternion(bFrameHeadingAngle, cAxis);
    const Eigen::Vector3d bFramePos =
        basePose.segment(0, 3) - (bFrameHeading * aAxis * bodyForwardLeanOffset.getSplineInterpolation(stridePhase) +
                                  bFrameHeading * bAxis * bodySidewaysLeanOffset.getSplineInterpolation(stridePhase));
    return {bFramePos, bFrameHeadingAngle};
}

void TrajectoryPlanner::addSwingPhaseForLimb(Limb& limb, const std::pair<double, double>& swingPhase) {
    if (swingPhase.first >= swingPhase.second)
        LENNY_LOG_ERROR("Swing phase must end after it begins")

    if (limb.swingPhases.size() > 0) {
        //If the new swing phase starts right when the previous one ends, merge them
        if (fabs(limb.swingPhases.back().second - swingPhase.first) < 1e-5) {
            limb.swingPhases.back().second = swingPhase.second;
            return;
        }

        //Make sure that this new swing phase starts after the previous one ended
        if (limb.swingPhases.back().second > swingPhase.first) {
            LENNY_LOG_WARNING("New swing phase starts after the previous one ended (%lf VS %lf)... Ignoring new swing phase!", limb.swingPhases.back().second,
                              swingPhase.first)
            return;
        }
    }

    limb.swingPhases.emplace_back(swingPhase);
}

double TrajectoryPlanner::getStridePhaseForTime(const double& time) const {
    if (strideUpdates.size() == 0 || time < strideUpdates.front().first)
        return 0.0;
    for (const auto& [start, end] : strideUpdates) {
        if (time >= start && time <= end)
            return (time - start) / end - start;
    }
    return 0.0;
}

}  // namespace lenny::kineloco