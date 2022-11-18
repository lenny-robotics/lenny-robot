#pragma once

#include <lenny/kineloco/Limb.h>
#include <lenny/robot/Robot.h>

namespace lenny::kineloco {

class TrajectoryPlanner {
public:
    //--- Constructor
    TrajectoryPlanner(const robot::Robot& robot, const Eigen::VectorXd& initialRobotState, const std::vector<LimbInfo>& limbInfos);
    ~TrajectoryPlanner() = default;

    //--- Set
    typedef std::vector<std::pair<Eigen::Vector6d, double>> BaseTrajectory;  //[state, time]
    void setBaseTrajectory(const BaseTrajectory& basetrajectory);
    void updateStrideDuration();
    void setupPeriodicGait();
    void generateLimbTrajectories(const bool& isInStand);

    //--- Get
    tools::Transformation getTargetBaseTransformationAtTime(const double& time) const;

    //--- Drawing & Gui
    void drawScene() const;
    void drawGui();

private:
    //--- Helpers
    double getPlanningHorizon() const;
    std::pair<Eigen::Vector3d, double> computeBodyFrameCoordinatesFromBaseFrame(const tools::Transformation& basePose,
                                                                                const double& time) const;  //[position, headingAngle]
    void addSwingPhaseForLimb(Limb& limb, const std::pair<double, double>& swingPhase);
    double getStridePhaseForTime(const double& time) const;

public:
    const robot::Robot& robot;
    const Eigen::VectorXd initialRobotState;

    std::vector<Limb> limbs;

    double strideDuration = 0.5;
    double swingFootHeight = 0.15;
    double ffStancePhaseForDefaultStepLength = 0.5;

private:
    double timeForLastCSUpdate = 0.0;
    std::vector<std::pair<double, double>> strideUpdates;

    tools::Trajectory<Eigen::Vector6d> baseTrajectory;
    tools::Trajectory1d bodyForwardLeanOffset, bodySidewaysLeanOffset, bodyYawOffset, swingFootHeightTrajectory;
};

}  // namespace lenny::kineloco