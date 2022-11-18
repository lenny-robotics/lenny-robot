#pragma once

#include <lenny/kineloco/LocomotionController.h>
#include <lenny/rapt/Agent.h>

namespace lenny::agents {

class BDSpotBaseRobot : public robot::Robot {
public:
    BDSpotBaseRobot(const tools::Model::F_loadModel& f_loadModel = nullptr) : robot::Robot(folderPath + "/robot.urdf", f_loadModel) {}
    inline static const std::string folderPath = LENNY_ROBOT_FOLDER "/data/bd_spot/base";
};

//-----------------------------------------------------------------------------------------------------------------------------------------

/**
 * COMMENT: We optimize for the agent robot with floating base, and generate the leg motions of the spot base robot kinematically
 */
class BDSpotAgent : public rapt::Agent {
public:
    BDSpotAgent(const std::string& name, const robot::Robot& agentRobot, const BDSpotBaseRobot& spotRobot, const Eigen::VectorXd& initialAgentRobotState,
                const bool& isInStand);
    virtual ~BDSpotAgent() = default;

    void drawScene(const MotionTrajectory& trajectory, const double& currentTime, const bool& isRecedingHorizon) const override;
    void drawScene(const Eigen::VectorXd& agentState) const override;

    virtual Eigen::Vector6d getSpotBasePoseFromAgentState(const Eigen::VectorXd& agentState) const = 0;

protected:
    void drawAdditionalGuiContent() override;

protected:
    const BDSpotBaseRobot& spotRobot;
    const Eigen::VectorXd initialSpotState;

    mutable kineloco::LocomotionController locomotionController;
    mutable Eigen::VectorXd currentSpotState;
    mutable double locomotionTime = 0.0;

    const bool isInStand;
};

//-----------------------------------------------------------------------------------------------------------------------------------------

class BDSpotFloatingRobot : public robot::Robot {
public:
    BDSpotFloatingRobot(const tools::Model::F_loadModel& f_loadModel = nullptr, const bool& isInStand = false)
        : robot::Robot(BDSpotBaseRobot::folderPath + "/floating_base.urdf", f_loadModel), isInStand(isInStand) {
        if (isInStand)
            base->loadLimitsFromFile(std::string(BDSpotBaseRobot::folderPath + "/base_limits_stand.json").c_str());
        else
            base->loadLimitsFromFile(std::string(BDSpotBaseRobot::folderPath + "/base_limits_walk.json").c_str());
    }
    const bool isInStand;
};

//-----------------------------------------------------------------------------------------------------------------------------------------

class BDSpotBaseAgent : public BDSpotAgent {
public:
    BDSpotBaseAgent(const std::string& name, const BDSpotFloatingRobot& floatingBaseRobot, const BDSpotBaseRobot& spotBaseRobot,
                    const Eigen::VectorXd& initialFloatingRobotState);
    BDSpotBaseAgent(const std::string& name, const BDSpotFloatingRobot& floatingBaseRobot, const BDSpotBaseRobot& spotBaseRobot);

    Eigen::Vector6d getSpotBasePoseFromAgentState(const Eigen::VectorXd& agentState) const override;
};

}  // namespace lenny::agents