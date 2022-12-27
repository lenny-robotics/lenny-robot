#include <lenny/agents/BDSpotBaseAgent.h>

namespace lenny::agents {

BDSpotAgent::BDSpotAgent(const std::string& name, const robot::Robot& agentRobot, const BDSpotBaseRobot& spotRobot,
                         const Eigen::VectorXd& initialAgentRobotState, const bool& isInStand)
    : rapt::Agent(name, agentRobot, initialAgentRobotState, Eigen::VectorXb::Ones(agentRobot.getStateSize())),
      spotRobot(spotRobot),
      initialSpotState(spotRobot.loadStateFromFile(std::string(BDSpotBaseRobot::folderPath + "/default_state.json").c_str()).value()),
      currentSpotState(initialSpotState),
      locomotionController(this->spotRobot, initialSpotState,
                           {kineloco::LimbInfo("front_right_lower_leg", Eigen::Vector3d(0.0, -0.41, 0.0), {0.5, 1.0}),
                            kineloco::LimbInfo("front_left_lower_leg", Eigen::Vector3d(0.0, -0.41, 0.0), {0.0, 0.5}),
                            kineloco::LimbInfo("rear_right_lower_leg", Eigen::Vector3d(0.0, -0.38, 0.0), {0.0, 0.5}),
                            kineloco::LimbInfo("rear_left_lower_leg", Eigen::Vector3d(0.0, -0.38, 0.0), {0.5, 1.0})}),
      isInStand(isInStand) {
    //Check robot
    if (spotRobot.getStateSize() != 18 || spotRobot.name != "spot")
        LENNY_LOG_ERROR("Are we sure this is the correct robot!?!")

    //Draw settings
    locomotionController.showTrajectories = false;
}

void BDSpotAgent::drawScene(const MotionTrajectory& trajectory, const double& currentTime, const bool& isRecedingHorizon) const {
    //Compute spot state
    if (showVisuals) {
        if (isRecedingHorizon) {
            locomotionTime += trajectory.deltaT;
            const double strideDuration = 0.5;
            locomotionController.setStrideDuration(strideDuration);

            const Eigen::Vector6d initialSpotPose = getSpotBasePoseFromRobotState(getInitialRobotState());
            const Eigen::Vector6d initialSpotVelocity = getInitialRobotVelocity().segment(0, 6);

            std::vector<std::pair<Eigen::Vector6d, double>> baseTrajectory;
            baseTrajectory.emplace_back(std::make_pair(initialSpotPose, 0.0));
            baseTrajectory.emplace_back(std::make_pair(initialSpotPose + 2.0 * strideDuration * initialSpotVelocity, 2.0 * strideDuration));

            locomotionController.computeRobotStateForTime(currentSpotState, baseTrajectory, locomotionTime, isInStand);
            currentSpotState.segment(0, 6) = getInitialRobotState().segment(0, 6);

            if (locomotionTime > 2.0 * strideDuration)
                locomotionTime = 0.0;
        } else {
            currentSpotState = initialSpotState;
            std::vector<std::pair<Eigen::Vector6d, double>> baseTrajectory;
            for (int i = -1; i < (int)trajectory.numSteps; i++) {
                const Eigen::VectorXd agentState = getAgentStateForTrajectoryIndex(trajectory, i);
                const double time = trajectory.getTimeForIndex(i);
                baseTrajectory.emplace_back(std::make_pair(getSpotBasePoseFromRobotState(getRobotStateFromAgentState(agentState)), time));
            }
            locomotionController.computeRobotStateForTime(currentSpotState, baseTrajectory, currentTime, isInStand);
        }

        //Draw spot robot
        spotRobot.drawVisuals(currentSpotState, std::nullopt, visualAlpha);
    }

    //Draw locomotion controller info
    locomotionController.drawScene();
    
    //Draw agent robot
    rapt::Agent::drawScene(getAgentStateForTrajectoryTime(trajectory, currentTime));
}

void BDSpotAgent::drawScene(const Eigen::VectorXd& agentState) const {
    Eigen::VectorXd spotState(initialSpotState);
    spotState.segment(0, 6) = getSpotBasePoseFromRobotState(getRobotStateFromAgentState(agentState));
    spotRobot.drawVisuals(spotState, std::nullopt, visualAlpha);
    rapt::Agent::drawScene(agentState);
}

void BDSpotAgent::drawAdditionalGuiContent() {
    locomotionController.drawGui();
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------

inline Eigen::VectorXd generateInitialSpotState(const Eigen::VectorXd& initialFloatingRobotState) {
    Eigen::VectorXd state(initialFloatingRobotState);
    state[1] = 0.49;
    state[3] = 0.0;
    state[4] = 0.0;
    return state;
}

BDSpotBaseAgent::BDSpotBaseAgent(const std::string& name, const BDSpotFloatingRobot& floatingBaseRobot, const BDSpotBaseRobot& spotBaseRobot,
                                 const Eigen::VectorXd& initialFloatingRobotState)
    : BDSpotAgent(name, floatingBaseRobot, spotBaseRobot, generateInitialSpotState(initialFloatingRobotState), floatingBaseRobot.isInStand) {
    //Check robot
    if (robot.getStateSize() != 6 || robot.name != "spot_floating_base")
        LENNY_LOG_ERROR("Are we sure this is the correct robot!?!")

    //Setup collision primitives
    loadCollisionPrimitivesFromFile(std::string(BDSpotBaseRobot::folderPath + "/collision_primitives.json").c_str());

    //Setup self-collision link map
    loadSelfCollisionLinkMapFromFile(std::string(BDSpotBaseRobot::folderPath + "/self_collision_link_map.json").c_str());
}

BDSpotBaseAgent::BDSpotBaseAgent(const std::string& name, const BDSpotFloatingRobot& floatingBaseRobot, const BDSpotBaseRobot& spotBaseRobot)
    : BDSpotBaseAgent(name, floatingBaseRobot, spotBaseRobot, Eigen::VectorXd::Zero(6)) {}

Eigen::Vector6d BDSpotBaseAgent::getSpotBasePoseFromRobotState(const Eigen::VectorXd& robotState) const {
    return robotState;
}

}  // namespace lenny::agents