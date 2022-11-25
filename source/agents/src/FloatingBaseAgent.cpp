#include <lenny/agents/FloatingBaseAgent.h>

namespace lenny::agents {

FloatingBaseAgent::FloatingBaseAgent(const std::string& name, const FloatingBaseRobot& robot, const Eigen::VectorXd& initialRobotState)
    : rapt::Agent(name, robot, initialRobotState, Eigen::VectorXb::Ones(6)) {
    //Check robot
    if (robot.getStateSize() != 6 || robot.name != "base")
        LENNY_LOG_ERROR("Are we sure this is the correct robot!?!")

    //Setup collision primitives
    loadCollisionPrimitivesFromFile(std::string(FloatingBaseRobot::folderPath + "/collision_primitives.json").c_str());

    //Setup self-collision link map
    loadSelfCollisionLinkMapFromFile(std::string(FloatingBaseRobot::folderPath + "/self_collision_link_map.json").c_str());
}

FloatingBaseAgent::FloatingBaseAgent(const std::string& name, const FloatingBaseRobot& robot)
    : FloatingBaseAgent(name, robot, robot.loadStateFromFile(std::string(FloatingBaseRobot::folderPath + "/default_state.json").c_str()).value()) {}

}  // namespace lenny::agents