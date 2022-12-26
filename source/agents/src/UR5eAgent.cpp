#include <lenny/agents/Robotiq2F85Gripper.h>
#include <lenny/agents/UR5eAgent.h>

namespace lenny::agents {

inline Eigen::VectorXb createU5eDofMask() {
    Eigen::VectorXb dofMask = Eigen::VectorXb::Ones(12);
    dofMask.segment(0, 6).setZero();
    return dofMask;
}

UR5eAgent::UR5eAgent(const std::string& name, const UR5eRobot& robot, const Eigen::VectorXd& initialRobotState)
    : rapt::Agent(name, robot, initialRobotState, createU5eDofMask()) {
    //Check robot
    if (robot.getStateSize() != 12 || robot.name != "ur5e")
        LENNY_LOG_ERROR("Are we sure this is the correct robot!?!")

    //Add gripper
    grippers.emplace_back(std::make_unique<Robotiq2F85Gripper>(
        robot, "wrist_3_link", tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.250), tools::utils::rotX(PI / 2.0) * tools::utils::rotY(-PI / 2.0)),
        tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.094), tools::utils::rotZ(PI / 2.0) * tools::utils::rotX(-PI / 2.0))));

    //Setup collision primitives
    loadCollisionPrimitivesFromFile(std::string(UR5eRobot::folderPath + "/collision_primitives.json").c_str());

    //Setup self-collision link map
    loadSelfCollisionLinkMapFromFile(std::string(UR5eRobot::folderPath + "/self_collision_link_map.json").c_str());
}

UR5eAgent::UR5eAgent(const std::string& name, const UR5eRobot& robot)
    : UR5eAgent(name, robot, robot.loadStateFromFile(std::string(UR5eRobot::folderPath + "/default_state.json").c_str()).value()) {}

}  // namespace lenny::agents