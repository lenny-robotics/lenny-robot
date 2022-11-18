#include <lenny/agents/KinovaGen3Agent.h>
#include <lenny/agents/SakeEZGripper.h>

namespace lenny::agents {

inline Eigen::VectorXb createKinovaGen3DofMask() {
    Eigen::VectorXb dofMask = Eigen::VectorXb::Ones(13);
    dofMask.segment(0, 6).setZero();
    return dofMask;
}

KinovaGen3Agent::KinovaGen3Agent(const std::string& name, const KinovaGen3Robot& robot, const Eigen::VectorXd& initialRobotState)
    : rapt::Agent(name, robot, initialRobotState, createKinovaGen3DofMask()) {
    //Check robot
    if (robot.getStateSize() != 13 || robot.name != "kinovagen3")
        LENNY_LOG_ERROR("Are we sure this is the correct robot!?!")

    //Setup gripper
    grippers.emplace_back(std::make_unique<SakeEZGripper>(
        robot, "bracelet_link", tools::Transformation(tools::utils::rotX(PI) * tools::utils::rotY(PI / 2.0), Eigen::Vector3d(0.0, 0.24, 0.0))));

    //Setup collision primitives
    loadCollisionPrimitivesFromFile(std::string(KinovaGen3Robot::folderPath + "/collision_primitives.json").c_str());
}

KinovaGen3Agent::KinovaGen3Agent(const std::string& name, const KinovaGen3Robot& robot)
    : KinovaGen3Agent(name, robot, robot.loadStateFromFile(std::string(KinovaGen3Robot::folderPath + "/default_state.json").c_str()).value()) {}

}  // namespace lenny::agents