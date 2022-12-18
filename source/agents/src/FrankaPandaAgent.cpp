#include <lenny/agents/FrankaPandaAgent.h>

namespace lenny::agents {

FrankaPandaAgent::Gripper::Gripper(const robot::Robot& robot)
    : rapt::Gripper(robot, "panda_link7", tools::Transformation(Eigen::Vector3d(-0.004, -0.204, 0.013), Eigen::QuaternionD::Identity()), "Panda Hand") {
    //Initialize visuals
    visuals.emplace_back(FrankaPandaRobot::folderPath + "/gripper/hand.obj");
    visuals.emplace_back(FrankaPandaRobot::folderPath + "/gripper/finger.obj");
    visuals.emplace_back(FrankaPandaRobot::folderPath + "/gripper/finger.obj");

    visuals.at(HAND).localTrafo = tools::Transformation(Eigen::Vector3d(0.0, -0.106, 0.0), tools::utils::rotY(-3.0 * PI / 4.0) * tools::utils::rotZ(PI));
    visuals.at(FINGER1).localTrafo = tools::Transformation(Eigen::Vector3d(0.0, -0.160, 0.0), tools::utils::rotY(-3.0 * PI / 4.0) * tools::utils::rotZ(PI));
    visuals.at(FINGER2).localTrafo = tools::Transformation(Eigen::Vector3d(0.0, -0.160, 0.0), tools::utils::rotY(PI) * tools::utils::rotY(-3.0 * PI / 4.0) * tools::utils::rotZ(PI));

    if (robot.f_loadModel)
        for (robot::Visual& visual : visuals)
            robot.f_loadModel(visual.model, visual.filePath);
}

void FrankaPandaAgent::Gripper::drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const {
    static const Eigen::Vector3d scale = Eigen::Vector3d::Ones();
    const tools::Transformation localFingerTrafo(Eigen::QuaternionD::Identity(), -fingerPercentage * 0.05 * Eigen::Vector3d::UnitZ());

    const tools::Transformation handPose = globalLinkPose * visuals.at(HAND).localTrafo;
    visuals.at(HAND).model->draw(handPose.position, handPose.orientation, scale, std::nullopt, alpha);

    const tools::Transformation finger1Pose = globalLinkPose * visuals.at(FINGER1).localTrafo * localFingerTrafo;
    visuals.at(FINGER1).model->draw(finger1Pose.position, finger1Pose.orientation, scale, std::nullopt, alpha);

    const tools::Transformation finger2Pose = globalLinkPose * visuals.at(FINGER2).localTrafo * localFingerTrafo;
    visuals.at(FINGER2).model->draw(finger2Pose.position, finger2Pose.orientation, scale, std::nullopt, alpha);

    rapt::Gripper::drawScene(globalLinkPose, alpha);
}

inline Eigen::VectorXb createPandaDofMask() {
    Eigen::VectorXb dofMask = Eigen::VectorXb::Ones(13);
    dofMask.segment(0, 6).setZero();
    return dofMask;
}

FrankaPandaAgent::FrankaPandaAgent(const std::string& name, const FrankaPandaRobot& robot, const Eigen::VectorXd& initialRobotState)
    : rapt::Agent(name, robot, initialRobotState, createPandaDofMask()) {
    //Check robot
    if (robot.getStateSize() != 13 || robot.name != "panda")
        LENNY_LOG_ERROR("Are we sure this is the correct robot!?!")

    //Setup grippers
    grippers.emplace_back(std::make_unique<Gripper>(robot));

    //Setup collision primitives
    loadCollisionPrimitivesFromFile(std::string(FrankaPandaRobot::folderPath + "/collision_primitives.json").c_str());

    //Setup self-collision link map
    loadSelfCollisionLinkMapFromFile(std::string(FrankaPandaRobot::folderPath + "/self_collision_link_map.json").c_str());
}

FrankaPandaAgent::FrankaPandaAgent(const std::string& name, const FrankaPandaRobot& robot)
    : FrankaPandaAgent(name, robot, robot.loadStateFromFile(std::string(FrankaPandaRobot::folderPath + "/default_state.json").c_str()).value()) {}

}  // namespace lenny::agents