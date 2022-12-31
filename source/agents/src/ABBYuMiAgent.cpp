#include <lenny/agents/ABBYuMiAgent.h>

namespace lenny::agents {

ABBYuMiAgent::Gripper::Gripper(const ABBYuMiRobot& robot, const SIDE side)
    : rapt::Gripper(
          robot, (side == RIGHT) ? "yumi_link_7_r" : "yumi_link_7_l",
          (side == RIGHT)
              ? tools::Transformation(Eigen::QuaternionD(0.873745, 0.371052, -0.067998, 0.307029), 0.9 * Eigen::Vector3d(0.085689, -0.080791, -0.092899))
              : tools::Transformation(Eigen::QuaternionD(0.873647, -0.371479, 0.069156, 0.306531), 0.9 * Eigen::Vector3d(0.085688, -0.080791, 0.092900)),
          (side == RIGHT) ? "Right Gripper" : "Left Gripper"),
      side(side) {
    //Initialize visuals
    visuals.emplace_back(ABBYuMiRobot::folderPath + "/gripper/Base.obj");
    visuals.emplace_back(ABBYuMiRobot::folderPath + "/gripper/Finger1.obj");
    visuals.emplace_back(ABBYuMiRobot::folderPath + "/gripper/Finger2.obj");

    const tools::Transformation localTrafo =
        (side == RIGHT)
            ? tools::Transformation(Eigen::QuaternionD(0.267567, -0.162689, 0.882496, -0.350915), 0.3 * Eigen::Vector3d(0.00888502, -0.00908031, -0.00943452))
            : tools::Transformation(Eigen::QuaternionD(-0.161838, 0.267871, -0.350816, 0.8826), 0.3 * Eigen::Vector3d(0.010336, -0.0088075, 0.0105368));
    for (robot::Visual& visual : visuals) {
        if (robot.f_loadModel)
            robot.f_loadModel(visual.model, visual.filePath);
        visual.localTrafo = localTrafo;
    }
}

void ABBYuMiAgent::Gripper::drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const {
    static const double gripperLength = 0.025;

    const tools::Transformation basePose = globalLinkPose * visuals.at(BASE).localTrafo;
    visuals.at(BASE).model->draw(basePose.position, basePose.orientation, visuals.at(BASE).scale, std::nullopt, alpha);

    const tools::Transformation finger1Pose =
        globalLinkPose * visuals.at(FINGER1).localTrafo *
        tools::Transformation(Eigen::QuaternionD::Identity(), Eigen::Vector3d((0.5 - getFingerPosition()) * gripperLength, 0.0, 0.0));
    visuals.at(FINGER1).model->draw(finger1Pose.position, finger1Pose.orientation, visuals.at(FINGER1).scale, std::nullopt, alpha);

    const tools::Transformation finger2Pose =
        globalLinkPose * visuals.at(FINGER2).localTrafo *
        tools::Transformation(Eigen::QuaternionD::Identity(), Eigen::Vector3d(-1.0 * (0.5 - getFingerPosition()) * gripperLength, 0.0, 0.0));
    visuals.at(FINGER2).model->draw(finger2Pose.position, finger2Pose.orientation, visuals.at(FINGER2).scale, std::nullopt, alpha);

    rapt::Gripper::drawScene(globalLinkPose, alpha);
}

inline Eigen::VectorXb createYuMiDofMask() {
    Eigen::VectorXb dofMask = Eigen::VectorXb::Ones(20);
    dofMask.segment(0, 6).setZero();
    return dofMask;
}

ABBYuMiAgent::ABBYuMiAgent(const std::string& name, const ABBYuMiRobot& robot, const Eigen::VectorXd& initialRobotState)
    : rapt::Agent(name, robot, initialRobotState, createYuMiDofMask()) {
    //Check robot
    if (robot.getStateSize() != 20 || robot.name != "yumi")
        LENNY_LOG_ERROR("Are we sure this is the correct robot!?!")

    //Setup grippers
    grippers.emplace_back(std::make_unique<Gripper>(robot, Gripper::RIGHT));
    grippers.emplace_back(std::make_unique<Gripper>(robot, Gripper::LEFT));

    //Setup collision primitives
    loadCollisionPrimitivesFromFile(std::string(ABBYuMiRobot::folderPath + "/collision_primitives.json").c_str());

    //Setup self-collision link map
    loadSelfCollisionLinkMapFromFile(std::string(ABBYuMiRobot::folderPath + "/self_collision_link_map.json").c_str());
}

ABBYuMiAgent::ABBYuMiAgent(const std::string& name, const ABBYuMiRobot& robot)
    : ABBYuMiAgent(name, robot, robot.loadStateFromFile(std::string(ABBYuMiRobot::folderPath + "/default_state.json").c_str()).value()) {}

}  // namespace lenny::agents