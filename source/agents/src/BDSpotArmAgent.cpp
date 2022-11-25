#include <lenny/agents/BDSpotArmAgent.h>

namespace lenny::agents {

BDSpotArmAgent::Gripper::Gripper(const BDSpotArmRobot& robot)
    : rapt::Gripper(robot, "link6", tools::Transformation(Eigen::QuaternionD::Identity(), Eigen::Vector3d(0.17, -0.02, 0.0)), "Gripper") {
    //Initialize visuals
    visuals.emplace_back(BDSpotArmRobot::folderPath + "/meshes/Jaw.obj");
    robot::Visual& visual = visuals.back();
    if (robot.f_loadModel)
        robot.f_loadModel(visual.model, visual.filePath);
    visual.localTrafo = tools::Transformation(Eigen::QuaternionD::Identity(), Eigen::Vector3d(-0.119, -0.075, 0.0));
    visual.scale = 0.001 * Eigen::Vector3d::Ones();
    visual.color = Eigen::Vector3d(0.91372, 0.64705, 0.0);
}

void BDSpotArmAgent::Gripper::drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const {
    static const Eigen::Vector3d localRotPos(0.182, 0.090, 0.0);
    static const Eigen::Vector3d localRotDir(0.0, 0.0, 1.0);
    static const double maxAngle = PI / 2.0;

    const robot::Visual& visual = visuals.back();
    const tools::Transformation globalZeroPose = globalLinkPose * visual.localTrafo;
    const Eigen::Vector3d globalRotAxis = globalZeroPose.getGlobalCoordinatesForVector(localRotDir);
    const Eigen::QuaternionD globalRot = tools::utils::getRotationQuaternion(fingerPercentage * maxAngle, globalRotAxis) * globalZeroPose.orientation;
    const Eigen::Vector3d globalPos = globalZeroPose.getGlobalCoordinatesForPoint(localRotPos) - globalRot * localRotPos;
    visual.model->draw(globalPos, globalRot, visual.scale, visual.color, alpha);

    rapt::Gripper::drawScene(globalLinkPose, alpha);
}

inline Eigen::VectorXd generateInitialSpotArmState(const Eigen::VectorXd& initialArmRobotState) {
    Eigen::VectorXd state(initialArmRobotState);
    state[1] = 0.49 + 0.07;
    state[3] = 0.0;
    state[4] = 0.0;
    return state;
}

BDSpotArmAgent::BDSpotArmAgent(const std::string& name, const BDSpotArmRobot& armRobot, const BDSpotBaseRobot& baseRobot,
                               const Eigen::VectorXd& initialArmRobotState)
    : BDSpotAgent(name, armRobot, baseRobot, generateInitialSpotArmState(initialArmRobotState), armRobot.isInStand) {
    //Check robots
    if (armRobot.getStateSize() != 12 || armRobot.name != "spotArm")
        LENNY_LOG_ERROR("Are we sure this is the correct robot!?!")

    //Setup gripper
    grippers.emplace_back(std::make_unique<Gripper>(armRobot));

    //Setup collision primitives
    loadCollisionPrimitivesFromFile(std::string(BDSpotArmRobot::folderPath + "/collision_primitives.json").c_str());

    //Setup self-collision link map
    loadSelfCollisionLinkMapFromFile(std::string(BDSpotArmRobot::folderPath + "/self_collision_link_map.json").c_str());

    //Set local base trafo
    localBaseTrafo.position = Eigen::Vector3d(0.292, 0.070, 0.0);
}

BDSpotArmAgent::BDSpotArmAgent(const std::string& name, const BDSpotArmRobot& armRobot, const BDSpotBaseRobot& baseRobot)
    : BDSpotArmAgent(name, armRobot, baseRobot,
                     generateInitialSpotArmState(armRobot.loadStateFromFile(std::string(BDSpotArmRobot::folderPath + "/default_state.json").c_str()).value())) {
}

Eigen::Vector6d BDSpotArmAgent::getSpotBasePoseFromRobotState(const Eigen::VectorXd& robotState) const {
    static const tools::Transformation localSpotArmTrafo(Eigen::QuaternionD::Identity(), Eigen::Vector3d(0.292, 0.070, 0.0));
    return spotRobot.base->getStateFromTransformation(robot.base->getTransformationFromState(robotState.segment(0, 6)) * localSpotArmTrafo.inverse());
}

}  // namespace lenny::agents