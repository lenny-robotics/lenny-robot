#include <lenny/agents/SakeEZGripper.h>
#include <lenny/tools/Renderer.h>

namespace lenny::agents {

SakeEZGripper::SakeEZGripper(const robot::Robot& robot, const std::string& linkName, const tools::Transformation& localTrafo)
    : rapt::Gripper(robot, linkName, localTrafo, "SakeEZGripper") {
    //Initialize visuals
    visuals.emplace_back(folderPath + "/meshes/SAKE_Palm_Dual_Gen2.obj");
    visuals.emplace_back(folderPath + "/meshes/SAKE_Finger_L1_Gen2.obj");
    visuals.emplace_back(folderPath + "/meshes/SAKE_Finger_L2_Gen2.obj");
    visuals.emplace_back(folderPath + "/meshes/SAKE_Finger_L1_Gen2.obj");
    visuals.emplace_back(folderPath + "/meshes/SAKE_Finger_L2_Gen2.obj");

    static const tools::Transformation localTrafo_gripper(tools::utils::rotY(PI), Eigen::Vector3d(0.0, 0.075, 0.0));
    static const tools::Transformation localTrafo_palm(tools::utils::rotZ(PI / 2.0), Eigen::Vector3d::Zero());
    static const tools::Transformation localTrafo_l1_left(tools::utils::rotX(PI / 2.0), Eigen::Vector3d(0.030, 0.075, 0.0));
    static const tools::Transformation localTrafo_l1_right(tools::utils::rotX(PI / 2.0) * tools::utils::rotZ(PI), Eigen::Vector3d(-0.030, 0.075, 0.0));
    static const tools::Transformation localTrafo_l2(Eigen::QuaternionD::Identity(), Eigen::Vector3d(0.050, 0.0, 0.0));

    visuals.at(PALM).localTrafo = localTrafo_gripper * localTrafo_palm;
    visuals.at(LINK1_LEFT).localTrafo = localTrafo_gripper * localTrafo_l1_left;
    visuals.at(LINK2_LEFT).localTrafo = localTrafo_l2;
    visuals.at(LINK1_RIGHT).localTrafo = localTrafo_gripper * localTrafo_l1_right;
    visuals.at(LINK2_RIGHT).localTrafo = localTrafo_l2;

    visuals.at(PALM).color = Eigen::Vector3d(0.0, 0.16471, 0.45882);
    visuals.at(LINK1_LEFT).color = Eigen::Vector3d(0.75, 0.75, 0.75);
    visuals.at(LINK2_LEFT).color = Eigen::Vector3d(0.75, 0.75, 0.75);
    visuals.at(LINK1_RIGHT).color = Eigen::Vector3d(0.75, 0.75, 0.75);
    visuals.at(LINK2_RIGHT).color = Eigen::Vector3d(0.75, 0.75, 0.75);

    if (robot.f_loadModel)
        for (robot::Visual& visual : visuals)
            robot.f_loadModel(visual.model, visual.filePath);
}

void SakeEZGripper::drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const {
    static const tools::Transformation localTrafo_plate(Eigen::QuaternionD::Identity(), Eigen::Vector3d(0.0, 0.070, 0.0));
    const Eigen::Vector4d color_plate(0.375, 0.375, 0.375, alpha);
    const tools::Transformation pose_plate = globalLinkPose * localTrafo_plate;
    tools::Renderer::I->drawCylinder(pose_plate.position, pose_plate.orientation, 0.011, 0.043, color_plate);

    const double fingerAngle = (1.0 - fingerPercentage) * (PI / 2.0 + 0.375);
    const tools::Transformation localFingerTrafo(tools::utils::getRotationQuaternion(fingerAngle, Eigen::Vector3d::UnitY()), Eigen::Vector3d::Zero());

    const tools::Transformation pose_palm = globalLinkPose * visuals.at(PALM).localTrafo;
    visuals.at(PALM).model->draw(pose_palm.position, pose_palm.orientation, visuals.at(PALM).scale, visuals.at(PALM).color, alpha);

    const tools::Transformation pose_l1_left = globalLinkPose * visuals.at(LINK1_LEFT).localTrafo * localFingerTrafo;
    visuals.at(LINK1_LEFT).model->draw(pose_l1_left.position, pose_l1_left.orientation, visuals.at(LINK1_LEFT).scale, visuals.at(LINK1_LEFT).color, alpha);

    const tools::Transformation pose_l2_left = pose_l1_left * visuals.at(LINK2_LEFT).localTrafo;
    visuals.at(LINK2_LEFT).model->draw(pose_l2_left.position, pose_l2_left.orientation, visuals.at(LINK2_LEFT).scale, visuals.at(LINK2_LEFT).color, alpha);

    const tools::Transformation pose_l1_right = globalLinkPose * visuals.at(LINK1_RIGHT).localTrafo * localFingerTrafo;
    visuals.at(LINK1_RIGHT).model->draw(pose_l1_right.position, pose_l1_right.orientation, visuals.at(LINK1_RIGHT).scale, visuals.at(LINK1_RIGHT).color, alpha);

    const tools::Transformation pose_l2_right = pose_l1_right * visuals.at(LINK2_RIGHT).localTrafo;
    visuals.at(LINK2_RIGHT).model->draw(pose_l2_right.position, pose_l2_right.orientation, visuals.at(LINK2_RIGHT).scale, visuals.at(LINK2_RIGHT).color, alpha);

    rapt::Gripper::drawScene(globalLinkPose, alpha);
}

}  // namespace lenny::agents