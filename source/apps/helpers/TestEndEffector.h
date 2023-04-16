#pragma once

#include <lenny/gui/Model.h>
#include <lenny/robot/EndEffector.h>

namespace lenny {

class TestEndEffector : public robot::EndEffector {
public:
    TestEndEffector(const std::string& linkName, const tools::Transformation& localGraspTrafo) : robot::EndEffector(linkName, 1, localGraspTrafo) {
        //Add meshes
        visuals.emplace_back(LENNY_ROBOT_APP_FOLDER "/config/onrobot_gripper/base_link.stl", gui::Model::f_loadModel,
                             tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.095), Eigen::QuaternionD(0.0, 0.707105, 0.707105, 0.0)),
                             0.001 * Eigen::Vector3d::Ones(), 0.7 * Eigen::Vector3d::Ones());
        visuals.emplace_back(LENNY_ROBOT_APP_FOLDER "/config/onrobot_gripper/left_finger_link.stl", gui::Model::f_loadModel,
                             tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.095), Eigen::QuaternionD(0.0, 0.707105, 0.707105, 0.0)),
                             0.001 * Eigen::Vector3d::Ones(), 0.2 * Eigen::Vector3d::Ones());
        visuals.emplace_back(LENNY_ROBOT_APP_FOLDER "/config/onrobot_gripper/right_finger_link.stl", gui::Model::f_loadModel,
                             tools::Transformation(Eigen::Vector3d(0.0, 0.0, -0.095), Eigen::QuaternionD(0.0, 0.707105, 0.707105, 0.0)),
                             0.001 * Eigen::Vector3d::Ones(), 0.2 * Eigen::Vector3d::Ones());

        //Setup drawing function
        f_drawVisuals = [](const std::vector<robot::Visual>& visuals, const Eigen::VectorXd& state, const tools::Transformation& globalLinkPose,
                           const std::optional<Eigen::Vector3d>& color, const double& alpha) -> void {
            //Draw base
            visuals.at(0).drawScene(globalLinkPose, color.has_value() ? color : visuals.at(0).color, alpha);

            //Draw left finger
            const double length = 0.022;
            const tools::Transformation localLeftFingerTrafo(Eigen::Vector3d(0.0, -length * (1.0 - state[0]), 0.0), Eigen::QuaternionD::Identity());
            visuals.at(1).drawScene(globalLinkPose * localLeftFingerTrafo, color.has_value() ? color : visuals.at(1).color, alpha);

            //Draw right finger
            const tools::Transformation localRightFingerTrafo(Eigen::Vector3d(0.0, length * (1.0 - state[0]), 0.0), Eigen::QuaternionD::Identity());
            visuals.at(2).drawScene(globalLinkPose * localRightFingerTrafo, color.has_value() ? color : visuals.at(2).color, alpha);
        };
    }
};

}  // namespace lenny