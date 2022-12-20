#include <lenny/agents/Robotiq2F85Gripper.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Renderer.h>

namespace lenny::agents {

Robotiq2F85Gripper::Constraint::Constraint(const robot::Robot& robot) : optimization::EqualityConstraint("Constraint"), robot(robot) {
    useTensorForHessian = false;

    infos.at(0) = {
        {"left_outer_knuckle", Eigen::Vector3d(0.035, 0.048, 0.0)}, {"left_inner_finger", Eigen::Vector3d(0.019, -0.006, 0.0)}, 10.0 * Eigen::Vector3d::Ones()};
    infos.at(1) = {{"right_outer_knuckle", Eigen::Vector3d(-0.035, 0.048, 0.0)},
                   {"right_inner_finger", Eigen::Vector3d(-0.019, -0.006, 0.0)},
                   10.0 * Eigen::Vector3d::Ones()};

    infos.at(2) = {{"left_inner_finger", Eigen::Vector3d(-0.009, 0.025, 0.0)},
                   {"robotiq_85_base_link", Eigen::Vector3d(0.043, 0.135, 0.0)},
                   1.0 * Eigen::Vector3d::UnitX()};
    infos.at(3) = {{"right_inner_finger", Eigen::Vector3d(0.009, 0.025, 0.0)},
                   {"robotiq_85_base_link", Eigen::Vector3d(-0.043, 0.135, 0.0)},
                   1.0 * Eigen::Vector3d::UnitX()};

    jointPairs.at(0) = {"finger_joint", "right_outer_knuckle_joint"};
    jointPairs.at(1) = {"left_inner_finger_joint", "right_inner_finger_joint"};
    jointPairs.at(2) = {"left_inner_knuckle_joint", "right_inner_knuckle_joint"};

    softificationWeights.setOnes(getConstraintNumber());
}

void Robotiq2F85Gripper::Constraint::setTargetsFromFromFingerPercentage(const double& fingerPercentage) {
    std::get<1>(infos.at(2)).second.x() = fingerPercentage * gripperLength * 0.5;
    std::get<1>(infos.at(3)).second.x() = fingerPercentage * gripperLength * 0.5 * -1.0;
}

uint Robotiq2F85Gripper::Constraint::getConstraintNumber() const {
    return infos.size() * 3 + jointPairs.size() * 1;
}

void Robotiq2F85Gripper::Constraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& x) const {
    C.resize(getConstraintNumber());
    C.setZero();
    Eigen::VectorXd robotState(6 + x.size());
    robotState << Eigen::Vector6d::Zero(), x;
    int iter = 0;
    for (const auto& [info1, info2, weight] : infos) {
        const Eigen::Vector3d p1 = robot.computeGlobalPoint(robotState, info1.second, info1.first);
        const Eigen::Vector3d p2 = robot.computeGlobalPoint(robotState, info2.second, info2.first);
        C.segment(iter, 3) = weight.cwiseProduct(p1 - p2);
        iter += 3;
    }
    for (const auto& [joint1, joint2] : jointPairs)
        C[iter++] = x[robot.getStateIndex(joint1) - 6] - x[robot.getStateIndex(joint2) - 6];
}

void Robotiq2F85Gripper::Constraint::computeJacobian(Eigen::SparseMatrixD& pCpX, const Eigen::VectorXd& x) const {
    Eigen::VectorXd robotState(6 + x.size());
    robotState << Eigen::Vector6d::Zero(), x;
    Eigen::VectorXb dofMask(6 + x.size());
    dofMask << Eigen::VectorXb::Zero(6), Eigen::VectorXb::Ones(x.size());
    Eigen::MatrixXd jac(getConstraintNumber(), x.size());
    jac.setZero();
    int iter = 0;
    for (const auto& [info1, info2, weight] : infos) {
        Eigen::MatrixXd jac1, jac2;
        robot.computePointJacobian(jac1, robotState, info1.second, info1.first, dofMask);
        robot.computePointJacobian(jac2, robotState, info2.second, info2.first, dofMask);
        for (int i = 0; i < 3; i++)
            jac.block(iter + i, 0, 1, x.size()) = weight[i] * (jac1.block(i, 6, 1, x.size()) - jac2.block(i, 6, 1, x.size()));
        iter += 3;
    }
    for (const auto& [joint1, joint2] : jointPairs) {
        jac.coeffRef(iter, robot.getStateIndex(joint1) - 6) = 1.0;
        jac.coeffRef(iter, robot.getStateIndex(joint2) - 6) = -1.0;
        iter++;
    }
    pCpX = jac.sparseView();
}

void Robotiq2F85Gripper::Constraint::computeTensor(Eigen::TensorD& p2CpX2, const Eigen::VectorXd& x) const {
    p2CpX2.resize(Eigen::Vector3i(getConstraintNumber(), x.size(), x.size()));
    p2CpX2.setZero();
}

void Robotiq2F85Gripper::Constraint::drawScene(const Eigen::VectorXd& robotState) const {
    for (const auto& [info1, info2, weight] : infos) {
        tools::Renderer::I->drawSphere(robot.computeGlobalPoint(robotState, info1.second, info1.first), 0.005, Eigen::Vector4d(1.0, 0.0, 0.0, 1.0));
        tools::Renderer::I->drawSphere(robot.computeGlobalPoint(robotState, info2.second, info2.first), 0.005, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0));
    }
}

void Robotiq2F85Gripper::Constraint::drawGui() {
    using tools::Gui;
    if (Gui::I->TreeNode("Link Constraint")) {
        for (uint iter = 0; auto& [info1, info2, weight] : infos) {
            if (Gui::I->TreeNode(("Info - " + std::to_string(iter++)).c_str())) {
                Gui::I->Text("Link1: %s", info1.first.c_str());
                Gui::I->Input("Local Coordinates 1", info1.second);

                Gui::I->Text("Link2: %s", info2.first.c_str());
                Gui::I->Input("Local Coordinates 2", info2.second);

                Gui::I->Input("Weight", weight);

                Gui::I->TreePop();
            }
        }

        Gui::I->Input("Gripper Length", gripperLength);

        Gui::I->TreePop();
    }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------

Robotiq2F85Gripper::Robotiq2F85Gripper(const robot::Robot& robot, const std::string& linkName, const tools::Transformation& localGripperTrafo,
                                       const tools::Transformation& localMountTrafo)
    : rapt::Gripper(robot, linkName, localGripperTrafo, "Robotiq2F85"),
      localMountTrafo(localMountTrafo),
      gripperRobot(folderPath + "/robot.urdf", robot.f_loadModel),
      gripperRobotState(Eigen::VectorXd::Zero(gripperRobot.getStateSize())),
      optimizer("Robotiq2F85GripperOptimizer"),
      constraint(gripperRobot) {
    optimizer.printInfos = false;
}

void Robotiq2F85Gripper::drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const {
    //Update constraint
    constraint.setTargetsFromFromFingerPercentage(fingerPercentage);

    //Compute joints
    Eigen::VectorXd joints = gripperRobotState.segment(6, 6);
    //constraint.testJacobian(joints);
    optimizer.optimize(joints, constraint, 100);

    //Get robot state
    gripperRobotState.segment(0, 6) = gripperRobot.base->getStateFromTransformation(globalLinkPose * localMountTrafo);
    gripperRobotState.segment(6, 6) = joints;

    //Draw debug info
    if (showDebugInfo)
        constraint.drawScene(gripperRobotState);

    //Draw robot
    gripperRobot.visualAlpha = alpha;
    gripperRobot.drawScene(gripperRobotState);

    //Draw base class
    rapt::Gripper::drawScene(globalLinkPose, alpha);
}

void Robotiq2F85Gripper::drawAdditionalGuiContent() {
    tools::Gui::I->Input("Local Mount Trafo", localMountTrafo);
    gripperRobot.drawGui(true);
    gripperRobot.drawFKGui(gripperRobotState, "Robot State");
    tools::Gui::I->Checkbox("Show Debug Info", showDebugInfo);
    constraint.drawGui();
}

}  // namespace lenny::agents