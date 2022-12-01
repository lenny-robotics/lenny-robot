#pragma once

#include <lenny/robot/Robot.h>

#include <iostream>

void testTransformationsAndDerivatives() {
    using namespace lenny;

    auto checkTransformations = [](const tools::Transformation& trafo, const tools::Transformation& test) {
        if (trafo.isApprox(test)) {
            LENNY_LOG_PRINT(tools::Logger::GREEN, "Test PASSED\n");
        } else {
            LENNY_LOG_PRINT(tools::Logger::RED, "Test FAILED\n");
            std::cout << trafo << " VS " << test << std::endl;
        }
    };

    auto checkVectors = [&](const Eigen::Vector3d& vec, const Eigen::Vector3d& test) {
        checkTransformations(tools::Transformation(vec, Eigen::QuaternionD::Identity()), tools::Transformation(test, Eigen::QuaternionD::Identity()));
    };

    auto checkOrientations = [&](const Eigen::QuaternionD& quat, const Eigen::QuaternionD& test) {
        checkTransformations(tools::Transformation(Eigen::Vector3d::Zero(), quat), tools::Transformation(Eigen::Vector3d::Zero(), test));
    };

    const robot::Robot robot(LENNYROBOT_TESTAPP_FOLDER "/config/testRobot1.urdf", nullptr);

    const Eigen::VectorXd state = Eigen::VectorXd::Random(robot.getStateSize());

    robot::Robot::LinkPoses globalLinkPoses;
    robot.computeGlobalLinkPoses(globalLinkPoses, state);

    for (const auto& [linkName, globalPose] : globalLinkPoses) {
        std::cout << "LINK: " << linkName << std::endl;

        std::cout << "computeGlobalPoint: ";
        const Eigen::Vector3d p_local(Eigen::Vector3d::Random());
        const Eigen::Vector3d p_global = robot.computeGlobalPoint(state, p_local, linkName);
        checkVectors(p_global, globalLinkPoses.at(linkName).getGlobalCoordinatesForPoint(p_local));

        std::cout << "computeLocalPoint ";
        checkVectors(robot.computeLocalPoint(state, p_global, linkName), p_local);

        robot.testPointJacobian(state, p_local, linkName);
        robot.testPointTensor(state, p_local, linkName);

        std::cout << "computeGlobalVector: ";
        const Eigen::Vector3d v_local(Eigen::Vector3d::Random());
        const Eigen::Vector3d v_global = robot.computeGlobalVector(state, v_local, linkName);
        checkVectors(v_global, globalLinkPoses.at(linkName).getGlobalCoordinatesForVector(v_local));

        std::cout << "computeLocalVector ";
        checkVectors(robot.computeLocalVector(state, v_global, linkName), v_local);

        robot.testVectorJacobian(state, v_local, linkName);
        robot.testVectorTensor(state, v_local, linkName);

        std::cout << "computeGlobalOrientation: ";
        const Eigen::QuaternionD q_local(Eigen::QuaternionD::UnitRandom());
        const Eigen::QuaternionD q_global = robot.computeGlobalOrientation(state, q_local, linkName);
        checkOrientations(q_global, globalLinkPoses.at(linkName).getGlobalCoordinates(q_local));

        std::cout << "computeLocalOrientation ";
        checkOrientations(robot.computeLocalOrientation(state, q_global, linkName), q_local);

        std::cout << "computeGlobalPose: ";
        const tools::Transformation t_local(Eigen::Vector3d::Random(), Eigen::QuaternionD::UnitRandom());
        const tools::Transformation t_global = robot.computeGlobalPose(state, t_local, linkName);
        checkTransformations(t_global, globalLinkPoses.at(linkName).getGlobalCoordinates(t_local));

        std::cout << "computeLocalPose ";
        checkTransformations(robot.computeLocalPose(state, t_global, linkName), t_local);

        std::cout << "-----------------------------------------------" << std::endl;
    }
}

void testNumberOfJointsInbetween() {
    using namespace lenny;

    const robot::Robot robot(LENNYROBOT_TESTAPP_FOLDER "/config/testRobot2.urdf", nullptr);

    std::cout << "NUMBER OF LINKS INBETWEEN:" << std::endl;
    for (const auto& [linkName_A, link_A] : robot.links) {
        for (const auto& [linkName_B, link_B] : robot.links) {
            const int numJointsInbetween = robot.getNumberOfJointsInbetween(linkName_A, linkName_B);
            std::cout << "Link A: " << linkName_A.c_str() << " / Link B: " << linkName_B.c_str() << " -> " << numJointsInbetween << std::endl;
        }
    }
    std::cout << "---------------------------" << std::endl;
}

void testFreeFloatingBase() {
    const lenny::robot::Robot robot(LENNYROBOT_TESTAPP_FOLDER "/config/testRobot3.urdf", nullptr);
    std::cout << "FREE FLOATING BASE TEST PASSED" << std::endl;
}
