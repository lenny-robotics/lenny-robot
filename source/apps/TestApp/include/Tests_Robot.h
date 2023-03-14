#pragma once

#include <gtest/gtest.h>
#include <lenny/robot/Robot.h>

TEST(robot, derivatives) {
    using namespace lenny;

    const robot::Robot robot(LENNY_ROBOT_APP_FOLDER "/config/test_robots/testRobot1.urdf", nullptr);
    const Eigen::VectorXd state = Eigen::VectorXd::Random(robot.getStateSize());

    for (const auto& [linkName, link] : robot.links) {
        //POINT
        const Eigen::Vector3d p_local(Eigen::Vector3d::Random());
        EXPECT_TRUE(robot.testPointJacobian(state, p_local, linkName));
        EXPECT_TRUE(robot.testPointTensor(state, p_local, linkName));

        //VECTOR
        const Eigen::Vector3d v_local(Eigen::Vector3d::Random());
        EXPECT_TRUE(robot.testVectorJacobian(state, v_local, linkName));
        EXPECT_TRUE(robot.testVectorTensor(state, v_local, linkName));
    }
}

TEST(robot, transformations) {
    using namespace lenny;

    const robot::Robot robot(LENNY_ROBOT_APP_FOLDER "/config/test_robots/testRobot1.urdf", nullptr);
    const Eigen::VectorXd state = Eigen::VectorXd::Random(robot.getStateSize());

    robot::Robot::LinkPoses globalLinkPoses;
    robot.computeGlobalLinkPoses(globalLinkPoses, state);

    for (const auto& [linkName, globalPose] : globalLinkPoses) {
        //POINT
        const Eigen::Vector3d p_local(Eigen::Vector3d::Random());
        const Eigen::Vector3d p_global = robot.computeGlobalPoint(state, p_local, linkName);
        EXPECT_TRUE(p_global.isApprox(globalLinkPoses.at(linkName).getGlobalCoordinatesForPoint(p_local)));
        EXPECT_TRUE(p_local.isApprox(robot.computeLocalPoint(state, p_global, linkName)));

        //VECTOR
        const Eigen::Vector3d v_local(Eigen::Vector3d::Random());
        const Eigen::Vector3d v_global = robot.computeGlobalVector(state, v_local, linkName);
        EXPECT_TRUE(v_global.isApprox(globalLinkPoses.at(linkName).getGlobalCoordinatesForVector(v_local)));
        EXPECT_TRUE(v_local.isApprox(robot.computeLocalVector(state, v_global, linkName)));

        //ORIENTATION
        const Eigen::QuaternionD q_local(Eigen::QuaternionD::UnitRandom());
        const Eigen::QuaternionD q_global = robot.computeGlobalOrientation(state, q_local, linkName);
        EXPECT_TRUE(q_global.isApprox(globalLinkPoses.at(linkName).getGlobalCoordinates(q_local)));
        EXPECT_TRUE(q_local.isApprox(robot.computeLocalOrientation(state, q_global, linkName)));

        //TRANSFORMATION
        const tools::Transformation t_local(Eigen::Vector3d::Random(), Eigen::QuaternionD::UnitRandom());
        const tools::Transformation t_global = robot.computeGlobalPose(state, t_local, linkName);
        EXPECT_TRUE(t_global.isApprox(globalLinkPoses.at(linkName).getGlobalCoordinates(t_local)));
        EXPECT_TRUE(t_local.isApprox(robot.computeLocalPose(state, t_global, linkName)));
    }
}

TEST(robot, numberOfJointsInbetween) {
    using namespace lenny;

    const robot::Robot robot(LENNY_ROBOT_APP_FOLDER "/config/test_robots/testRobot2.urdf", nullptr);

    std::map<std::pair<std::string, std::string>, int> lookup = {
        {{"0", "0"}, 0},          {{"0", "1"}, 1},          {{"0", "1_1"}, 2},        {{"0", "1_1_1"}, 3},     {{"0", "1_2"}, 2},      {{"0", "1_2_1"}, 3},
        {{"0", "2"}, 1},          {{"0", "2_1"}, 2},        {{"0", "2_1_1"}, 3},      {{"0", "2_2"}, 2},       {{"0", "2_2_1"}, 3},    {{"1", "1"}, 0},
        {{"1", "1_1"}, 1},        {{"1", "1_1_1"}, 2},      {{"1", "1_2"}, 1},        {{"1", "1_2_1"}, 2},     {{"1", "2"}, -1},       {{"1", "2_1"}, -1},
        {{"1", "2_1_1"}, -1},     {{"1", "2_2"}, -1},       {{"1", "2_2_1"}, -1},     {{"1_1", "1_1"}, 0},     {{"1_1", "1_1_1"}, 1},  {{"1_1", "1_2"}, -1},
        {{"1_1", "1_2_1"}, -1},   {{"1_1", "2"}, -1},       {{"1_1", "2_1"}, -1},     {{"1_1", "2_1_1"}, -1},  {{"1_1", "2_2"}, -1},   {{"1_1", "2_2_1"}, -1},
        {{"1_1_1", "1_1_1"}, 0},  {{"1_1_1", "1_2"}, -1},   {{"1_1_1", "1_2_1"}, -1}, {{"1_1_1", "2"}, -1},    {{"1_1_1", "2_1"}, -1}, {{"1_1_1", "2_1_1"}, -1},
        {{"1_1_1", "2_2"}, -1},   {{"1_1_1", "2_2_1"}, -1}, {{"1_2", "1_2"}, 0},      {{"1_2", "1_2_1"}, 1},   {{"1_2", "2"}, -1},     {{"1_2", "2_1"}, -1},
        {{"1_2", "2_1_1"}, -1},   {{"1_2", "2_2"}, -1},     {{"1_2", "2_2_1"}, -1},   {{"1_2_1", "1_2_1"}, 0}, {{"1_2_1", "2"}, -1},   {{"1_2_1", "2_1"}, -1},
        {{"1_2_1", "2_1_1"}, -1}, {{"1_2_1", "2_2"}, -1},   {{"1_2_1", "2_2_1"}, -1}, {{"2", "2"}, 0},         {{"2", "2_1"}, 1},      {{"2", "2_1_1"}, 2},
        {{"2", "2_2"}, 1},        {{"2", "2_2_1"}, 2},      {{"2_1", "2_1"}, 0},      {{"2_1", "2_1_1"}, 1},   {{"2_1", "2_2"}, -1},   {{"2_1", "2_2_1"}, -1},
        {{"2_1_1", "2_1_1"}, 0},  {{"2_1_1", "2_2"}, -1},   {{"2_1_1", "2_2_1"}, -1}, {{"2_2", "2_2"}, 0},     {{"2_2", "2_2_1"}, 1},  {{"2_2_1", "2_2_1"}, 0}};

    for (const auto& [linkName_A, link_A] : robot.links) {
        for (const auto& [linkName_B, link_B] : robot.links) {
            const int numJointsInbetween = robot.getNumberOfJointsInbetween(linkName_A, linkName_B);
            const std::string substr_A = linkName_A.substr(4, linkName_A.length() - 4);
            const std::string substr_B = linkName_B.substr(4, linkName_B.length() - 4);
            if (lookup.find({substr_A, substr_B}) != lookup.end()) {
                EXPECT_EQ(lookup.at({substr_A, substr_B}), numJointsInbetween);
            } else if (lookup.find({substr_B, substr_A}) != lookup.end()) {
                EXPECT_EQ(lookup.at({substr_B, substr_A}), numJointsInbetween);
            } else {
                LENNY_LOG_ERROR("Could not find `%s - %s` in lookup", substr_A.c_str(), substr_B.c_str())
            }
        }
    }
}

TEST(robot, freeFloatingBase) {
    const lenny::robot::Robot robot(LENNY_ROBOT_APP_FOLDER "/config/test_robots/testRobot3.urdf", nullptr);
}

//ToDo: Test for velocity and acceleration estimates!