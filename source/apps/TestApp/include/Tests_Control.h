#pragma once

//#include <gtest/gtest.h> //ToDo!!
#include <lenny/control/BasicTrajectoryTracker.h>
#include <lenny/control/EmulatorControlInterface.h>
#include <lenny/gui/Plot.h>
#include <lenny/tools/Timer.h>

//TEST(control, interface) {
void run_control_test() {
    using namespace lenny;

    //Initialize components
    const robot::Robot robot(LENNY_ROBOT_APP_FOLDER "/config/test_robots/testRobot1.urdf", nullptr);
    Eigen::VectorXb dofMask = Eigen::VectorXb::Ones(robot.getStateSize());
    dofMask.segment(0, 6).setZero();
    control::EmulatorControlInterface rci =
        control::EmulatorControlInterface(robot, dofMask, gui::Plot<control::EmulatorControlInterface::PlotType>::f_addPlot);
    control::BasicTrajectoryTracker btt = control::BasicTrajectoryTracker(rci);

    //Connect robot
    rci.initializeCommunication();

    //Perform tests several times
    for (int i = 0; i < 3; i++) {
        //Clear recorded data
        rci.clearPlots();

        //Set random delay
        rci.emulator.noiseFactor = 0.0;
        rci.emulator.timeDelay = tools::utils::getRandomNumberInRange({0.1, 0.5});
        LENNY_LOG_DEBUG("-------------------------------------------------------------------------------------------------------------------------------------")
        LENNY_LOG_DEBUG("Emulator time delay: %lf", rci.emulator.timeDelay)

        //Goto random state in random time
        const Eigen::VectorXd targetRobotState = PI * Eigen::VectorXd::Random(robot.getStateSize());
        btt.gotoStateInTime(targetRobotState, tools::utils::getRandomNumberInRange({1.0, 5.0}));
        while (!btt.isTrackingRunning()) {
        }  //First wait until tracking is started
        while (btt.isTrackingRunning()) {
        }  //Now wait until tracking is finished

        //Estimate delay
        std::map<std::string, double> estimatePerDof;
        rci.estimateDelay(estimatePerDof);

        //Evaluate
        for (const auto& [dofName, estimate] : estimatePerDof) {
            const double absError = std::abs(rci.emulator.timeDelay - estimate);
            LENNY_LOG_DEBUG("Absolute error for dof `%s`: %lf", dofName.c_str(), absError)
        }
    }

    //Disconnect
    rci.terminateCommunication();
}