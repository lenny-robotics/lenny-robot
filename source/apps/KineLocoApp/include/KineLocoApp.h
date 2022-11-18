#pragma once

#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>
#include <lenny/gui/Plot.h>
#include <lenny/kineloco/LocomotionController.h>
#include <lenny/tools/Animator.h>

namespace lenny {

class KineLocoApp : public gui::Application {
public:
    KineLocoApp();
    ~KineLocoApp() = default;

    //--- Process
    void restart() override;
    void process() override;

    //--- Drawing
    void drawScene() const override;
    void drawGui() override;

public:
    robot::Robot robot = robot::Robot(LENNY_ROBOT_FOLDER "/data/bd_spot/base/robot.urdf", gui::Model::f_loadModel);
    const Eigen::VectorXd initialSpotState = robot.loadStateFromFile(LENNY_ROBOT_FOLDER "/data/bd_spot/base/default_state.json").value();
    kineloco::LocomotionController locomotionController =
        kineloco::LocomotionController(robot, initialSpotState,
                                       {kineloco::LimbInfo("front_right_lower_leg", Eigen::Vector3d(0.0, -0.41, 0.0), {0.5, 1.0}),
                                        kineloco::LimbInfo("front_left_lower_leg", Eigen::Vector3d(0.0, -0.41, 0.0), {0.0, 0.5}),
                                        kineloco::LimbInfo("rear_right_lower_leg", Eigen::Vector3d(0.0, -0.38, 0.0), {0.0, 0.5}),
                                        kineloco::LimbInfo("rear_left_lower_leg", Eigen::Vector3d(0.0, -0.38, 0.0), {0.5, 1.0})});
    Eigen::VectorXd currentSpotState = initialSpotState;
    tools::Transformation targetBasePose = robot.base->getTransformationFromState(initialSpotState.segment(0, 6));
    uint numSteps = 100;
    double deltaT = 1.0 / 30.0;
    mutable tools::Animator animator = tools::Animator(numSteps, deltaT);
};

}  // namespace lenny