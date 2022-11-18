#pragma once

#include <lenny/rapt/Agent.h>

namespace lenny::agents {

class FloatingBaseRobot : public robot::Robot {
public:
    FloatingBaseRobot(const tools::Model::F_loadModel& f_loadModel = nullptr) : robot::Robot(folderPath + "/robot.urdf", f_loadModel) {}
    inline static const std::string folderPath = LENNY_ROBOT_FOLDER "/data/floating_base";
};

//-----------------------------------------------------------------------------------------------------------------------------------------

class FloatingBaseAgent : public rapt::Agent {
public:
    FloatingBaseAgent(const std::string& name, const FloatingBaseRobot& robot, const Eigen::VectorXd& initialRobotState);
    FloatingBaseAgent(const std::string& name, const FloatingBaseRobot& robot);
    ~FloatingBaseAgent() = default;
};

}  // namespace lenny::agents