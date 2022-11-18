#pragma once

#include <lenny/rapt/Agent.h>

namespace lenny::agents {

class UR5eRobot : public robot::Robot {
public:
    UR5eRobot(const tools::Model::F_loadModel& f_loadModel = nullptr) : robot::Robot(folderPath + "/robot.urdf", f_loadModel) {}
    inline static const std::string folderPath = LENNY_ROBOT_FOLDER "/data/ur_5e";
};

//-----------------------------------------------------------------------------------------------------------------------------------------

class UR5eAgent : public rapt::Agent {
public:
    UR5eAgent(const std::string& name, const UR5eRobot& robot, const Eigen::VectorXd& initialRobotState);
    UR5eAgent(const std::string& name, const UR5eRobot& robot);
};

}  // namespace lenny::agents