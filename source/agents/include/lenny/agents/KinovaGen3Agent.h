#pragma once

#include <lenny/rapt/Agent.h>

namespace lenny::agents {

class KinovaGen3Robot : public robot::Robot {
public:
    KinovaGen3Robot(const tools::Model::F_loadModel& f_loadModel = nullptr) : robot::Robot(folderPath + "/robot.urdf", f_loadModel) {}
    inline static const std::string folderPath = LENNY_ROBOT_FOLDER "/data/kinova_gen3";
};

//-----------------------------------------------------------------------------------------------------------------------------------------

class KinovaGen3Agent : public rapt::Agent {
public:
    KinovaGen3Agent(const std::string& name, const KinovaGen3Robot& robot, const Eigen::VectorXd& initialRobotState);
    KinovaGen3Agent(const std::string& name, const KinovaGen3Robot& robot);
};

}  // namespace lenny::agents