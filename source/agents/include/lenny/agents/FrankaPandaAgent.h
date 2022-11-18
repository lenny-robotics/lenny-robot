#pragma once

#include <lenny/rapt/Agent.h>

namespace lenny::agents {

class FrankaPandaRobot : public robot::Robot {
public:
    FrankaPandaRobot(const tools::Model::F_loadModel& f_loadModel = nullptr) : robot::Robot(folderPath + "/robot.urdf", f_loadModel) {}
    inline static const std::string folderPath = LENNY_ROBOT_FOLDER "/data/franka_panda";
};

//-----------------------------------------------------------------------------------------------------------------------------------------

class FrankaPandaAgent : public rapt::Agent {
public:
    class Gripper : public rapt::Gripper {
    public:
        Gripper(const robot::Robot& robot);
        ~Gripper() = default;

        void drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const override;

    public:
        enum VISUALS { HAND, FINGER1, FINGER2 };
    };

    FrankaPandaAgent(const std::string& name, const FrankaPandaRobot& robot, const Eigen::VectorXd& initialRobotState);
    FrankaPandaAgent(const std::string& name, const FrankaPandaRobot& robot);
};

}  // namespace lenny::agents