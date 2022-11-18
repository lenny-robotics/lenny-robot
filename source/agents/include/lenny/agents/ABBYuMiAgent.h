#pragma once

#include <lenny/rapt/Agent.h>

namespace lenny::agents {

class ABBYuMiRobot : public robot::Robot {
public:
    ABBYuMiRobot(const tools::Model::F_loadModel& f_loadModel = nullptr) : robot::Robot(folderPath + "/robot.urdf", f_loadModel) {}
    inline static const std::string folderPath = LENNY_ROBOT_FOLDER "/data/abb_yumi";
};

//-------------------------------------------------------------------------------------------------------------------------------------

class ABBYuMiAgent : public rapt::Agent {
public:
    class Gripper : public rapt::Gripper {
    public:
        enum SIDE { LEFT, RIGHT };
        Gripper(const ABBYuMiRobot& robot, const SIDE side);
        ~Gripper() = default;

        void drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const override;

    public:
        const SIDE side;
        enum VISUAL { BASE, FINGER1, FINGER2 };
    };

    ABBYuMiAgent(const std::string& name, const ABBYuMiRobot& robot, const Eigen::VectorXd& initialRobotState);
    ABBYuMiAgent(const std::string& name, const ABBYuMiRobot& robot);
};

}  // namespace lenny::agents