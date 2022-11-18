#pragma once

#include <lenny/rapt/Gripper.h>
#include <lenny/optimization/EqualityConstraint.h>
#include <lenny/optimization/NewtonOptimizer.h>

namespace lenny::agents {

class Robotiq2F85Gripper : public rapt::Gripper {
private:
    class Constraint : public optimization::EqualityConstraint {
    public:
        Constraint(const robot::Robot& robot);
        ~Constraint() = default;

        void setTargetsFromFromFingerPercentage(const double& fingerPercentage);

        uint getConstraintNumber() const override;
        void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& x) const override;
        void computeJacobian(Eigen::SparseMatrixD& pCpX, const Eigen::VectorXd& x) const override;
        void computeTensor(Eigen::TensorD& p2CpX2, const Eigen::VectorXd& x) const override;

        void drawScene(const Eigen::VectorXd& robotState) const;
        void drawGui() override;

    public:
        const robot::Robot& robot;
        typedef std::pair<std::string, Eigen::Vector3d> LinkInfo;              //[link name, local coordiantes]
        std::array<std::tuple<LinkInfo, LinkInfo, Eigen::Vector3d>, 4> infos;  //[link info 1, link info2, weights]
        double gripperLength = 0.085;
        std::array<std::pair<std::string, std::string>, 3> jointPairs;  //[joint1, joint2]
    };

public:
    Robotiq2F85Gripper(const robot::Robot& robot, const std::string& linkName, const tools::Transformation& localTrafo);
    ~Robotiq2F85Gripper() = default;

    void drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const override;

private:
    void drawAdditionalGuiContent();

public:
    inline static const std::string folderPath = LENNY_ROBOT_FOLDER "/data/robotiq_2f85gripper";
    mutable robot::Robot gripperRobot;
    bool showDebugInfo = false;

private:
    mutable Eigen::VectorXd gripperRobotState;
    optimization::NewtonOptimizer optimizer;
    mutable Constraint constraint;
};

}  // namespace lenny::agents