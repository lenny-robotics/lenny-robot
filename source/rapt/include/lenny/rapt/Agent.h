#pragma once

#include <lenny/collision/Primitive.h>
#include <lenny/rapt/Gripper.h>
#include <lenny/robot/Robot.h>

namespace lenny::rapt {

class Agent {
public:
    //--- Typedefs
    LENNY_GENERAGE_TYPEDEFS(Agent)

    //--- Constructor
    Agent(const std::string& name, const robot::Robot& robot, const Eigen::VectorXd& initialRobotState, const Eigen::VectorXb& dofMask);
    virtual ~Agent() = default;

    //--- Public helpers
    uint getStateSize() const;                                 //Returns size of AGENT state
    void checkState(const Eigen::VectorXd& agentState) const;  //Checks AGENT state

    Eigen::VectorXd getAgentStateFromRobotState(const Eigen::VectorXd& robotState) const;
    Eigen::VectorXd getRobotStateFromAgentState(const Eigen::VectorXd& agentState) const;

    Eigen::VectorXd getInitialAgentState() const;
    Eigen::VectorXd getInitialRobotState() const;

    void setInitialRobotStateFromRobotState(const Eigen::VectorXd& robotState);
    void setInitialRobotStateFromAgentState(const Eigen::VectorXd& agentState);

    Eigen::VectorXd getInitialAgentVelocity() const;
    Eigen::VectorXd getInitialRobotVelocity() const;

    void setInitialRobotVelocityFromRobotVelocity(const Eigen::VectorXd& robotVelocity);
    void setInitialRobotVelocityFromAgentVelocity(const Eigen::VectorXd& agentVelocity);

    void setDofMask(const Eigen::VectorXb& dofMask);
    const Eigen::VectorXb& getDofMask() const;

    uint getRobotDofIndexFromAgentDofIndex(const uint& agentDofIndex) const;
    const robot::Limits& getLimitsForDofIndex(const uint& agentDofIndex, const robot::Robot::LIMITS_TYPE limitsType) const;
    bool isPositionalDof(const uint& agentDofIndex) const;
    std::string getDescriptionForDofIndex(const uint& agentDofIndex) const;

    //--- Computations
    Eigen::Vector3d computeGlobalPoint(const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_local, const std::string& linkName) const;
    void computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_local, const std::string& linkName) const;
    void computePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_local, const std::string& linkName) const;

    Eigen::Vector3d computeGlobalVector(const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_local, const std::string& linkName) const;
    void computeVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_local, const std::string& linkName) const;
    void computeVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_local, const std::string& linkName) const;

    //--- Tests
    void testPointJacobian(const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_local, const std::string& linkName) const;
    void testPointTensor(const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_local, const std::string& linkName) const;

    void testVectorJacobian(const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_local, const std::string& linkName) const;
    void testVectorTensor(const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_local, const std::string& linkName) const;

    //--- Transformation helpers
    Eigen::QuaternionD computeGlobalOrientation(const Eigen::VectorXd& agentState, const Eigen::QuaternionD& q_local, const std::string& linkName) const;
    tools::Transformation computeGlobalPose(const Eigen::VectorXd& agentState, const tools::Transformation& t_local, const std::string& linkName) const;

    Eigen::Vector3d computeLocalPoint(const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_global, const std::string& linkName) const;
    Eigen::Vector3d computeLocalVector(const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_global, const std::string& linkName) const;
    Eigen::QuaternionD computeLocalOrientation(const Eigen::VectorXd& agentState, const Eigen::QuaternionD& q_global, const std::string& linkName) const;
    tools::Transformation computeLocalPose(const Eigen::VectorXd& agentState, const tools::Transformation& t_global, const std::string& linkName) const;

    //--- Animation helpers
    class MotionTrajectory {
    public:
        MotionTrajectory(const Eigen::VectorXd& data, const uint& numSteps, const double& deltaT) : data(data), numSteps(numSteps), deltaT(deltaT) {}
        ~MotionTrajectory() = default;

        void check(const uint& agentStateSize) const;
        double getTotalTime() const;
        int getIndexForTime(const double& time) const;
        double getTimeForIndex(const int& index) const;

    public:
        const Eigen::VectorXd& data;
        const uint& numSteps;
        const double& deltaT;
    };
    Eigen::VectorXd getAgentStateForTrajectoryIndex(const MotionTrajectory& trajectory, const int& index) const;
    Eigen::VectorXd getAgentStateForTrajectoryTime(const MotionTrajectory& trajectory, const double& time) const;

    //--- Drawing
    virtual void drawScene(const MotionTrajectory& trajectory, const double& currentTime, const bool& isRecedingHorizon) const;
    virtual void drawScene(const Eigen::VectorXd& agentState) const;
    void drawGui(const bool withDrawingOptions = false);

    //--- Collision primitives
    void addCollisionSphere(const std::string& linkName, const Eigen::Vector3d& localPosition, const double& radius);
    void addCollisionCapsule(const std::string& linkName, const Eigen::Vector3d& localStartPosition, const Eigen::Vector3d& localEndPosition,
                             const double& radius);
    void addCollisionRectangle(const std::string& linkName, const Eigen::Vector3d& localCenterPoint, const Eigen::QuaternionD& localOrientation,
                               const Eigen::Vector2d& localDimensions, const double& safetyMargin);
    void addCollisionBox(const std::string& linkName, const Eigen::Vector3d& localCenterPoint, const Eigen::QuaternionD& localOrientation,
                         const Eigen::Vector3d& localDimensions, const double& safetyMargin);

    bool saveCollisionPrimitivesToFile(const std::string& filePath) const;
    bool loadCollisionPrimitivesFromFile(const char* filePath);

    //--- Self collision link map
    void generateSelfCollisionLinkMap(const uint& ignoreConsecutiveLinksIndex);
    bool saveSelfCollisionLinkMapToFile(const std::string& filePath) const;
    bool loadSelfCollisionLinkMapFromFile(const char* filePath);

protected:
    //--- Protected helpers
    void convertRobotJacobianToAgentJacobian(Eigen::MatrixXd& agentJacobian, const Eigen::MatrixXd& robotJacobian) const;
    void convertRobotTensorToAgentTensor(Eigen::TensorD& agentTensor, const Eigen::TensorD& robotTensor) const;

    //--- Drawing
    void drawRobot(const Eigen::VectorXd& agentState) const;
    void drawGrippers(const Eigen::VectorXd& agentState) const;
    void drawCollisionPrimitives(const Eigen::VectorXd& agentState) const;
    virtual void drawAdditionalGuiContent() {}

public:
    //--- Public members
    const std::string name;
    const robot::Robot& robot;
    std::vector<Gripper::UPtr> grippers;
    std::unordered_map<std::string, std::vector<collision::Primitive::SPtr>> collisionPrimitives;  //[linkName, primitives]
    std::unordered_map<std::string, std::vector<std::string>> selfCollisionLinkMap;                //[linkName, linkNames]
    tools::Transformation localBaseTrafo = tools::Transformation();                                //For base constraints
    Eigen::QuaternionD nominalBaseRotation = Eigen::QuaternionD::Identity();                       //For base pose constraints

    //--- Drawing
    double infoAlpha = 1.0;
    double visualAlpha = 1.0;

    bool showSkeleton = false;
    bool showCoordinateFrames = false;
    bool showJointAxes = false;
    bool showJointLimits = false;
    bool showVisuals = true;
    bool showGrippers = true;
    bool showCollisionPrimitives = false;

protected:
    //--- Protected members
    Eigen::VectorXd initialRobotState;     //Full generalized coordinate state of the ROBOT
    Eigen::VectorXd initialRobotVelocity;  //Full generalized coordinate velocity of the ROBOT
    Eigen::VectorXb dofMask;               //Size of full generalized coordinate state of the ROBOT

    static tools::FiniteDifference fd;
};

}  // namespace lenny::rapt
