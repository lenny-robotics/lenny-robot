#pragma once

#include <lenny/robot/Base.h>
#include <lenny/robot/EndEffector.h>
#include <lenny/robot/Joint.h>
#include <lenny/robot/Link.h>
#include <lenny/tools/FiniteDifference.h>

#include <map>

namespace lenny::robot {

class Robot {
public:
    //--- Constructor
    Robot(const std::string& filePath, const tools::Model::F_loadModel& f_loadModel);
    Robot(Robot&&) = default;
    virtual ~Robot() = default;

    //--- Initialize
    void loadFromFile();

    //--- Checkers
    void checkLinkName(const std::string& linkName) const;
    void checkJointName(const std::string& jointName) const;
    void checkState(const Eigen::VectorXd& state) const;
    void checkDofMask(const Eigen::VectorXb& dofMask) const;

    //--- Helpers
    uint getStateSize() const;
    uint getStateIndex(const std::string& jointName) const;
    std::string getDescriptionForDofIndex(const uint& dofIndex) const;
    enum LIMITS_TYPE { POSITION, VELOCITY, ACCELERATION };
    const robot::Limits& getLimitsForDofIndex(const uint& dofIndex, const LIMITS_TYPE limitsType) const;
    int getNumberOfJointsInbetween(const std::string& linkName_A, const std::string& linkName_B) const;

    //--- Computations
    Eigen::Vector3d computeGlobalPoint(const Eigen::VectorXd& state, const Eigen::Vector3d& p_local, const std::string& linkName) const;
    void computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& p_local, const std::string& linkName,
                              const Eigen::VectorXb& dofMask) const;
    void computePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& p_local, const std::string& linkName,
                            const Eigen::VectorXb& dofMask) const;

    Eigen::Vector3d computeGlobalVector(const Eigen::VectorXd& state, const Eigen::Vector3d& v_local, const std::string& linkName) const;
    void computeVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& v_local, const std::string& linkName,
                               const Eigen::VectorXb& dofMask) const;
    void computeVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& v_local, const std::string& linkName,
                             const Eigen::VectorXb& dofMask) const;

    //--- Tests
    void testPointJacobian(const Eigen::VectorXd& state, const Eigen::Vector3d& p_local, const std::string& linkName) const;
    void testPointTensor(const Eigen::VectorXd& state, const Eigen::Vector3d& p_local, const std::string& linkName) const;

    void testVectorJacobian(const Eigen::VectorXd& state, const Eigen::Vector3d& v_local, const std::string& linkName) const;
    void testVectorTensor(const Eigen::VectorXd& state, const Eigen::Vector3d& v_local, const std::string& linkName) const;

    //--- Transformation helpers
    Eigen::QuaternionD computeGlobalOrientation(const Eigen::VectorXd& state, const Eigen::QuaternionD& q_local, const std::string& linkName) const;
    tools::Transformation computeGlobalPose(const Eigen::VectorXd& state, const tools::Transformation& t_local, const std::string& linkName) const;

    Eigen::Vector3d computeLocalPoint(const Eigen::VectorXd& state, const Eigen::Vector3d& p_global, const std::string& linkName) const;
    Eigen::Vector3d computeLocalVector(const Eigen::VectorXd& state, const Eigen::Vector3d& v_global, const std::string& linkName) const;
    Eigen::QuaternionD computeLocalOrientation(const Eigen::VectorXd& state, const Eigen::QuaternionD& q_global, const std::string& linkName) const;
    tools::Transformation computeLocalPose(const Eigen::VectorXd& state, const tools::Transformation& t_global, const std::string& linkName) const;

    typedef std::map<std::string, tools::Transformation> LinkPoses;
    void computeGlobalLinkPoses(LinkPoses& globalLinkPoses, const Eigen::VectorXd& state) const;

    //--- Drawing
    void drawScene(const Eigen::VectorXd& state, const std::map<std::string, Eigen::VectorXd>& endEffectorStates) const;
    enum class DRAWING_FLAGS : std::uint8_t {
        SHOW_NONE = 0b00000000,
        SHOW_SKELETON = 0b00000001,
        SHOW_COORDINATE_FRAMES = 0b00000010,
        SHOW_JOINT_AXES = 0b00000100,
        SHOW_JOINT_LIMITS = 0b00001000,
        SHOW_VISUALS = 0b00010000,
        SHOW_GRASP_LOCATIONS = 0b00100000
    };
    void drawScene(const Eigen::VectorXd& state, const std::map<std::string, Eigen::VectorXd>& endEffectorStates, const DRAWING_FLAGS& flags,
                   const double& visualAlpha, const double& infoAlpha) const;
    void drawVisuals(const Eigen::VectorXd& state, const std::map<std::string, Eigen::VectorXd>& endEffectorStates, const std::optional<Eigen::Vector3d>& color,
                     const double& alpha) const;
    void drawSkeleton(const Eigen::VectorXd& state, const double& radius, const Eigen::Vector4d& linkColor, const Eigen::Vector4d& jointColor) const;

    //--- Gui
    void drawGui(const bool withDrawingOptions = false);
    bool drawFKGui(Eigen::VectorXd& state, const char* label) const;

    //--- Interaction ([linkName, globalIntersectionPoint]
    std::optional<std::pair<std::string, Eigen::Vector3d>> getFirstLinkHitByRay(const Eigen::VectorXd& state, const Ray& ray) const;

    //--- Save & load
    bool saveStateToFile(const Eigen::VectorXd& state, const std::string& filePath) const;
    std::optional<Eigen::VectorXd> loadStateFromFile(const char* filePath) const;

protected:
    //--- Computation helpers
    template <typename T>
    T computeGlobal(const Eigen::VectorXd& state, const T& local, const std::string& linkName,
                    const std::function<T(const tools::Transformation& trafo, const T& local)>& getGlobalCoordinates) const;
    void computeJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& local, const std::string& linkName,
                         const Eigen::VectorXb& dofMask,
                         const std::function<Eigen::Vector3d(const tools::Transformation& trafo, const Eigen::Vector3d& local)>& getGlobalCoordinates) const;
    void computeTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& local, const std::string& linkName,
                       const Eigen::VectorXb& dofMask,
                       const std::function<Eigen::Vector3d(const tools::Transformation& trafo, const Eigen::Vector3d& local)>& getGlobalCoordinates) const;

    //--- Drawing
    void drawSkeleton(const LinkPoses& globalLinkPoses, const double& radius, const Eigen::Vector4d& linkColor, const Eigen::Vector4d& jointColor) const;
    void drawCoordinateFrames(const LinkPoses& globalLinkPoses) const;
    void drawJointAxes(const LinkPoses& globalLinkPoses) const;
    void drawJointLimits(const Eigen::VectorXd& state, const LinkPoses& globalLinkPoses) const;
    void drawVisuals(const LinkPoses& globalLinkPoses, const std::map<std::string, Eigen::VectorXd>& endEffectorStates,
                     const std::optional<Eigen::Vector3d>& color, const double& alpha) const;
    void drawGraspLocations(const LinkPoses& globalLinkPoses) const;

public:
    //--- Info
    std::string name;                             //Name of the robot
    const std::string filePath;                   //Store file path
    const tools::Model::F_loadModel f_loadModel;  //Store model load function

    //--- Members
    Base::UPtr base;
    std::map<std::string, Link> links;
    std::map<std::string, Joint> joints;
    std::map<std::string, EndEffector> endEffectors;

    //--- Drawing
    double skeletonRadius = 0.01;
    double infoAlpha = 1.0;
    double visualAlpha = 1.0;

    bool showSkeleton = false;
    bool showCoordinateFrames = false;
    bool showJointAxes = false;
    bool showJointLimits = false;
    bool showVisuals = true;
    bool showGraspLocations = false;

protected:
    typedef std::pair<std::optional<std::string>, std::vector<std::string>> ParentToChildrenJoints;  //[parent, children]
    std::map<std::string, ParentToChildrenJoints> linkToJoints;                                      //[link, ParentToChildren]

    static tools::FiniteDifference fd;
};

//--- Drawing flag operator overloads
inline Robot::DRAWING_FLAGS operator|(Robot::DRAWING_FLAGS lhs, Robot::DRAWING_FLAGS rhs) {
    return static_cast<Robot::DRAWING_FLAGS>(static_cast<std::underlying_type_t<Robot::DRAWING_FLAGS>>(lhs) |
                                             static_cast<std::underlying_type_t<Robot::DRAWING_FLAGS>>(rhs));
}

inline Robot::DRAWING_FLAGS& operator|=(Robot::DRAWING_FLAGS& lhs, Robot::DRAWING_FLAGS rhs) {
    lhs = lhs | rhs;
    return lhs;
}

inline bool operator&(Robot::DRAWING_FLAGS lhs, Robot::DRAWING_FLAGS rhs) {
    return static_cast<std::underlying_type_t<Robot::DRAWING_FLAGS>>(lhs) & static_cast<std::underlying_type_t<Robot::DRAWING_FLAGS>>(rhs);
}

}  // namespace lenny::robot
