#include <lenny/robot/Robot.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Renderer.h>
#include <lenny/urdf/Model.h>

#include <fstream>

//helper type
template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};

//explicit deduction guide
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

namespace lenny::robot {

tools::FiniteDifference Robot::fd = tools::FiniteDifference("Robot");

Robot::Robot(const std::string& filePath, const tools::Model::F_loadModel& f_loadModel) : filePath(filePath), f_loadModel(f_loadModel) {
    loadFromFile();
}

void Robot::loadFromFile() {
    //--- Load robot using urdf loader
    urdf::Model urdfModel(filePath);

    //--- Define helpers
    auto rpy_to_q = [&](const double& roll, const double& pitch, const double& yaw) -> Eigen::QuaternionD {
        return Eigen::QuaternionD(tools::utils::rotZ(yaw) * tools::utils::rotY(pitch) * tools::utils::rotX(roll));
    };

    auto transform_to_trafo = [&](const urdf::Transform& t) -> tools::Transformation {
        return tools::Transformation(Eigen::Vector3d(t.position.x, t.position.y, t.position.z), rpy_to_q(t.rotation.roll, t.rotation.pitch, t.rotation.yaw));
    };

    auto vector3_to_vector3d = [&](const urdf::Vector3& v) -> Eigen::Vector3d { return Eigen::Vector3d(v.x, v.y, v.z); };

    auto material_by_name = [&](const std::string& name) -> const urdf::Material& {
        for (const urdf::Material& material : urdfModel.materials)
            if (material.name == name)
                return material;
        LENNY_LOG_ERROR("Material with name `%s` could not be found");
        return urdfModel.materials.at(0);
    };

    //--- Make preparations
    //Find urdf base (does not appear as child in any joint)
    std::vector<uint> urdfBaseIndices;
    for (uint i = 0; i < urdfModel.links.size(); i++) {
        bool hasAChild = false;
        for (const urdf::Joint& joint : urdfModel.joints) {
            if (joint.child_link_name == urdfModel.links.at(i).name) {
                hasAChild = true;
                break;
            }
        }
        if (!hasAChild)
            urdfBaseIndices.push_back(i);
    }
    if (urdfBaseIndices.size() != 1)
        LENNY_LOG_ERROR("There should be only one urdf base index, but there are `%d`", urdfBaseIndices.size());
    const urdf::Link& urdfBaseLink = urdfModel.links.at(urdfBaseIndices.at(0));

    //--- Gather global link poses
    //Initialize
    std::map<std::string, std::optional<tools::Transformation>> globalLinkPoses;
    for (const urdf::Link& link : urdfModel.links)
        globalLinkPoses.insert({link.name, std::nullopt});

    //Rotate base, since our coordinate system is different (y axis points up)
    globalLinkPoses.at(urdfBaseLink.name) = tools::Transformation(Eigen::Vector3d::Zero(), tools::utils::rotX(-PI / 2.0));

    //Iterate over joints (we do it several times until all of them could be set)
    bool trafoNotFound = true;
    int iter = 0;
    while (trafoNotFound && iter++ < 10) {
        trafoNotFound = false;
        for (const urdf::Joint& joint : urdfModel.joints) {
            if (!globalLinkPoses.at(joint.child_link_name).has_value()) {
                if (globalLinkPoses.at(joint.parent_link_name).has_value())
                    globalLinkPoses.at(joint.child_link_name) =
                        globalLinkPoses.at(joint.parent_link_name).value() * transform_to_trafo(joint.parent_to_joint_transform);
                else
                    trafoNotFound = true;
            }
        }
    }
    if (trafoNotFound)
        LENNY_LOG_ERROR("There is at least one global transformation that could not be found");

    //--- Set robot name
    this->name = urdfModel.name.has_value() ? urdfModel.name.value() : "Robot";

    //--- Setup joints
    this->joints.clear();
    for (const urdf::Joint& urdfJoint : urdfModel.joints) {
        //Ignore everything but hinge joints
        if (urdfJoint.type != urdf::Joint::REVOLUTE && urdfJoint.type != urdf::Joint::CONTINUOUS)
            continue;

        //Insert joint into list
        this->joints.insert({urdfJoint.name, robot::Joint(urdfJoint.parent_link_name, urdfJoint.child_link_name)});
        robot::Joint& robotJoint = this->joints.at(urdfJoint.name);

        //Set local coordinates
        robotJoint.pJPos = globalLinkPoses.at(urdfJoint.parent_link_name)->orientation * transform_to_trafo(urdfJoint.parent_to_joint_transform).position;

        //Set joint axis
        robotJoint.axis = globalLinkPoses.at(urdfJoint.child_link_name)->orientation * vector3_to_vector3d(urdfJoint.axis.value());

        //Set limits
        if (urdfJoint.limits.has_value()) {
            //Angle limits
            if (urdfJoint.limits->lower.has_value() && urdfJoint.limits->upper.has_value())
                robotJoint.angleLimits = {urdfJoint.limits->lower.value(), urdfJoint.limits->upper.value()};

            //Velocity limits
            if (urdfJoint.limits->velocity.has_value())
                robotJoint.velLimits = {-urdfJoint.limits->velocity.value(), urdfJoint.limits->velocity.value()};
        }
    }

    //--- Setup links (only the once that are part of the kinematic chain according to the hinge joints)
    this->links.clear();
    if (urdfModel.links.size() == 1)  //Free-floating base
        this->links.insert({urdfBaseLink.name, robot::Link()});
    for (const urdf::Link& urdfLink : urdfModel.links)
        for (const auto& [jointName, joint] : this->joints)
            if (urdfLink.name == joint.parentName || urdfLink.name == joint.childName)
                this->links.insert({urdfLink.name, robot::Link()});

    //--- Load models (convert all other joints to fixed joints, and load the model accordingly)
    //Extract path
    std::string modelPath(filePath);
    modelPath.erase(modelPath.find_last_of("/"));

    //Load visuals
    for (const urdf::Link& urdfLink : urdfModel.links) {
        //No visuals, move on to next link
        if (urdfLink.visuals.size() <= 0)
            continue;

        //Check if link exists in list
        if (this->links.find(urdfLink.name) != this->links.end()) {
            //Get info
            robot::Link& robotLink = this->links.at(urdfLink.name);
            const tools::Transformation& globalTrafo = globalLinkPoses.at(urdfLink.name).value();

            //Loop over all visuals
            for (const urdf::Link::Visual& urdfVisual : urdfLink.visuals) {
                //Check if geometry contains a value
                if (urdfVisual.geometry.has_value()) {
                    //Only consider meshes
                    auto mesh = [&](const urdf::Mesh& mesh) {
                        //Extract information
                        const std::string filePath = modelPath + mesh.filePath.substr(mesh.filePath.find("/meshes/"));
                        const tools::Transformation origin =
                            (urdfVisual.origin.has_value() ? transform_to_trafo(urdfVisual.origin.value()) : tools::Transformation());
                        const tools::Transformation localTrafo(origin.position, globalTrafo.inverse().getLocalCoordinates(origin.orientation));
                        const Eigen::Vector3d scale = vector3_to_vector3d(mesh.scale);
                        std::optional<Eigen::Vector3d> color = std::nullopt;
                        if (urdfVisual.material_name.has_value()) {
                            const urdf::Material& material = material_by_name(urdfVisual.material_name.value());
                            if (material.color.has_value())
                                color = Eigen::Vector3d(material.color->r, material.color->g, material.color->b);
                        }

                        //Add to list
                        robotLink.visuals.emplace_back(filePath, f_loadModel, localTrafo, scale, color);
                    };
                    auto others = [&](const auto& other) { LENNY_LOG_WARNING("Only mesh visuals are considered for now") };
                    std::visit(overloaded{mesh, others}, urdfVisual.geometry.value());
                }
            }
        } else {
            LENNY_LOG_WARNING("Visuals of URDF link `%s` are not loaded, since it is not part of the kinematic chain", urdfLink.name.c_str())
        }
    }

    //--- Setup base (corresponding link does not have a child)
    std::vector<std::string> baseCandidates;
    for (const auto& [linkName, link] : this->links) {
        bool hasAChild = false;
        for (const auto& [jointName, joint] : this->joints) {
            if (joint.childName == linkName) {
                hasAChild = true;
                break;
            }
        }
        if (!hasAChild)
            baseCandidates.push_back(linkName);
    }
    if (baseCandidates.size() != 1)
        LENNY_LOG_ERROR("There should be only one base candidate, but there are `%d`", baseCandidates.size());
    base = std::make_unique<Base>(baseCandidates.at(0));

    //--- Setup Link to Joints Map
    this->linkToJoints.clear();
    for (const auto& [linkName, link] : links) {
        this->linkToJoints.insert({linkName, {std::nullopt, {}}});

        for (const auto& [jointName, joint] : joints) {
            if (linkName == joint.childName)
                this->linkToJoints.at(linkName).first = jointName;
            else if (linkName == joint.parentName)
                this->linkToJoints.at(linkName).second.push_back(jointName);
        }
    }
}

void Robot::checkLinkName(const std::string& linkName) const {
    if (links.find(linkName) == links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not exist", linkName.c_str());
}

void Robot::checkJointName(const std::string& jointName) const {
    if (joints.find(jointName) == joints.end())
        LENNY_LOG_ERROR("Joint with name `%s` does not exist", jointName.c_str());
}

void Robot::checkState(const Eigen::VectorXd& state) const {
    if (state.size() != getStateSize())
        LENNY_LOG_ERROR("Invalid input state (size %d VS %d)", state.size(), getStateSize());
}

void Robot::checkDofMask(const Eigen::VectorXb& dofMask) const {
    if (dofMask.size() != getStateSize())
        LENNY_LOG_ERROR("Invalid input dof mask (size %d VS %d)", dofMask.size(), getStateSize());
}

uint Robot::getStateSize() const {
    return 6 + joints.size();
}

uint Robot::getStateIndex(const std::string& jointName) const {
    checkJointName(jointName);
    return std::distance(joints.begin(), joints.find(jointName)) + 6;
}

std::string Robot::getDescriptionForDofIndex(const uint& dofIndex) const {
    if (dofIndex >= getStateSize())
        LENNY_LOG_ERROR("Invalid dof index input")

    if (dofIndex < 6)
        return std::string(Base::dofNames[dofIndex]);
    auto it = joints.begin();  //+ robotDofIndex - 6;
    for (uint i = 0; i < dofIndex - 6; i++)
        it++;
    return it->first;
}

const Limits& Robot::getLimitsForDofIndex(const uint& dofIndex, const LIMITS_TYPE limitsType) const {
    if (dofIndex >= getStateSize())
        LENNY_LOG_ERROR("Invalid dof index input")

    if (dofIndex < 6) {  //Base
        if (limitsType == POSITION)
            return base->posLimitsList[dofIndex];
        else if (limitsType == VELOCITY)
            return base->velLimitsList[dofIndex];
    } else {  //Joints
        auto jIter = joints.begin();
        std::advance(jIter, dofIndex - 6);
        if (limitsType == POSITION)
            return jIter->second.angleLimits;
        else if (limitsType == VELOCITY)
            return jIter->second.velLimits;
    }
    return base->posLimitsList[0];  //Dummy
}

/*
 * COMMENT:-1: Not part of same kinematic chain / 0: same link
 */
int Robot::getNumberOfJointsInbetween(const std::string& linkName_A, const std::string& linkName_B) const {
    //Check inputs
    checkLinkName(linkName_A);
    checkLinkName(linkName_B);

    //Check if links are the same
    if (linkName_A == linkName_B)
        return 0;

    //Start from linkName_A and go FORWARD in the kinematic chain
    {
        bool linkFound = false;
        std::vector<int> numJointsInbetween = {0};
        auto goForward = [&](auto&& goForward, const std::string& linkName) -> void {
            const auto& jointChildNames = this->linkToJoints.at(linkName).second;
            const int currentNumJoints = numJointsInbetween.back();
            for (int i = 0; i < jointChildNames.size(); i++) {
                if (linkFound)
                    return;

                if (i > 0)
                    numJointsInbetween.push_back(currentNumJoints);

                numJointsInbetween.back()++;

                const Joint& joint = joints.at(jointChildNames[i]);
                if (joint.childName == linkName_B) {
                    linkFound = true;
                    return;
                }

                goForward(goForward, joint.childName);
            }
        };
        goForward(goForward, linkName_A);
        if (linkFound)
            return numJointsInbetween.back();
    }

    //Start from linkName_A and go BACKWARDS in the kinematic chain
    {
        bool linkFound = false;
        int numJointsInbetween = 0;
        auto goBackward = [&](auto&& goBackward, const std::string& linkName) -> void {
            if (linkFound)
                return;

            const auto& jointParentName = this->linkToJoints.at(linkName).first;
            if (jointParentName.has_value()) {
                numJointsInbetween++;

                const Joint& joint = joints.at(jointParentName.value());
                if (joint.parentName == linkName_B) {
                    linkFound = true;
                    return;
                }

                goBackward(goBackward, joint.parentName);
            }
        };
        goBackward(goBackward, linkName_A);
        if (linkFound)
            return numJointsInbetween;
    }

    return -1;
}

Eigen::Vector3d Robot::computeGlobalPoint(const Eigen::VectorXd& state, const Eigen::Vector3d& p_local, const std::string& linkName) const {
    checkState(state);
    checkLinkName(linkName);
    auto getGlobalCoordinates = [](const tools::Transformation& trafo, const Eigen::Vector3d& local) -> Eigen::Vector3d {
        return trafo.getGlobalCoordinatesForPoint(local);
    };
    return computeGlobal<Eigen::Vector3d>(state, p_local, linkName, getGlobalCoordinates);
}

void Robot::computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& p_local, const std::string& linkName,
                                 const Eigen::VectorXb& dofMask) const {
    checkState(state);
    checkLinkName(linkName);
    checkDofMask(dofMask);
    auto getGlobalCoordinates = [](const tools::Transformation& trafo, const Eigen::Vector3d& local) -> Eigen::Vector3d {
        return trafo.getGlobalCoordinatesForPoint(local);
    };
    computeJacobian(jacobian, state, p_local, linkName, dofMask, getGlobalCoordinates);
    if (dofMask[0])
        jacobian.block(0, 0, 3, 1) = Eigen::Vector3d::UnitX();
    if (dofMask[1])
        jacobian.block(0, 1, 3, 1) = Eigen::Vector3d::UnitY();
    if (dofMask[2])
        jacobian.block(0, 2, 3, 1) = Eigen::Vector3d::UnitZ();
}

void Robot::computePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& p_local, const std::string& linkName,
                               const Eigen::VectorXb& dofMask) const {
    checkState(state);
    checkLinkName(linkName);
    checkDofMask(dofMask);
    auto getGlobalCoordinates = [](const tools::Transformation& trafo, const Eigen::Vector3d& local) -> Eigen::Vector3d {
        return trafo.getGlobalCoordinatesForPoint(local);
    };
    computeTensor(tensor, state, p_local, linkName, dofMask, getGlobalCoordinates);
}

Eigen::Vector3d Robot::computeGlobalVector(const Eigen::VectorXd& state, const Eigen::Vector3d& v_local, const std::string& linkName) const {
    checkState(state);
    checkLinkName(linkName);
    auto getGlobalCoordinates = [](const tools::Transformation& trafo, const Eigen::Vector3d& local) -> Eigen::Vector3d {
        return trafo.getGlobalCoordinatesForVector(local);
    };
    return computeGlobal<Eigen::Vector3d>(state, v_local, linkName, getGlobalCoordinates);
}

void Robot::computeVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& v_local, const std::string& linkName,
                                  const Eigen::VectorXb& dofMask) const {
    checkState(state);
    checkLinkName(linkName);
    checkDofMask(dofMask);
    auto getGlobalCoordinates = [](const tools::Transformation& trafo, const Eigen::Vector3d& local) -> Eigen::Vector3d {
        return trafo.getGlobalCoordinatesForVector(local);
    };
    computeJacobian(jacobian, state, v_local, linkName, dofMask, getGlobalCoordinates);
}

void Robot::computeVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& v_local, const std::string& linkName,
                                const Eigen::VectorXb& dofMask) const {
    checkState(state);
    checkLinkName(linkName);
    checkDofMask(dofMask);
    auto getGlobalCoordinates = [](const tools::Transformation& trafo, const Eigen::Vector3d& local) -> Eigen::Vector3d {
        return trafo.getGlobalCoordinatesForVector(local);
    };
    computeTensor(tensor, state, v_local, linkName, dofMask, getGlobalCoordinates);
}

void Robot::testPointJacobian(const Eigen::VectorXd& state, const Eigen::Vector3d& p_local, const std::string& linkName) const {
    checkState(state);
    checkLinkName(linkName);
    auto eval = [&](Eigen::VectorXd& P, const Eigen::VectorXd& s) -> void { P = computeGlobalPoint(s, p_local, linkName); };
    auto anal = [&](Eigen::MatrixXd& dPdS, const Eigen::VectorXd& s) -> void {
        computePointJacobian(dPdS, s, p_local, linkName, Eigen::VectorXb::Ones(s.size()));
    };
    fd.testMatrix(eval, anal, state, "Point Jacobian", 3, true);
}

void Robot::testPointTensor(const Eigen::VectorXd& state, const Eigen::Vector3d& p_local, const std::string& linkName) const {
    checkState(state);
    checkLinkName(linkName);
    auto eval = [&](Eigen::MatrixXd& dPdS, const Eigen::VectorXd& s) -> void {
        computePointJacobian(dPdS, s, p_local, linkName, Eigen::VectorXb::Ones(s.size()));
    };
    auto anal = [&](Eigen::TensorD& d2PdS2, const Eigen::VectorXd& s) -> void {
        computePointTensor(d2PdS2, s, p_local, linkName, Eigen::VectorXb::Ones(s.size()));
    };
    fd.testTensor(eval, anal, state, "Point Tensor", 3, state.size());
}

void Robot::testVectorJacobian(const Eigen::VectorXd& state, const Eigen::Vector3d& v_local, const std::string& linkName) const {
    checkState(state);
    checkLinkName(linkName);
    auto eval = [&](Eigen::VectorXd& V, const Eigen::VectorXd& s) -> void { V = computeGlobalVector(s, v_local, linkName); };
    auto anal = [&](Eigen::MatrixXd& dVdS, const Eigen::VectorXd& s) -> void {
        computeVectorJacobian(dVdS, s, v_local, linkName, Eigen::VectorXb::Ones(s.size()));
    };
    fd.testMatrix(eval, anal, state, "Vector Jacobian", 3, true);
}

void Robot::testVectorTensor(const Eigen::VectorXd& state, const Eigen::Vector3d& v_local, const std::string& linkName) const {
    checkState(state);
    checkLinkName(linkName);
    auto eval = [&](Eigen::MatrixXd& dVdS, const Eigen::VectorXd& s) -> void {
        computeVectorJacobian(dVdS, s, v_local, linkName, Eigen::VectorXb::Ones(s.size()));
    };
    auto anal = [&](Eigen::TensorD& d2VdS2, const Eigen::VectorXd& s) -> void {
        computeVectorTensor(d2VdS2, s, v_local, linkName, Eigen::VectorXb::Ones(s.size()));
    };
    fd.testTensor(eval, anal, state, "Vector Tensor", 3, state.size());
}

Eigen::QuaternionD Robot::computeGlobalOrientation(const Eigen::VectorXd& state, const Eigen::QuaternionD& q_local, const std::string& linkName) const {
    checkState(state);
    checkLinkName(linkName);
    auto getGlobalCoordinates = [](const tools::Transformation& trafo, const Eigen::QuaternionD& local) -> Eigen::QuaternionD {
        return trafo.getGlobalCoordinates(local);
    };
    return computeGlobal<Eigen::QuaternionD>(state, q_local, linkName, getGlobalCoordinates);
}

tools::Transformation Robot::computeGlobalPose(const Eigen::VectorXd& state, const tools::Transformation& t_local, const std::string& linkName) const {
    checkState(state);
    checkLinkName(linkName);
    auto getGlobalCoordinates = [](const tools::Transformation& trafo, const tools::Transformation& local) -> tools::Transformation {
        return trafo.getGlobalCoordinates(local);
    };
    return computeGlobal<tools::Transformation>(state, t_local, linkName, getGlobalCoordinates);
}

Eigen::Vector3d Robot::computeLocalPoint(const Eigen::VectorXd& state, const Eigen::Vector3d& p_global, const std::string& linkName) const {
    return computeGlobalPose(state, tools::Transformation(), linkName).getLocalCoordinatesForPoint(p_global);
}

Eigen::Vector3d Robot::computeLocalVector(const Eigen::VectorXd& state, const Eigen::Vector3d& v_global, const std::string& linkName) const {
    return computeGlobalPose(state, tools::Transformation(), linkName).getLocalCoordinatesForVector(v_global);
}

Eigen::QuaternionD Robot::computeLocalOrientation(const Eigen::VectorXd& state, const Eigen::QuaternionD& q_global, const std::string& linkName) const {
    return computeGlobalPose(state, tools::Transformation(), linkName).getLocalCoordinates(q_global);
}

tools::Transformation Robot::computeLocalPose(const Eigen::VectorXd& state, const tools::Transformation& t_global, const std::string& linkName) const {
    return computeGlobalPose(state, tools::Transformation(), linkName).getLocalCoordinates(t_global);
}

void Robot::computeGlobalLinkPoses(LinkPoses& globalLinkPoses, const Eigen::VectorXd& state) const {
    //Test state
    checkState(state);

    //Clear
    globalLinkPoses.clear();

    //Add base transformation
    globalLinkPoses.insert({base->linkName, base->getTransformationFromState(state.segment(0, 6))});

    //Loop recursively over kinematic chain
    auto setPoses = [&](auto&& setPoses, const std::string& linkName) -> void {
        const auto& jointChildNames = this->linkToJoints.at(linkName).second;
        for (const std::string& jointChildName : jointChildNames) {
            const Joint& joint = joints.at(jointChildName);
            const uint stateIndex = getStateIndex(jointChildName);

            tools::Transformation trafo;
            trafo.position = globalLinkPoses.at(linkName).getGlobalCoordinatesForPoint(joint.pJPos);
            trafo.orientation = globalLinkPoses.at(linkName).orientation * tools::utils::getRotationQuaternion(state[stateIndex], joint.axis);

            globalLinkPoses.insert({joint.childName, trafo});

            setPoses(setPoses, joint.childName);
        }
    };
    setPoses(setPoses, base->linkName);
}

double Robot::estimateAngularVelocity(const double& currentAngle, const double& previousAngle, const double& dt) {
    double diff = currentAngle - previousAngle;
    while (diff > PI)
        diff -= PI;
    while (diff < -PI)
        diff += PI;
    if (diff >= PI || diff <= -PI)
        LENNY_LOG_ERROR("Something is still wrong with the difference: %lf - %lf = %lf", currentAngle, previousAngle, diff)
    return diff / dt;
};

Eigen::VectorXd Robot::estimateVelocity(const Eigen::VectorXd& currentState, const Eigen::VectorXd& previousState, const double& dt) const {
    //Perform checks
    checkState(currentState);
    checkState(previousState);
    if (dt < 1e-6)
        LENNY_LOG_ERROR("Invalid input for dt: '%lf'", dt)

    //Velocity estimation
    Eigen::VectorXd velocity(getStateSize());
    velocity.segment(0, 3) = (currentState.segment(0, 3) - previousState.segment(0, 3)) / dt;
    for (int i = 3; i < getStateSize(); i++)
        velocity[i] = estimateAngularVelocity(currentState[i], previousState[i], dt);
    return velocity;
}

double Robot::estimateAngularAcceleration(const double& currentAngle, const double& previousAngle, const double& oldAngle, const double& dt) {
    const double currentVelocity = estimateAngularVelocity(currentAngle, previousAngle, dt);
    const double previousVelocity = estimateAngularVelocity(previousAngle, oldAngle, dt);
    return (currentVelocity - previousVelocity) / dt;
}

Eigen::VectorXd Robot::estimateAcceleration(const Eigen::VectorXd& currentState, const Eigen::VectorXd& previousState, const Eigen::VectorXd& oldState,
                                            const double& dt) const {
    const Eigen::VectorXd currentVelocity = estimateVelocity(currentState, previousState, dt);
    const Eigen::VectorXd previousVelocity = estimateVelocity(previousState, oldState, dt);
    return (currentVelocity - previousVelocity) / dt;
}

void Robot::drawScene(const Eigen::VectorXd& state, const std::map<std::string, Eigen::VectorXd>& endEffectorStates) const {
    DRAWING_FLAGS flags = DRAWING_FLAGS::SHOW_NONE;
    if (showSkeleton)
        flags |= DRAWING_FLAGS::SHOW_SKELETON;
    if (showJointAxes)
        flags |= DRAWING_FLAGS::SHOW_JOINT_AXES;
    if (showJointLimits)
        flags |= DRAWING_FLAGS::SHOW_JOINT_LIMITS;
    if (showCoordinateFrames)
        flags |= DRAWING_FLAGS::SHOW_COORDINATE_FRAMES;
    if (showVisuals)
        flags |= DRAWING_FLAGS::SHOW_VISUALS;
    if (showGraspLocations)
        flags |= DRAWING_FLAGS::SHOW_GRASP_LOCATIONS;
    drawScene(state, endEffectorStates, flags, visualAlpha, infoAlpha);
}

void Robot::drawScene(const Eigen::VectorXd& state, const std::map<std::string, Eigen::VectorXd>& endEffectorStates, const DRAWING_FLAGS& flags,
                      const double& visualAlpha, const double& infoAlpha) const {
    LinkPoses globalLinkPoses;
    computeGlobalLinkPoses(globalLinkPoses, state);

    if (flags & DRAWING_FLAGS::SHOW_SKELETON) {
        Eigen::Vector4d linkColor, jointColor;
        linkColor << Link::skeletonColor, infoAlpha;
        jointColor << Joint::skeletonColor, infoAlpha;
        drawSkeleton(globalLinkPoses, skeletonRadius, linkColor, jointColor);
    }
    if (flags & DRAWING_FLAGS::SHOW_JOINT_AXES)
        drawJointAxes(globalLinkPoses);
    if (flags & DRAWING_FLAGS::SHOW_JOINT_LIMITS)
        drawJointLimits(state, globalLinkPoses);
    if (flags & DRAWING_FLAGS::SHOW_COORDINATE_FRAMES)
        drawCoordinateFrames(globalLinkPoses);
    if (flags & DRAWING_FLAGS::SHOW_VISUALS)
        drawVisuals(globalLinkPoses, endEffectorStates, std::nullopt, visualAlpha);
    if (flags & DRAWING_FLAGS::SHOW_GRASP_LOCATIONS)
        drawGraspLocations(globalLinkPoses);
}

void Robot::drawVisuals(const Eigen::VectorXd& state, const std::map<std::string, Eigen::VectorXd>& endEffectorStates,
                        const std::optional<Eigen::Vector3d>& color, const double& alpha) const {
    LinkPoses globalLinkPoses;
    computeGlobalLinkPoses(globalLinkPoses, state);
    drawVisuals(globalLinkPoses, endEffectorStates, color, alpha);
}

void Robot::drawSkeleton(const Eigen::VectorXd& state, const double& radius, const Eigen::Vector4d& linkColor, const Eigen::Vector4d& jointColor) const {
    LinkPoses globalLinkPoses;
    computeGlobalLinkPoses(globalLinkPoses, state);
    drawSkeleton(globalLinkPoses, radius, linkColor, jointColor);
}

void Robot::drawGui(const bool withDrawingOptions) {
    using tools::Gui;
    if (Gui::I->TreeNode(("Robot - `" + name + "`").c_str())) {
        base->drawGui();

        if (Gui::I->TreeNode("Links")) {
            for (auto& [name, link] : links)
                link.drawGui(name);
            Gui::I->TreePop();
        }

        if (Gui::I->TreeNode("Joints")) {
            for (auto& [name, joint] : joints)
                joint.drawGui(name);
            Gui::I->TreePop();
        }

        if (Gui::I->TreeNode("EndEffectors")) {
            for (auto& [name, endEffector] : endEffectors)
                endEffector.drawGui(name);
            Gui::I->TreePop();
        }

        if (withDrawingOptions) {
            if (Gui::I->TreeNode("Drawing")) {
                Gui::I->Checkbox("Show Skeleton", showSkeleton);
                Gui::I->Checkbox("Show Coordinate Frames", showCoordinateFrames);
                Gui::I->Checkbox("Show Joint Axes", showJointAxes);
                Gui::I->Checkbox("Show Joint Limits", showJointLimits);
                Gui::I->Checkbox("Show Visuals", showVisuals);
                Gui::I->Checkbox("Show Grasp Locations", showGraspLocations);

                Gui::I->Slider("Info Alpha", infoAlpha, 0.0, 1.0);
                Gui::I->Slider("Visual Alpha", visualAlpha, 0.0, 1.0);

                Gui::I->TreePop();
            }
        }

        Gui::I->TreePop();
    }
}

bool Robot::drawFKGui(Eigen::VectorXd& state, const char* label, const LIMITS_TYPE& limitsType) const {
    checkState(state);

    using tools::Gui;
    bool triggered = false;
    if (Gui::I->TreeNode(label)) {
        //--- Base
        for (uint i = 0; i < 6; i++) {
            Limits limits = std::nullopt;
            if (limitsType == POSITION)
                limits = base->posLimitsList[i];
            else if (limitsType == VELOCITY)
                limits = base->velLimitsList[i];

            const auto bounds = limits.has_value() ? limits.value() : std::pair<double, double>{-2.0 * PI, 2.0 * PI};
            if (Gui::I->Slider(std::string(Base::dofNames[i]).c_str(), state[i], bounds.first, bounds.second))
                triggered = true;
        }

        //--- Joints
        for (const auto& [jointName, joint] : joints) {
            Limits limits = std::nullopt;
            if (limitsType == POSITION)
                limits = joint.angleLimits;
            else if (limitsType == VELOCITY)
                limits = joint.velLimits;

            const auto bounds = limits.has_value() ? limits.value() : std::pair<double, double>{-2.0 * PI, 2.0 * PI};
            if (Gui::I->Slider(jointName.c_str(), state[getStateIndex(jointName)], bounds.first, bounds.second))
                triggered = true;
        }

        Gui::I->TreePop();
    }

    return triggered;
}

std::optional<std::pair<std::string, Eigen::Vector3d>> Robot::getFirstLinkHitByRay(const Eigen::VectorXd& state, const Ray& ray) const {
    double t = HUGE_VALF;
    auto getGlobalIntersectionPointForVisual = [&](const Visual& visual, const tools::Transformation& globalLinkPose) -> std::optional<Eigen::Vector3d> {
        std::optional<Eigen::Vector3d> globalIntersectionPoint = std::nullopt;
        const tools::Transformation visualPose = globalLinkPose * visual.localTrafo;
        const std::optional<tools::Model::HitInfo> hitInfo = visual.model->hitByRay(visualPose.position, visualPose.orientation, visual.scale, ray);
        if (hitInfo.has_value() && hitInfo->t < t) {
            t = hitInfo->t;
            globalIntersectionPoint = ray.origin + ray.direction * hitInfo->t;
        }
        return globalIntersectionPoint;
    };

    LinkPoses globalLinkPoses;
    computeGlobalLinkPoses(globalLinkPoses, state);
    std::optional<std::pair<std::string, Eigen::Vector3d>> info = std::nullopt;

    //Loop over links
    for (const auto& [linkName, link] : links) {
        const auto& globalLinkPose = globalLinkPoses.at(linkName);
        for (const auto& visual : link.visuals) {
            const auto linkIntersectionPoint = getGlobalIntersectionPointForVisual(visual, globalLinkPose);
            if (linkIntersectionPoint.has_value())
                info = {linkName, linkIntersectionPoint.value()};
        }
    }

    //Loop over endeffectors
    for (const auto& [endEffectorName, endEffector] : endEffectors) {
        const auto& globalLinkPose = globalLinkPoses.at(endEffector.linkName);
        for (const auto& visual : endEffector.visuals) {
            const auto linkIntersectionPoint = getGlobalIntersectionPointForVisual(visual, globalLinkPose);
            if (linkIntersectionPoint.has_value())
                info = {endEffector.linkName, linkIntersectionPoint.value()};
        }
    }
    return info;
}

struct Dof {
    std::string name;
    double value;

    static void to_json(json& j, const Dof& o) {
        TO_JSON(o, name)
        TO_JSON(o, value)
    }

    static void from_json(const json& j, Dof& o) {
        FROM_JSON(o, name)
        FROM_JSON(o, value)
    }
};

bool Robot::saveStateToFile(const Eigen::VectorXd& state, const std::string& filePath) const {
    //Test state
    checkState(state);

    //Check file extensions
    if (!tools::utils::checkFileExtension(filePath, "json")) {
        LENNY_LOG_WARNING("File `%s` should be a `json` file... Abort!", filePath.c_str())
        return false;
    }

    //Open file
    std::ofstream file(filePath);
    if (!file.is_open()) {
        LENNY_LOG_WARNING("File `%s` could not be opened... Abort!", filePath.c_str());
        return false;
    }

    //Setup json
    json js;

    //Robot
    {
        json js_tmp;
        js_tmp["name"] = name;
        js.push_back(js_tmp);
    }

    //Base
    for (int i = 0; i < 6; i++) {
        json js_tmp;
        Dof dof = {std::string(Base::dofNames[i]), state[i]};
        Dof::to_json(js_tmp, dof);
        js.push_back(js_tmp);
    }

    //Joints
    for (const auto& [jointName, joint] : joints) {
        json js_tmp;
        Dof dof = {jointName, state[getStateIndex(jointName)]};
        Dof::to_json(js_tmp, dof);
        js.push_back(js_tmp);
    }

    //Stream to file
    file << std::setw(2) << js << std::endl;

    //Close file
    file.close();

    //Wrap up
    LENNY_LOG_INFO("Successfully saved robot state into file `%s`", filePath.c_str());
    return true;
}

std::optional<Eigen::VectorXd> Robot::loadStateFromFile(const char* fP) const {
    //Initialize state
    std::optional<Eigen::VectorXd> state = std::nullopt;

    //Initialize file path
    const std::string filePath = fP ? std::string(fP) : tools::utils::browseFile();

    //Check file extensions
    if (!tools::utils::checkFileExtension(filePath, "json")) {
        LENNY_LOG_WARNING("File `%s` should be a `json` file... Abort!", filePath.c_str())
        return state;
    }

    //Open file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        LENNY_LOG_WARNING("File `%s` could not be opened... Abort!", filePath.c_str());
        return state;
    }

    //Load json from file
    json js;
    file >> js;

    //Load state
    state = Eigen::VectorXd::Zero(getStateSize());
    int iter = 0;
    for (auto& element : js) {
        if (iter == 0) {
            //Load robot name
            const std::string robotName = element.value("name", std::string());
            if (robotName != this->name)
                LENNY_LOG_ERROR("Trying to load state for wrong robot, namely `%s`", robotName.c_str())
        } else if (iter <= 6) {
            //Load root dofs
            Dof dof;
            Dof::from_json(element, dof);
            const auto dof_enum = magic_enum::enum_cast<Base::DOFS>(dof.name);
            const auto index = magic_enum::enum_integer(dof_enum.value());
            state.value()[index] = dof.value;
        } else {
            //Load joint dofs
            Dof dof;
            Dof::from_json(element, dof);
            state.value()[getStateIndex(dof.name)] = dof.value;
        }
        iter++;
    }

    //Close file
    file.close();

    //Wrap up
    LENNY_LOG_INFO("Robot state successfully loaded from file `%s`", filePath.c_str());
    return state;
}

template <typename T>
T Robot::computeGlobal(const Eigen::VectorXd& state, const T& local, const std::string& linkName,
                       const std::function<T(const tools::Transformation& trafo, const T& local)>& getGlobalCoordinates) const {
    //Initialize with local point
    T global(local);

    //Follow kinematic chain from the front to the back
    std::optional<std::string> jointParentName = this->linkToJoints.at(linkName).first;
    while (jointParentName.has_value()) {
        //Get transformation
        const Joint& joint = joints.at(jointParentName.value());
        const uint stateIndex = getStateIndex(jointParentName.value());
        const tools::Transformation trafo(joint.pJPos, tools::utils::getRotationQuaternion(state[stateIndex], joint.axis));

        //Update global
        global = getGlobalCoordinates(trafo, global);

        //Update jointParentName
        jointParentName = this->linkToJoints.at(joint.parentName).first;
    }

    //Transform with root pose
    global = getGlobalCoordinates(base->getTransformationFromState(state.segment(0, 6)), global);

    return global;
}

void Robot::computeJacobian(
    Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state, const Eigen::Vector3d& local, const std::string& linkName, const Eigen::VectorXb& dofMask,
    const std::function<Eigen::Vector3d(const tools::Transformation& trafo, const Eigen::Vector3d& local)>& getGlobalCoordinates) const {
    //Initialize jacobian
    jacobian.resize(3, state.size());
    jacobian.setZero();

    //Initialize base vector (used for base derivative computation)
    Eigen::Vector3d baseVector(local);

    //Follow kinematic chain from the front to the back
    std::optional<std::string> jointParentName = this->linkToJoints.at(linkName).first;
    while (jointParentName.has_value()) {
        //Get transformation
        const uint stateIndex = getStateIndex(jointParentName.value());
        const Joint& joint = joints.at(jointParentName.value());
        const tools::Transformation trafo(joint.pJPos, tools::utils::getRotationQuaternion(state[stateIndex], joint.axis));

        //Check dof mask
        if (dofMask[stateIndex]) {
            //Initialize current vector
            const Eigen::Vector3d currentVector = joint.axis.cross(trafo.getGlobalCoordinatesForVector(baseVector));

            //Transform current vector into global frame
            const Eigen::Vector3d vec = computeGlobalVector(state, currentVector, joint.parentName);

            //Add to jacobian block
            jacobian.block(0, stateIndex, 3, 1) = vec;
        }

        //Update baseVector
        baseVector = getGlobalCoordinates(trafo, baseVector);

        //Update jointParentName
        jointParentName = this->linkToJoints.at(joint.parentName).first;
    }

    Eigen::Matrix<double, 3, 6> baseJacobian;
    base->computeVectorJacobian(baseJacobian, state.segment(0, 6), baseVector);
    for (int i = 0; i < 6; i++)
        if (dofMask[i])
            jacobian.block(0, i, 3, 1) = baseJacobian.block(0, i, 3, 1);
}

void Robot::computeTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& state, const Eigen::Vector3d& local, const std::string& linkName,
                          const Eigen::VectorXb& dofMask,
                          const std::function<Eigen::Vector3d(const tools::Transformation& trafo, const Eigen::Vector3d& local)>& getGlobalCoordinates) const {
    //Prepare tensor
    tensor.resize(Eigen::Vector3i(3, state.size(), state.size()));
    tensor.setZero();

    //Initialize base vector (used for base derivative computation)
    Eigen::Vector3d outerBaseVector(local);

    //Start outer loop
    std::optional<std::string> outerJointParentName = this->linkToJoints.at(linkName).first;
    while (outerJointParentName.has_value()) {
        //Get outer infos
        const uint outerStateIndex = getStateIndex(outerJointParentName.value());
        const Joint& outerJoint = joints.at(outerJointParentName.value());
        const tools::Transformation outerTrafo(outerJoint.pJPos, tools::utils::getRotationQuaternion(state[outerStateIndex], outerJoint.axis));

        //Check dof mask from outer loop
        if (dofMask[outerStateIndex]) {
            //Initialize base vector
            Eigen::Vector3d innerBaseVector = outerJoint.axis.cross(outerTrafo.getGlobalCoordinatesForVector(outerBaseVector));

            //Compute current vector
            const Eigen::Vector3d currentVector = outerJoint.axis.cross(innerBaseVector);

            //Transform current vector into global frame
            const Eigen::Vector3d vec = computeGlobalVector(state, currentVector, outerJoint.parentName);

            //Add to tensor
            for (int i = 0; i < 3; i++)
                tensor.addEntry(Eigen::Vector3i(i, outerStateIndex, outerStateIndex), vec[i]);

            //Start inner loop
            std::optional<std::string> innerJointParentName = this->linkToJoints.at(outerJoint.parentName).first;
            while (innerJointParentName.has_value()) {
                //Get inner infos
                const uint innerStateIndex = getStateIndex(innerJointParentName.value());
                const Joint& innerJoint = joints.at(innerJointParentName.value());
                const tools::Transformation innerTrafo(innerJoint.pJPos, tools::utils::getRotationQuaternion(state[innerStateIndex], innerJoint.axis));

                //Check dof mask from inner loop
                if (dofMask[innerStateIndex]) {
                    //Compute current vector
                    const Eigen::Vector3d currentVector = innerJoint.axis.cross(innerTrafo.getGlobalCoordinatesForVector(innerBaseVector));

                    //Transform current vector into global frame
                    const Eigen::Vector3d vec = computeGlobalVector(state, currentVector, innerJoint.parentName);

                    //Add to tensor
                    for (int i = 0; i < 3; i++) {
                        tensor.addEntry(Eigen::Vector3i(i, outerStateIndex, innerStateIndex), vec[i]);
                        tensor.addEntry(Eigen::Vector3i(i, innerStateIndex, outerStateIndex), vec[i]);
                    }
                }

                //Update innerBaseVector
                innerBaseVector = innerTrafo.getGlobalCoordinatesForVector(innerBaseVector);

                //Update innerJointParentName
                innerJointParentName = this->linkToJoints.at(innerJoint.parentName).first;
            }

            //Base dof derivatives
            Eigen::Matrix<double, 3, 6> innerBaseJacobian;
            base->computeVectorJacobian(innerBaseJacobian, state.segment(0, 6), innerBaseVector);
            for (int k = 0; k < innerBaseJacobian.outerSize(); ++k) {
                for (Eigen::Matrix<double, 3, 6>::InnerIterator it(innerBaseJacobian, k); it; ++it) {
                    if (dofMask[it.col()]) {
                        tensor.addEntry(Eigen::Vector3i(it.row(), it.col(), outerStateIndex), it.value());
                        tensor.addEntry(Eigen::Vector3i(it.row(), outerStateIndex, it.col()), it.value());
                    }
                }
            }
        }

        //Update outerBaseVector
        outerBaseVector = getGlobalCoordinates(outerTrafo, outerBaseVector);

        //Update outerJointParentName
        outerJointParentName = this->linkToJoints.at(outerJoint.parentName).first;
    }

    //Base derivatives
    Eigen::TensorD outerBaseTensor;
    base->computeVectorTensor(outerBaseTensor, state.segment(0, 6), outerBaseVector);
    std::vector<std::pair<Eigen::Vector3i, double>> outerBaseTensorEntries;
    outerBaseTensor.getEntryList(outerBaseTensorEntries);
    for (const auto& [index, value] : outerBaseTensorEntries) {
        if (dofMask[index[1]] && dofMask[index[2]])
            tensor.addEntry(index, value);
    }
}

void Robot::drawSkeleton(const LinkPoses& globalLinkPoses, const double& radius, const Eigen::Vector4d& linkColor, const Eigen::Vector4d& jointColor) const {
    //Draw base
    tools::Renderer::I->drawPlane(globalLinkPoses.at(base->linkName).position, globalLinkPoses.at(base->linkName).orientation, 0.1 * Eigen::Vector2d::Ones(),
                                  linkColor);

    auto draw = [&](auto&& draw, const std::string& linkName) -> void {
        //Draw skeleton
        const auto& [jointParentName, jointChildNames] = this->linkToJoints.at(linkName);
        for (const std::string& jointChildName : jointChildNames) {
            const Joint& joint = joints.at(jointChildName);

            const Eigen::Vector3d parentPosition = globalLinkPoses.at(joint.parentName).position;
            const Eigen::Vector3d childPosition = globalLinkPoses.at(joint.childName).position;
            tools::Renderer::I->drawCylinder(parentPosition, childPosition, radius, linkColor);
            tools::Renderer::I->drawSphere(childPosition, 1.25 * radius, jointColor);

            draw(draw, joint.childName);
        }
    };
    draw(draw, base->linkName);
}

void Robot::drawCoordinateFrames(const LinkPoses& globalLinkPoses) const {
    for (const auto& [linkName, link] : this->links) {
        const tools::Transformation& trafo = globalLinkPoses.at(linkName);
        tools::Renderer::I->drawCoordinateSystem(trafo.position, trafo.orientation, 0.1, 0.01, infoAlpha);
    }
}

void Robot::drawJointAxes(const LinkPoses& globalLinkPoses) const {
    for (const auto& [jointName, joint] : this->joints) {
        const Eigen::Vector3d point = globalLinkPoses.at(joint.parentName).getGlobalCoordinatesForPoint(joint.pJPos);
        const Eigen::Vector3d axis = globalLinkPoses.at(joint.parentName).getGlobalCoordinatesForVector(joint.axis);
        tools::Renderer::I->drawArrow(point, 0.1 * axis, 0.01, Eigen::Vector4d(1.0, 0.0, 0.0, infoAlpha));
    }
}

void Robot::drawJointLimits(const Eigen::VectorXd& state, const LinkPoses& globalLinkPoses) const {
    using tools::Renderer;
    for (const auto& [jointName, joint] : joints) {
        if (!joint.angleLimits.has_value())
            continue;

        const tools::Transformation globalParentLinkPose = globalLinkPoses.at(joint.parentName);
        const Eigen::Vector3d normal = globalParentLinkPose.getGlobalCoordinatesForVector(joint.axis).normalized();
        const Eigen::Vector3d zeroAngleDir_local = joint.axis.cross(-joint.pJPos).normalized();
        const Eigen::Vector3d zeroAngleDir_global = globalParentLinkPose.getGlobalCoordinatesForVector(zeroAngleDir_local).normalized();
        const double currentAngle = state[getStateIndex(jointName)];
        const Eigen::Vector3d currentAngleDir = tools::utils::getRotationQuaternion(currentAngle, normal) * zeroAngleDir_global;

        Eigen::Matrix3d rotation;
        rotation.col(0) = normal.cross(zeroAngleDir_global);
        rotation.col(1) = normal;
        rotation.col(2) = zeroAngleDir_global;

        const Eigen::Vector3d globalLinkPosition = globalLinkPoses.at(joint.childName).position;
        const double sectorRadius = 10.0 * skeletonRadius;
        const auto& [lowerLimit, upperLimit] = joint.angleLimits.value();

        std::pair<double, double> drawRange = {lowerLimit, upperLimit};
        Eigen::Vector4d badRangeColor = Eigen::Vector4d(0.75, 0.0, 0.0, infoAlpha);
        if (upperLimit - lowerLimit > 2 * PI) {
            badRangeColor = Eigen::Vector4d(0.75, 0.75, 0.0, infoAlpha);
            if (currentAngle < 0.5 * (lowerLimit + upperLimit))
                drawRange = {lowerLimit, 0.5 * (lowerLimit + upperLimit)};
            else
                drawRange = {0.5 * (lowerLimit + upperLimit), upperLimit};
        }
        const std::pair<float, float> goodRange = {(float)(drawRange.first + 2.0 * PI + PI / 180.0), (float)(drawRange.second + 2.0 * PI + PI / 180.0)};
        const std::pair<float, float> badRange = {(float)(drawRange.second + PI / 180.0), (float)(drawRange.first + 2.0 * PI + PI / 180.0)};

        //Draw sectors
        Renderer::I->drawSector(globalLinkPosition, Eigen::QuaternionD(rotation), sectorRadius, goodRange, Eigen::Vector4d(0.0, 0.75, 0.0, infoAlpha));
        Renderer::I->drawSector(globalLinkPosition, Eigen::QuaternionD(rotation), sectorRadius, badRange, badRangeColor);

        //Current joint angle
        Renderer::I->drawArrow(globalLinkPosition, currentAngleDir * sectorRadius, 0.005, Eigen::Vector4d(0.0, 0.0, 0.75, infoAlpha));

        //Zero joint angle
        Renderer::I->drawArrow(globalLinkPosition, zeroAngleDir_global * sectorRadius, 0.005, Eigen::Vector4d(0.5, 0.5, 0.5, infoAlpha));
    }
}

void Robot::drawVisuals(const LinkPoses& globalLinkPoses, const std::map<std::string, Eigen::VectorXd>& endEffectorStates,
                        const std::optional<Eigen::Vector3d>& color, const double& alpha) const {
    for (const auto& [linkName, link] : this->links) {
        for (const auto& visual : link.visuals) {
            const std::optional<Eigen::Vector3d> col = color.has_value() ? color : visual.color;
            visual.drawScene(globalLinkPoses.at(linkName), col, alpha);
        }
    }

    for (const auto& [eeName, ee] : endEffectors) {
        const Eigen::VectorXd eeState =
            (endEffectorStates.find(eeName) != endEffectorStates.end()) ? endEffectorStates.at(eeName) : Eigen::VectorXd::Zero(ee.stateSize);
        checkLinkName(ee.linkName);
        ee.drawScene(eeState, globalLinkPoses.at(ee.linkName), color, alpha);
    }
}

void Robot::drawGraspLocations(const LinkPoses& globalLinkPoses) const {
    for (const auto& [eeName, ee] : endEffectors) {
        checkLinkName(ee.linkName);
        ee.drawGraspLocation(globalLinkPoses.at(ee.linkName));
    }
}

}  // namespace lenny::robot
