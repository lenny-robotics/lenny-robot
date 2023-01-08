#include <lenny/collision/Primitives.h>
#include <lenny/rapt/Agent.h>
#include <lenny/rapt/AgentCollisionParent.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Trajectory.h>

#include <fstream>

namespace lenny::rapt {

tools::FiniteDifference Agent::fd = tools::FiniteDifference("Agent");

Agent::Agent(const std::string& name, const robot::Robot& robot, const Eigen::VectorXd& initialRobotState, const Eigen::VectorXb& dofMask)
    : name(name), robot(robot) {
    setDofMask(dofMask);
    setInitialRobotStateFromRobotState(initialRobotState);
    initialRobotVelocity = Eigen::VectorXd::Zero(robot.getStateSize());
    generateSelfCollisionLinkMap(1);
    for (const auto& [eeName, ee] : robot.endEffectors) {
        if (ee->stateSize == 1)
            grippers.insert({eeName, Gripper(ee)});
        else
            LENNY_LOG_WARNING("EndEffector `%s` cannot become a gripper...", eeName.c_str())
    }
}

uint Agent::getStateSize() const {
    uint size = 0;
    for (int i = 0; i < dofMask.size(); i++)
        size += (uint)dofMask[i];
    return size;
}

void Agent::checkState(const Eigen::VectorXd& agentState) const {
    if (agentState.size() != getStateSize())
        LENNY_LOG_ERROR("Incorrect input state: should be size %d, but instead is size %d", getStateSize(), agentState.size())
}

Eigen::VectorXd Agent::getAgentStateFromRobotState(const Eigen::VectorXd& robotState) const {
    robot.checkState(robotState);

    Eigen::VectorXd agentState(getStateSize());
    int agentIter = 0;
    for (int i = 0; i < robotState.size(); i++) {
        if (dofMask[i])
            agentState[agentIter++] = robotState[i];
    }
    return agentState;
}

Eigen::VectorXd Agent::getRobotStateFromAgentState(const Eigen::VectorXd& agentState) const {
    checkState(agentState);

    Eigen::VectorXd robotState(robot.getStateSize());
    int agentIter = 0;
    for (int i = 0; i < robotState.size(); i++)
        robotState[i] = dofMask[i] ? agentState[agentIter++] : initialRobotState[i];
    return robotState;
}

Eigen::VectorXd Agent::getInitialAgentState() const {
    return getAgentStateFromRobotState(initialRobotState);
}

Eigen::VectorXd Agent::getInitialRobotState() const {
    return initialRobotState;
}

void Agent::setInitialRobotStateFromRobotState(const Eigen::VectorXd& robotState) {
    robot.checkState(robotState);
    initialRobotState = robotState;
}

void Agent::setInitialRobotStateFromAgentState(const Eigen::VectorXd& agentState) {
    initialRobotState = getRobotStateFromAgentState(agentState);
}

Eigen::VectorXd Agent::getInitialAgentVelocity() const {
    Eigen::VectorXd agentVelocity(getStateSize());
    int agentIter = 0;
    for (int i = 0; i < initialRobotVelocity.size(); i++) {
        if (dofMask[i])
            agentVelocity[agentIter++] = initialRobotVelocity[i];
    }
    return agentVelocity;
}

Eigen::VectorXd Agent::getInitialRobotVelocity() const {
    return initialRobotVelocity;
}

void Agent::setInitialRobotVelocityFromRobotVelocity(const Eigen::VectorXd& robotVelocity) {
    robot.checkState(robotVelocity);
    initialRobotVelocity = robotVelocity;
}

void Agent::setInitialRobotVelocityFromAgentVelocity(const Eigen::VectorXd& agentVelocity) {
    checkState(agentVelocity);
    int agentIter = 0;
    for (int i = 0; i < initialRobotVelocity.size(); i++)
        initialRobotVelocity[i] = dofMask[i] ? agentVelocity[agentIter++] : 0.0;
}

void Agent::setDofMask(const Eigen::VectorXb& dofMask) {
    robot.checkDofMask(dofMask);
    this->dofMask = dofMask;
}

const Eigen::VectorXb& Agent::getDofMask() const {
    return dofMask;
}

uint Agent::getRobotDofIndexFromAgentDofIndex(const uint& agentDofIndex) const {
    //Perform checks
    if (agentDofIndex >= getStateSize())
        LENNY_LOG_ERROR("Invalid input `agentDofIndex`: %d VS %d", agentDofIndex, getStateSize())

    //Get robotDofIndex from agentDofIndex
    int robotDofIndex = -1;
    int testDofIndex = -1;
    for (int i = 0; i < robot.getStateSize(); i++) {
        if (dofMask[i])
            testDofIndex++;
        if (testDofIndex == agentDofIndex) {
            robotDofIndex = i;
            break;
        }
    }
    if (robotDofIndex < 0 || robotDofIndex >= robot.getStateSize())
        LENNY_LOG_ERROR("Something is wrong... Invalid robot dof index!")

    return robotDofIndex;
}

const robot::Limits& Agent::getLimitsForDofIndex(const uint& agentDofIndex, const robot::Robot::LIMITS_TYPE limitsType) const {
    const uint robotDofIndex = getRobotDofIndexFromAgentDofIndex(agentDofIndex);
    return robot.getLimitsForDofIndex(robotDofIndex, limitsType);
}

bool Agent::isPositionalDof(const uint& agentDofIndex) const {
    if (agentDofIndex >= getStateSize())
        LENNY_LOG_ERROR("Invalid input `agentDofIndex`")

    if (agentDofIndex >= 3)
        return false;

    uint testDofIndex = 0;
    for (uint i = 0; i < 3; i++) {
        if (dofMask[i])
            testDofIndex++;
    }
    if (testDofIndex > agentDofIndex)
        return true;
    return false;
}

std::string Agent::getDescriptionForDofIndex(const uint& agentDofIndex) const {
    const uint robotDofIndex = getRobotDofIndexFromAgentDofIndex(agentDofIndex);
    return robot.getDescriptionForDofIndex(robotDofIndex);
}

Eigen::Vector3d Agent::computeGlobalPoint(const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_local, const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    return robot.computeGlobalPoint(robotState, p_local, linkName);
}

void Agent::computePointJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_local,
                                 const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    Eigen::MatrixXd robotJacobian(3, robot.getStateSize());
    robot.computePointJacobian(robotJacobian, robotState, p_local, linkName, dofMask);
    convertRobotJacobianToAgentJacobian(jacobian, robotJacobian);
}

void Agent::computePointTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_local, const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    Eigen::TensorD robotTensor(Eigen::Vector3i(3, robot.getStateSize(), robot.getStateSize()));
    robot.computePointTensor(robotTensor, robotState, p_local, linkName, dofMask);
    convertRobotTensorToAgentTensor(tensor, robotTensor);
}

Eigen::Vector3d Agent::computeGlobalVector(const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_local, const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    return robot.computeGlobalVector(robotState, v_local, linkName);
}

void Agent::computeVectorJacobian(Eigen::MatrixXd& jacobian, const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_local,
                                  const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    Eigen::MatrixXd robotJacobian(3, robot.getStateSize());
    robot.computeVectorJacobian(robotJacobian, robotState, v_local, linkName, dofMask);
    convertRobotJacobianToAgentJacobian(jacobian, robotJacobian);
}

void Agent::computeVectorTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_local, const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    Eigen::TensorD robotTensor(Eigen::Vector3i(3, robot.getStateSize(), robot.getStateSize()));
    robot.computeVectorTensor(robotTensor, robotState, v_local, linkName, dofMask);
    convertRobotTensorToAgentTensor(tensor, robotTensor);
}

void Agent::testPointJacobian(const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_local, const std::string& linkName) const {
    auto eval = [&](Eigen::VectorXd& P, const Eigen::VectorXd& s) -> void { P = computeGlobalPoint(s, p_local, linkName); };
    auto anal = [&](Eigen::MatrixXd& dPdS, const Eigen::VectorXd& s) -> void { computePointJacobian(dPdS, s, p_local, linkName); };
    fd.testMatrix(eval, anal, agentState, "Point Jacobian", 3, true);
}

void Agent::testPointTensor(const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_local, const std::string& linkName) const {
    auto eval = [&](Eigen::MatrixXd& dPdS, const Eigen::VectorXd& s) -> void { computePointJacobian(dPdS, s, p_local, linkName); };
    auto anal = [&](Eigen::TensorD& d2PdS2, const Eigen::VectorXd& s) -> void { computePointTensor(d2PdS2, s, p_local, linkName); };
    fd.testTensor(eval, anal, agentState, "Point Tensor", 3, agentState.size());
}

void Agent::testVectorJacobian(const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_local, const std::string& linkName) const {
    auto eval = [&](Eigen::VectorXd& V, const Eigen::VectorXd& s) -> void { V = computeGlobalVector(s, v_local, linkName); };
    auto anal = [&](Eigen::MatrixXd& dVdS, const Eigen::VectorXd& s) -> void { computeVectorJacobian(dVdS, s, v_local, linkName); };
    fd.testMatrix(eval, anal, agentState, "Vector Jacobian", 3, true);
}

void Agent::testVectorTensor(const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_local, const std::string& linkName) const {
    auto eval = [&](Eigen::MatrixXd& dVdS, const Eigen::VectorXd& s) -> void { computeVectorJacobian(dVdS, s, v_local, linkName); };
    auto anal = [&](Eigen::TensorD& d2VdS2, const Eigen::VectorXd& s) -> void { computeVectorTensor(d2VdS2, s, v_local, linkName); };
    fd.testTensor(eval, anal, agentState, "Vector Tensor", 3, agentState.size());
}

Eigen::QuaternionD Agent::computeGlobalOrientation(const Eigen::VectorXd& agentState, const Eigen::QuaternionD& q_local, const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    return robot.computeGlobalOrientation(robotState, q_local, linkName);
}

tools::Transformation Agent::computeGlobalPose(const Eigen::VectorXd& agentState, const tools::Transformation& t_local, const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    return robot.computeGlobalPose(robotState, t_local, linkName);
}

Eigen::Vector3d Agent::computeLocalPoint(const Eigen::VectorXd& agentState, const Eigen::Vector3d& p_global, const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    return robot.computeLocalPoint(robotState, p_global, linkName);
}

Eigen::Vector3d Agent::computeLocalVector(const Eigen::VectorXd& agentState, const Eigen::Vector3d& v_global, const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    return robot.computeLocalVector(robotState, v_global, linkName);
}

Eigen::QuaternionD Agent::computeLocalOrientation(const Eigen::VectorXd& agentState, const Eigen::QuaternionD& q_global, const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    return robot.computeLocalOrientation(robotState, q_global, linkName);
}

tools::Transformation Agent::computeLocalPose(const Eigen::VectorXd& agentState, const tools::Transformation& t_global, const std::string& linkName) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    return robot.computeLocalPose(robotState, t_global, linkName);
}

void Agent::MotionTrajectory::check(const uint& agentStateSize) const {
    if (data.size() != numSteps * agentStateSize)
        LENNY_LOG_ERROR("Invalid motion trajectory (%d VS %d)", data.size(), numSteps * agentStateSize)
}

double Agent::MotionTrajectory::getTotalTime() const {
    return (double)numSteps * deltaT;
}

int Agent::MotionTrajectory::getIndexForTime(const double& time) const {
    int index = (int)std::round((time / getTotalTime()) * (double)numSteps) - 1;
    tools::utils::boundToRange(index, -1, (int)numSteps - 1);
    return index;
}

double Agent::MotionTrajectory::getTimeForIndex(const int& index) const {
    return (double)index * deltaT;
}

Eigen::VectorXd Agent::getAgentStateForTrajectoryIndex(const MotionTrajectory& trajectory, const int& index) const {
    trajectory.check(getStateSize());
    if (index < -1)
        return getInitialAgentState() - trajectory.deltaT * getInitialAgentVelocity();
    else if (index == -1)
        return getInitialAgentState();
    else if (index > trajectory.numSteps - 1)
        return trajectory.data.segment((trajectory.numSteps - 1) * getStateSize(), getStateSize());
    return trajectory.data.segment(index * getStateSize(), getStateSize());
}

Eigen::VectorXd Agent::getAgentStateForTrajectoryTime(const MotionTrajectory& trajectory, const double& time) const {
    trajectory.check(getStateSize());
    const double totalTime = trajectory.getTotalTime();
    if (time <= 0.0)
        return getAgentStateForTrajectoryIndex(trajectory, -1);
    else if (time >= totalTime)
        return getAgentStateForTrajectoryIndex(trajectory, (int)trajectory.numSteps - 1);

    int lowerIndex = (int)std::floor((time / totalTime) * (double)trajectory.numSteps);
    tools::utils::boundToRange(lowerIndex, -1, (int)trajectory.numSteps - 1);
    int upperIndex = (int)std::ceil((time / totalTime) * (double)trajectory.numSteps);
    tools::utils::boundToRange(upperIndex, -1, (int)trajectory.numSteps - 1);

    tools::TrajectoryXd genericTrajectory;
    for (int i = lowerIndex; i <= upperIndex; i++)
        genericTrajectory.addEntry((double)i * trajectory.deltaT, getAgentStateForTrajectoryIndex(trajectory, i));
    return genericTrajectory.getLinearInterpolation(time);
}

void Agent::drawScene(const MotionTrajectory& trajectory, const double& currentTime, const bool& isRecedingHorizon) const {
    drawScene(getAgentStateForTrajectoryTime(trajectory, currentTime));
}

void Agent::drawScene(const Eigen::VectorXd& agentState) const {
    checkState(agentState);
    drawRobot(agentState);
    if (showCollisionPrimitives)
        drawCollisionPrimitives(agentState);
}

void Agent::drawGui(const bool withDrawingOptions) {
    using tools::Gui;
    if (Gui::I->TreeNode(std::string("Agent - `" + name + "`").c_str())) {
        robot.drawFKGui(initialRobotState, "Initial Robot State");

        if (Gui::I->TreeNode("Grippers")) {
            for (auto& [gripperName, gripper] : grippers)
                gripper.drawGui(gripperName);
            Gui::I->TreePop();
        }

        if (Gui::I->TreeNode("Collision")) {
            if (Gui::I->TreeNode("Primitives")) {
                if (Gui::I->TreeNode("List")) {
                    int iter = 0;
                    for (auto& [linkName, primitives] : collisionPrimitives)
                        for (auto& primitive : primitives)
                            primitive->drawGui(linkName + " - " + std::to_string(iter++));

                    Gui::I->TreePop();
                }

                if (Gui::I->TreeNode("Save & Load")) {
                    if (Gui::I->Button("Save To File"))
                        saveCollisionPrimitivesToFile(LENNY_PROJECT_FOLDER "/logs/CollisionPrimitives-" + name + "-" + tools::utils::getCurrentDateAndTime() +
                                                      ".json");

                    if (Gui::I->Button("Load From File"))
                        loadCollisionPrimitivesFromFile(nullptr);

                    Gui::I->TreePop();
                }

                Gui::I->TreePop();
            }

            if (Gui::I->TreeNode("Link Map (Self-Collision)")) {
                if (Gui::I->TreeNode("Generate")) {
                    static uint ignoreConsecutiveLinksIndex = 1;
                    Gui::I->Slider("Ignore Consecutive Link Index", ignoreConsecutiveLinksIndex, 0, 10);
                    if (Gui::I->Button("Generate Map"))
                        generateSelfCollisionLinkMap(ignoreConsecutiveLinksIndex);

                    Gui::I->TreePop();
                }

                if (Gui::I->TreeNode("Map")) {
                    for (const auto& [linkName, linkNameList] : selfCollisionLinkMap) {
                        if (Gui::I->TreeNode(linkName.c_str())) {
                            for (const auto& entry : linkNameList)
                                Gui::I->Text("--> %s", entry.c_str());
                            Gui::I->TreePop();
                        }
                    }

                    Gui::I->TreePop();
                }

                if (Gui::I->TreeNode("Save & Load")) {
                    if (Gui::I->Button("Save To File"))
                        saveSelfCollisionLinkMapToFile(LENNY_PROJECT_FOLDER "/logs/SelfCollisionLinkMap-" + name + "-" + tools::utils::getCurrentDateAndTime() +
                                                       ".json");

                    if (Gui::I->Button("Load From File"))
                        loadSelfCollisionLinkMapFromFile(nullptr);

                    Gui::I->TreePop();
                }

                Gui::I->TreePop();
            }

            Gui::I->TreePop();
        }

        if (Gui::I->TreeNode("Settings")) {
            Gui::I->Input("Local Base Trafo", localBaseTrafo);

            Gui::I->TreePop();
        }

        if (withDrawingOptions && Gui::I->TreeNode("Drawing")) {
            Gui::I->Checkbox("Show Skeleton", showSkeleton);
            Gui::I->Checkbox("Show Coordinate Frames", showCoordinateFrames);
            Gui::I->Checkbox("Show Joint Axes", showJointAxes);
            Gui::I->Checkbox("Show Joint Limits", showJointLimits);
            Gui::I->Checkbox("Show Visuals", showVisuals);
            Gui::I->Checkbox("Show Grasp Locations", showGraspLocations);
            Gui::I->Checkbox("Show Collision Primitives", showCollisionPrimitives);

            Gui::I->Slider("Info Alpha", infoAlpha, 0.0, 1.0);
            Gui::I->Slider("Visual Alpha", visualAlpha, 0.0, 1.0);

            Gui::I->TreePop();
        }

        drawAdditionalGuiContent();

        Gui::I->TreePop();
    }
}

inline void insertToList(const std::string& linkName, std::unordered_map<std::string, std::vector<collision::Primitive::SPtr>>& collisionPrimitives) {
    if (collisionPrimitives.find(linkName) == collisionPrimitives.end())
        collisionPrimitives.insert({linkName, {}});
}

void Agent::addCollisionSphere(const std::string& linkName, const Eigen::Vector3d& localPosition, const double& radius) {
    insertToList(linkName, collisionPrimitives);
    collisionPrimitives.at(linkName).emplace_back(
        std::make_shared<collision::Sphere>(std::make_shared<AgentCollisionParent>(*this, linkName), localPosition, radius));
}

void Agent::addCollisionCapsule(const std::string& linkName, const Eigen::Vector3d& localStartPosition, const Eigen::Vector3d& localEndPosition,
                                const double& radius) {
    insertToList(linkName, collisionPrimitives);
    collisionPrimitives.at(linkName).emplace_back(
        std::make_shared<collision::Capsule>(std::make_shared<AgentCollisionParent>(*this, linkName), localStartPosition, localEndPosition, radius));
}

void Agent::addCollisionRectangle(const std::string& linkName, const Eigen::Vector3d& localCenterPoint, const Eigen::QuaternionD& localOrientation,
                                  const Eigen::Vector2d& localDimensions, const double& safetyMargin) {
    insertToList(linkName, collisionPrimitives);
    collisionPrimitives.at(linkName).emplace_back(std::make_shared<collision::Rectangle>(std::make_shared<AgentCollisionParent>(*this, linkName),
                                                                                         localCenterPoint, localOrientation, localDimensions, safetyMargin));
}

void Agent::addCollisionBox(const std::string& linkName, const Eigen::Vector3d& localCenterPoint, const Eigen::QuaternionD& localOrientation,
                            const Eigen::Vector3d& localDimensions, const double& safetyMargin) {
    insertToList(linkName, collisionPrimitives);
    collisionPrimitives.at(linkName).emplace_back(std::make_shared<collision::Box>(std::make_shared<AgentCollisionParent>(*this, linkName), localCenterPoint,
                                                                                   localOrientation, localDimensions, safetyMargin));
}

bool Agent::saveCollisionPrimitivesToFile(const std::string& filePath) const {
    std::vector<collision::Primitive::SPtr> primitives;
    for (const auto& [linkName, prims] : collisionPrimitives)
        primitives.insert(primitives.end(), prims.begin(), prims.end());
    return collision::savePrimitivesToFile(primitives, filePath);
}

bool Agent::loadCollisionPrimitivesFromFile(const char* filePath) {
    const collision::F_getParent f_getParent = [&](const std::string& linkName) -> const collision::Parent::SPtr {
        return std::make_shared<AgentCollisionParent>(*this, linkName);
    };
    std::vector<collision::Primitive::SPtr> primitives;
    const bool successful = collision::loadPrimitivesFromFile(primitives, f_getParent, filePath);

    collisionPrimitives.clear();
    for (const auto& prim : primitives) {
        const std::string linkName = prim->parent->description;
        if (collisionPrimitives.find(linkName) == collisionPrimitives.end())
            collisionPrimitives.insert({linkName, {}});
        collisionPrimitives.at(linkName).emplace_back(prim);
    }
    return successful;
}

void Agent::generateSelfCollisionLinkMap(const uint& ignoreConsecutiveLinksIndex) {
    selfCollisionLinkMap.clear();
    for (auto it_A = robot.links.begin(); it_A != robot.links.end(); it_A++) {
        selfCollisionLinkMap.insert({it_A->first, {}});
        for (auto it_B = it_A; it_B != robot.links.end(); it_B++) {
            const int numJointsInbetween = robot.getNumberOfJointsInbetween(it_A->first, it_B->first);
            if (numJointsInbetween != 0 && (numJointsInbetween < 0 || numJointsInbetween > (int)ignoreConsecutiveLinksIndex))
                selfCollisionLinkMap.at(it_A->first).emplace_back(it_B->first);
        }
    }
}

bool Agent::saveSelfCollisionLinkMapToFile(const std::string& filePath) const {
    //Check file extensions
    if (!tools::utils::checkFileExtension(filePath, "json"))
        LENNY_LOG_ERROR("File `%s` should be a `json` file", filePath.c_str())

    //Open file
    std::ofstream file(filePath);
    if (!file.is_open()) {
        LENNY_LOG_WARNING("File `%s` could not be opened\n", filePath.c_str());
        return false;
    }

    //To json
    json js;
    for (const auto& [linkName, linkNameList] : selfCollisionLinkMap) {
        json js_tmp;
        js_tmp["link"] = linkName;
        js_tmp["links"] = linkNameList;
        js.push_back(js_tmp);
    }

    //Stream to file
    file << std::setw(2) << js << std::endl;

    //Close file
    file.close();

    //Wrap up
    LENNY_LOG_INFO("Successfully saved self collision link map into file `%s`", filePath.c_str());
    return true;
}

bool Agent::loadSelfCollisionLinkMapFromFile(const char* fP) {
    //Initialize file path
    const std::string filePath = fP ? std::string(fP) : tools::utils::browseFile();

    //Check file extensions
    if (!tools::utils::checkFileExtension(filePath, "json"))
        LENNY_LOG_ERROR("File `%s` should be a `json` file", filePath.c_str())

    //Open file
    std::ifstream file(filePath);
    if (!file.is_open())
        LENNY_LOG_ERROR("File `%s` could not be opened\n", filePath.c_str());

    //Load json from file
    json js;
    file >> js;

    //From json
    selfCollisionLinkMap.clear();
    for (const auto& js_tmp : js) {
        //Read entries
        const std::string link = js_tmp.value("link", std::string());
        const std::vector<std::string> links = js_tmp.value("links", std::vector<std::string>());

        //Check entries
        robot.checkLinkName(link);
        for (const auto& ln : links)
            robot.checkLinkName(ln);

        //Add to map
        selfCollisionLinkMap.insert({link, links});
    }

    //Close file
    file.close();

    //Wrap up
    LENNY_LOG_INFO("Self collision link map successfully loaded from file `%s`", filePath.c_str());
    return true;
}

void Agent::convertRobotJacobianToAgentJacobian(Eigen::MatrixXd& agentJacobian, const Eigen::MatrixXd& robotJacobian) const {
    agentJacobian.resize(3, getStateSize());
    int agentIter = 0;
    for (int i = 0; i < robotJacobian.cols(); i++) {
        if (dofMask[i])
            agentJacobian.col(agentIter++) = robotJacobian.col(i);
    }
}

void Agent::convertRobotTensorToAgentTensor(Eigen::TensorD& agentTensor, const Eigen::TensorD& robotTensor) const {
    Eigen::VectorXi rtaim(robot.getStateSize());  //robot to agent indices map
    int agentIter = 0;
    for (int i = 0; i < robot.getStateSize(); i++)
        rtaim[i] = dofMask[i] ? agentIter++ : -1;

    std::vector<std::pair<Eigen::Vector3i, double>> robotTensorEntries;
    robotTensor.getEntryList(robotTensorEntries);

    agentTensor.resize(Eigen::Vector3i(3, getStateSize(), getStateSize()));
    agentTensor.setZero();
    for (const auto& [index, value] : robotTensorEntries) {
        const Eigen::Vector3i newIndex(index[0], rtaim[index[1]], rtaim[index[2]]);
        if (newIndex[1] >= 0 && newIndex[2] >= 0)
            agentTensor.addEntry(newIndex, value);
    }
}

void Agent::drawRobot(const Eigen::VectorXd& agentState) const {
    const Eigen::VectorXd robotState = getRobotStateFromAgentState(agentState);
    using DRAWING_FLAGS = robot::Robot::DRAWING_FLAGS;
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

    //Collect end-effector states from grippers
    std::map<std::string, Eigen::VectorXd> endEffectorStates;
    for (const auto& [gripperName, gripper] : grippers) {
        gripper.updateFingerPercentage();
        Eigen::VectorXd state(1);
        state << gripper.getCurrentFingerPercentage();
        endEffectorStates.insert({gripperName, state});
    }

    robot.drawScene(robotState, endEffectorStates, flags, visualAlpha, infoAlpha);
}

void Agent::drawCollisionPrimitives(const Eigen::VectorXd& agentState) const {
    for (const auto& [linkName, primitives] : collisionPrimitives)
        for (const auto& primitive : primitives)
            primitive->drawScene(agentState, Eigen::Vector4d(0.75, 0.0, 0.0, 0.5));
}

}  // namespace lenny::rapt