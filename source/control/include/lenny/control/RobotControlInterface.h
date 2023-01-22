#pragma once

#include <lenny/robot/Robot.h>
#include <lenny/tools/Plot.h>
#include <lenny/tools/Typedefs.h>

#include <thread>

namespace lenny::control {

class RobotControlInterface {
public:
    //--- Typedefs
    LENNY_GENERAGE_TYPEDEFS(RobotControlInterface)
    typedef std::array<double, 5> PlotType;

    //--- Constructor
    RobotControlInterface(const robot::Robot& robot, const Eigen::VectorXb& dofMask, const tools::Plot<PlotType>::F_addPlot f_addPlot);
    virtual ~RobotControlInterface() = default;

    //--- Communication
    void initializeCommunication();
    void terminateCommunication();
    virtual bool isConnected() const = 0;

    //--- Get and set values
    void getCurrentValues(Eigen::VectorXd& currentRobotPosition, Eigen::VectorXd& currentRobotVelocity) const;
    void setTargetValues(const Eigen::VectorXd& targetRobotPosition, const Eigen::VectorXd& targetRobotVelocity);
    void setTargetValues(const Eigen::VectorXd& targetRobotPosition, const double& dt);
    virtual void stop();

    //--- Helpers
    bool positionReached(const Eigen::VectorXd& currentRobotPosition, const Eigen::VectorXd& targetRobotPosition) const;

    //--- Drawing
    void drawScene(double alpha = 0.5) const;
    void drawGui();

protected:
    //--- Initialization
    void initializeValues();

    //--- Read & write
    virtual void sendControlCommandsToPhysicalRobot(double dt) = 0;  //Send commands to robot based on COMMAND values
    virtual void readPhysicalRobotValues(double dt) = 0;             //Read CURRENT values from robot

    //--- Connect / Disconnect
    virtual void connect() = 0;
    virtual void disconnect() = 0;

    //--- Helpers
    uint getStateSize() const;
    Eigen::VectorXd getControlStateFromRobotState(const Eigen::VectorXd& robotState) const;
    Eigen::VectorXd getRobotStateFromControlState(const Eigen::VectorXd& controlState, const Eigen::VectorXd& fullRobotState) const;
    Eigen::VectorXd estimateControlVelocity(const Eigen::VectorXd& position1, const Eigen::VectorXd& position0, const double& dt) const;
    void computeCommandsUsingPDControl();
    void applyEmergencyStopCheck();

    //--- Communication thread
    void applyComThread();

    //--- Drawing
    virtual void drawAdditionalGuiContent() {}

    //--- Plot
    void initializePlots(const tools::Plot<PlotType>::F_addPlot f_addPlot);
    void clearPlots();
    void updatePlots();

public:
    //--- Simulated robot
    const robot::Robot& robot;

    //--- Settings
    bool readCommands = true;  //if false, current values are set by target values
    bool sendCommands = false;
    double targetFramerate = 100.0;
    bool printDebugInfo = false;

    //--- PD gains
    double k_P = 1.0, k_D = 0.0;

    //--- Drawing
    bool showCurrentRobot = true;
    bool showTargetRobot = true;
    bool showCommandRobot = true;

    Eigen::Vector3d currentDrawColor = Eigen::Vector3d(0.86275, 0.07843, 0.23529);  //Red
    Eigen::Vector3d targetDrawColor = Eigen::Vector3d(0.48627, 0.98823, 0.0);       //Lawn green
    Eigen::Vector3d commandDrawColor = Eigen::Vector3d(1.0, 0.72157, 0.10980);      //Golden Yellow

protected:
    //--- Dof mask
    const Eigen::VectorXb dofMask;    //Masks robot state dofs that are not active for physical robot
    Eigen::VectorXd fullTargetState;  //Target state of entire robot

    //--- Current, target, and command values (size based on dof mask)
    Eigen::VectorXd currentPosition, currentVelocity;  //CURRENT: Read from the robot
    Eigen::VectorXd targetPosition, targetVelocity;    //TARGET: Target for the robot to reach
    Eigen::VectorXd commandPosition, commandVelocity;  //COMMAND: Sent to the robot, computed using PD

    //--- Tolerances
    Eigen::VectorXd positionReachedTolerances, emergencyStopTolerances;  //Per DoF

    //--- Communication thread
    bool comThreadIsRunning = false;
    std::thread comThread;
    double currentFramerate = targetFramerate;
    double averageDeltaT;

    //--- Plots
    enum PLOT_ENTRY { LOWER_LIMIT, UPPER_LIMIT, CURRENT_VALUE, TARGET_VALUE, COMMAND_VALUE };
    tools::Plot<PlotType>::List positionPlots, velocityPlots;
};

}  // namespace lenny::control