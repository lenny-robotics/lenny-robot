#pragma once

#include <lenny/agents/ABBYuMiAgent.h>
#include <lenny/agents/BDSpotArmAgent.h>
#include <lenny/agents/BDSpotBaseAgent.h>
#include <lenny/agents/FloatingBaseAgent.h>
#include <lenny/agents/FrankaPandaAgent.h>
#include <lenny/agents/KinovaGen3Agent.h>
#include <lenny/agents/UR5eAgent.h>
#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>

namespace lenny {

class AgentsApp : public gui::Application {
public:
    AgentsApp();
    ~AgentsApp() = default;

    //--- Drawing
    void drawScene() const override;
    void drawGui() override;

public:
    agents::ABBYuMiRobot yuMiRobot = agents::ABBYuMiRobot(gui::Model::f_loadModel);
    agents::FloatingBaseRobot baseRobot = agents::FloatingBaseRobot(gui::Model::f_loadModel);
    agents::FrankaPandaRobot pandaRobot = agents::FrankaPandaRobot(gui::Model::f_loadModel);
    agents::KinovaGen3Robot kinovaRobot = agents::KinovaGen3Robot(gui::Model::f_loadModel);
    agents::UR5eRobot urRobot = agents::UR5eRobot(gui::Model::f_loadModel);
    agents::BDSpotFloatingRobot spotFloatingRobot = agents::BDSpotFloatingRobot(gui::Model::f_loadModel);
    agents::BDSpotBaseRobot spotBaseRobot = agents::BDSpotBaseRobot(gui::Model::f_loadModel);
    agents::BDSpotArmRobot spotArmRobot = agents::BDSpotArmRobot(gui::Model::f_loadModel);

    std::vector<rapt::Agent::SPtr> agents = {std::make_shared<agents::ABBYuMiAgent>("YuMi", yuMiRobot),
                                              std::make_shared<agents::FloatingBaseAgent>("Base", baseRobot),
                                              std::make_shared<agents::FrankaPandaAgent>("Panda", pandaRobot),
                                              std::make_shared<agents::KinovaGen3Agent>("Kinova", kinovaRobot),
                                              std::make_shared<agents::UR5eAgent>("UR5", urRobot),
                                              std::make_shared<agents::BDSpotBaseAgent>("Spot Base", spotFloatingRobot, spotBaseRobot),
                                              std::make_shared<agents::BDSpotArmAgent>("Spot Arm", spotArmRobot, spotBaseRobot)};
};

}  // namespace lenny