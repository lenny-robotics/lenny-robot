#pragma once

#include <lenny/agents/ABBYuMiAgent.h>
#include <lenny/agents/BDSpotArmAgent.h>
#include <lenny/agents/FrankaPandaAgent.h>
#include <lenny/agents/KinovaGen3Agent.h>
#include <lenny/agents/UR5eAgent.h>
#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>

namespace lenny {

class PrimitiveGeneratorApp : public gui::Application {
public:
    PrimitiveGeneratorApp();
    ~PrimitiveGeneratorApp() = default;

    void autogenerateCollisionPrimitives();
    void process() override;

    void drawScene() const override;
    void drawGui() override;

public:
    agents::UR5eRobot robot = agents::UR5eRobot(gui::Model::f_loadModel);
    agents::UR5eAgent agent = agents::UR5eAgent("Agent", robot);

    //    agents::ABBYuMiRobot robot = agents::ABBYuMiRobot(gui::Model::f_loadModel);
    //    agents::ABBYuMiAgent agent = agents::ABBYuMiAgent("Agent", robot);

    //    agents::FrankaPandaRobot robot = agents::FrankaPandaRobot(gui::Model::f_loadModel);
    //    agents::FrankaPandaAgent agent = agents::FrankaPandaAgent("Agent", robot);

    //    agents::KinovaGen3Robot robot = agents::KinovaGen3Robot(gui::Model::f_loadModel);
    //    agents::KinovaGen3Agent agent = agents::KinovaGen3Agent("Agent", robot);

    //    agents::BDSpotArmRobot armRobot = agents::BDSpotArmRobot(gui::Model::f_loadModel);
    //    agents::BDSpotBaseRobot baseRobot = agents::BDSpotBaseRobot(gui::Model::f_loadModel);
    //    agents::BDSpotArmAgent agent = agents::BDSpotArmAgent("Agent", armRobot, baseRobot);

    double sphereTolerance = 0.1;
    double capsuleTolerance = 0.1;
    double dimFactor = 1.0;
};

}  // namespace lenny