#pragma once

#include <lenny/tools/Timer.h>

namespace lenny::rapt {

class Gripper {
public:
    //--- Constructor
    Gripper() = default;
    ~Gripper() = default;

    //--- Helpers
    void close();
    void open();
    void setTargetFingerPercentage(const double& newTargetFingerPercentage);
    double getCurrentFingerPercentage() const;
    void updateFingerPercentage() const;

    //--- Drawing
    void drawGui(const std::string& description);

public:
    double fingerVelocity = 1.5;  //percentage per second

private:
    mutable tools::Timer timer;
    mutable double currentFingerPercentage = 0.5;  //Between 0 and 1, where 0 is fully closed, and 1 is full open
    double targetFingerPercentage = currentFingerPercentage;
};

}  // namespace lenny::rapt