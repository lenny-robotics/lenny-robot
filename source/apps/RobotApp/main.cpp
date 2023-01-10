#include "RobotApp.h"
#include "RobotTests.h"

int main() {
    testTransformationsAndDerivatives();
    testNumberOfJointsInbetween();
    testFreeFloatingBase();

    lenny::RobotApp app;
    app.run();

    return 0;
}