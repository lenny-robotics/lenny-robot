#include <lenny/kineloco/ContactPhase.h>
#include <lenny/tools/Utils.h>

namespace lenny::kineloco {

ContactPhase::ContactPhase(const bool inSwingMode, const double& duration, const double& timeLeft)
    : inSwingMode(inSwingMode), duration(duration), timeLeft(timeLeft) {
    if (this->duration < 1e-5)
        this->duration = 1e-5;
    tools::utils::boundToRange(this->timeLeft, 0.0, this->duration);
}

bool ContactPhase::isInSwing() const {
    return inSwingMode;
}

double ContactPhase::getTimeLeft() const {
    return timeLeft;
}

double ContactPhase::getDuration() const {
    return duration;
}

double ContactPhase::getPercentageOfTimeEllapsed() const {
    return 1.0 - (duration - timeLeft) / duration;
}

}  // namespace lenny::kineloco
