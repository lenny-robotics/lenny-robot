#pragma once

namespace lenny::kineloco {

class ContactPhase {
public:
    ContactPhase(const bool inSwingMode, const double& duration, const double& timeLeft);
    ~ContactPhase() = default;

    bool isInSwing() const;

    double getTimeLeft() const;
    double getDuration() const;
    double getPercentageOfTimeEllapsed() const;

private:
    bool inSwingMode;
    double duration, timeLeft;
};

}  // namespace lenny::kineloco