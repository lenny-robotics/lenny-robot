#include <lenny/kineloco/Limb.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Logger.h>

namespace lenny::kineloco {

void Limb::reset() {
    trajectory.clear();
    swingPhases.clear();
}

ContactPhase Limb::getContactPhaseForTime(const double& time) const {
    //If there ar no swing phases, we leave it in stance
    if (swingPhases.size() == 0)
        return ContactPhase(false, 100.0, 50.0);

    //Time might be before any of the swing phases are starting
    if (swingPhases.front().first > time)
        return ContactPhase(false, swingPhases.front().first - time, swingPhases.front().first - time);

    //Time could be after the last swing phase ends
    if (swingPhases.back().second < time)
        return ContactPhase(false, 100.0, 50.0);

    //Time falls between or within swing phases
    for (int i = 0; i < swingPhases.size(); i++) {
        //Time falls within swing phase
        const auto& [start_i, end_i] = swingPhases[i];
        if (time >= start_i && time <= end_i)
            return ContactPhase(true, end_i - start_i, end_i - time);

        if (i + 1 < swingPhases.size()) {
            //Time falls within stance phase
            const auto& [start_ip1, end_ip1] = swingPhases[i + 1];
            if (time <= start_ip1)
                return ContactPhase(false, start_ip1 - end_i, start_ip1 - time);
        }
    }

    //Unforeseen case
    LENNY_LOG_WARNING("Unforeseen use case...")
    return ContactPhase(false, 100, 50);
}

void Limb::drawGui() {
    using tools::Gui;
    if (Gui::I->TreeNode(("Limb - `" + linkName + "`").c_str())) {
        if (Gui::I->TreeNode("Time Interval")) {
            if (Gui::I->Slider("Lower", timeInterval.first, 0.0, 1.0))
                if (timeInterval.first > timeInterval.second)
                    timeInterval.first = timeInterval.second;
            if (Gui::I->Slider("Upper", timeInterval.second, 0.0, 1.0))
                if (timeInterval.first > timeInterval.second)
                    timeInterval.second = timeInterval.first;

            Gui::I->TreePop();
        }

        int iter = 1;
        for (const auto& [start, end] : swingPhases) {
            if (Gui::I->TreeNode(("Swing Phase - " + std::to_string(iter++)).c_str())) {
                Gui::I->Text("Start: %lf", start);
                Gui::I->Text("End: %lf", end);
                Gui::I->TreePop();
            }
        }

        Gui::I->TreePop();
    }
}

}  // namespace lenny::kineloco