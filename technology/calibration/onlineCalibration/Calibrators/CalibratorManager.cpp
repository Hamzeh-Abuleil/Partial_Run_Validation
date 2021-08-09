#include "technology/calibration/onlineCalibration/Calibrators/CalibratorManager.h"
#include <assert.h>

namespace OnlineCalibration {

CalibratorManager::CalibratorManager(Targets targets) :
     _targets(targets) {
}

State CalibratorManager::calcStateHelper(const StateInfo& stateInfo, const unsigned int lastRunGFI) const {
    switch (_runConfig) {
    case RunConfig::RUN:
        //TODO default implementation to set state enum using only confidence
        return State::NUM_OF_STATES;
        break;
    case RunConfig::DONT_RUN:
        //TODO decrease state depending on last run gfi
        return State::NUM_OF_STATES;
        break;
    default:
        assert(true);
        return State::NUM_OF_STATES;
    }
}

void CalibratorManager::dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
    writer.setData("runConfig", reinterpret_cast<const int *>(&_runConfig));
    writer.setData("lastRunGFI", reinterpret_cast<const int *>(&_lastRunGFI));
    writer.setData("targets", reinterpret_cast<const int *>(&_targets[0]), _targets.size());
#endif
}

}
