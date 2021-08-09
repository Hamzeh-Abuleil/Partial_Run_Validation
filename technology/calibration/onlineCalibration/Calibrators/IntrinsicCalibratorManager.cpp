#include "technology/calibration/onlineCalibration/Calibrators/IntrinsicCalibratorManager.h"

namespace OnlineCalibration {

IntrinsicCalibratorManager::IntrinsicCalibratorManager(Targets& targets) : CalibratorManager(targets) {}

void IntrinsicCalibratorManager::init() {}

const IntrinsicCalibResults& IntrinsicCalibratorManager::getCurrentCalibration() const {
    return _calibrationResults;
}

void IntrinsicCalibratorManager::calcResultHelper(IntrinsicCalibrator& calibrator) {
    const ExtrinsicCalibration& exCalib = calibrator.getCurrentCalibration();
    _calibrationResults.calibration = exCalib;
    const StateInfo& stateInfo = calibrator.getStateInfo();
    _calibrationResults.state = calcState(stateInfo);
    //_calibrationResults.fs = calcFS();
}

void IntrinsicCalibratorManager::dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
    CalibratorManager::dumpData(writer);
    _calibrationResults.dumpData(writer);
#endif
}

bool IntrinsicCalibratorManager::verifyProperties() {
    return true;
}

}
