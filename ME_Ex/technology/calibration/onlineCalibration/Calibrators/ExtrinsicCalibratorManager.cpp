#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibratorManager.h"

namespace OnlineCalibration {


ExtrinsicCalibratorManager::ExtrinsicCalibratorManager(Targets& targets) :
    CalibratorManager(targets) {
}

const ExtrinsicCalibResults& ExtrinsicCalibratorManager::getCurrentCalibration() const {
    return _calibrationResults;
}

void ExtrinsicCalibratorManager::calcResultHelper(ExtrinsicCalibrator& calibrator) {
    const ExtrinsicCalibration& exCalib = calibrator.getCurrentCalibration();
    _calibrationResults.calibration = exCalib;
    _calibrationResults.state = calcState( _calibrationResults.stateDegradeCause);
    //_calibrationResults.fs = calcFS();
}

void ExtrinsicCalibratorManager::dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
    CalibratorManager::dumpData(writer);
    _calibrationResults.dumpData(writer);
#endif
}

}
