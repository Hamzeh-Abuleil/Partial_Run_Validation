#ifndef __OC_EXTRINSIC_CALIBRATOR_MANAGER_H_
#define __OC_EXTRINSIC_CALIBRATOR_MANAGER_H_

#include "technology/calibration/onlineCalibration/Calibrators/CalibratorManager.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"
#include "technology/calibration/onlineCalibration/ExtrinsicCalibrationResults.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibrator.h"

namespace OnlineCalibration {

class ExtrinsicCalibratorManager : public CalibratorManager {
public:
    ExtrinsicCalibratorManager(Targets& targets);
    virtual const ExtrinsicCalibResults& getCurrentCalibration() const;
    virtual void calcResultHelper(ExtrinsicCalibrator& calibrator);
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const override;
    virtual void updateDriverProfileInput(dstruct_t* driverProfile) {}
protected:
    ExtrinsicCalibResults _calibrationResults;

};

}

#endif //__OC_EXTRINSIC_CALIBRATOR_MANAGER_H_
