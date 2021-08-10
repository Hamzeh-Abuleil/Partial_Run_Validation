#ifndef __OC_INTRINSIC_CALIBRATION_MANAGER_H
#define __OC_INTRINSIC_CALIBRATION_MANAGER_H

#include "technology/calibration/onlineCalibration/Calibrators/CalibratorManager.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"
#include "technology/calibration/onlineCalibration/IntrinsicCalibrationResults.h"
#include "technology/calibration/onlineCalibration/Calibrators/IntrinsicCalibrator.h"

namespace OnlineCalibration {

void init(TargetsLists& targetList);

class IntrinsicCalibratorManager : public CalibratorManager {
public:
    IntrinsicCalibratorManager(Targets& targets);
    virtual void init() override;
    virtual const IntrinsicCalibResults& getCurrentCalibration() const;
    virtual void calcResultHelper(IntrinsicCalibrator& calibrator);
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const override;
    virtual bool verifyProperties() override;

protected:
    IntrinsicCalibResults _calibrationResults;
};

}

#endif
