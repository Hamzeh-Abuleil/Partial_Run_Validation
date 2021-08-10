#ifndef __OC_EXTRINSIC_CALIBRATION_RESULTS_H_
#define __OC_EXTRINSIC_CALIBRATION_RESULTS_H_

#include "technology/calibration/onlineCalibration/CalibrationResults.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibration.h"

namespace OnlineCalibration {

class ExtrinsicCalibResults : public CalibrationResults {
public:
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const override {
#ifdef DEBUG
        CalibrationResults::dumpData(writer);
        calibration.dumpData(writer);
#endif
    }
    ExtrinsicCalibration calibration;
};

}

#endif //__OC_CALIBRATION_RESULTS_H_
