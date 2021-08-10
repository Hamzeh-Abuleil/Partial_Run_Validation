#ifndef __OC_INTRINSIC_CALIBRATION_RESULTS_H_
#define __OC_INTRINSIC_CALIBRATION_RESULTS_H_

#include "technology/calibration/onlineCalibration/CalibrationResults.h"
#include "technology/calibration/onlineCalibration/Calibrators/IntrinsicCalibration.h"

namespace OnlineCalibration {

class IntrinsicCalibResults : public CalibrationResults {
public:
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const override {
#ifdef DEBUG
        CalibrationResults::dumpData(writer);
        calibration.dumpData(writer);
#endif
    }
    IntrinsicCalibration calibration;
};

}

#endif
