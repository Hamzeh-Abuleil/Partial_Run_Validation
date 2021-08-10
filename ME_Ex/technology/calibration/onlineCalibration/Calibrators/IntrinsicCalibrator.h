#ifndef __OC_INTRINSIC_CALIBRATOR_H
#define __OC_INTRINSIC_CALIBRATOR_H

#include "technology/calibration/onlineCalibration/Calibrators/Calibrator.h"
#include "technology/calibration/onlineCalibration/Calibrators/IntrinsicCalibration.h"

namespace OnlineCalibration {

class IntrinsicCalibrator : public Calibrator {
public:
    IntrinsicCalibrator(const Targets& targets) : _calibration(targets) {}
    virtual const IntrinsicCalibration& getCurrentCalibration() const {
        assert(!_calibration.getTargets().empty());
        return _calibration;
    }
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const override {
#ifdef DEBUG
        _calibration.dumpData(writer);
#endif
    }
protected:
    IntrinsicCalibration _calibration;
};

}

#endif

