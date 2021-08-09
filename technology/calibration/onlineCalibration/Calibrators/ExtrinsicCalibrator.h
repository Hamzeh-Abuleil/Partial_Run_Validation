#ifndef __OC_EXTRINSIC_CALIBRATOR_H
#define __OC_EXTRINSIC_CALIBRATOR_H

#include "technology/calibration/onlineCalibration/Calibrators/Calibrator.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibration.h"

namespace OnlineCalibration {

class ExtrinsicCalibrator : public Calibrator {
public:
    ExtrinsicCalibrator(const Targets& targets) : _calibration(targets) {}
    virtual void init() = 0;
    virtual const ExtrinsicCalibration& getCurrentCalibration() const {
        assert(!_calibration.getTargets().empty());
        return _calibration;
    }
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const override {
#ifdef DEBUG
        _calibration.dumpData(writer);
#endif
    }
    virtual bool verifyProperties() {
        return _calibration.verifyProperties();
    }

protected:
    ExtrinsicCalibration _calibration;
};

}

#endif
