#ifndef __OC_INTRINSIC_CALIBRATION_H_
#define __OC_INTRINSIC_CALIBRATION_H_

#include "technology/calibration/onlineCalibration/Calibrators/Calibration.h"

namespace OnlineCalibration {

class IntrinsicCalibration : public Calibration {
public:
    IntrinsicCalibration() : Calibration(Targets()) {}
    IntrinsicCalibration(const Targets& targets) : Calibration(targets) {}
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const override {
#ifdef DEBUG
        Calibration::dumpData(writer);
#endif
    }
};

}

#endif //__OC_INTRINSIC_CALIBRATION_H_
