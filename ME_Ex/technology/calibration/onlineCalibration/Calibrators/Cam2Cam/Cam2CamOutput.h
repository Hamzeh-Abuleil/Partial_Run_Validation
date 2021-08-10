/*
 * Cam2CamOutput.h
 *
 *  Created on: Dec 10, 2019
 *      Author: sarap
 */

#ifndef _CAM2CAM_OUTPUT_H_
#define _CAM2CAM_OUTPUT_H_

#include "technology/calibration/onlineCalibration/OnlineCalibrationDefs.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibration.h"
#include "technology/calibration/onlineCalibration/Calibrators/StateInfo.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamInternalStateInfo.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationModelIF.h"
#include "basicTypes/mfl/itrkWriter_API.h"

namespace OnlineCalibration {

namespace Cam2Cam {

class Cam2CamOutput {
public:
	static Cam2CamOutput& instance();
	~Cam2CamOutput() {}

    void toItrkHeaders();
    void toItrkOutput(State state, const ExtrinsicCalibration &ec, const StateInfo &si) const;
    void toItrkInternalState(const ExtrinsicCalibration &ec, const Cam2CamInternalStateInfo &isi) const;
    void toItrkModelIF(const OnlineCalibrationModelIF *m, int index) const;

private:
    Cam2CamOutput();
    enum ItrkType {e_OUTPUT,
                   e_MODEL_IF,
                   e_CALIBRATOR_INTERNAL_STATE,
                   e_ITRK_NUM_TYPES};

    itrkWriter::ItrkWriterHandle *_itrkHandle[e_ITRK_NUM_TYPES];

};

} /* namespace Cam2Cam */

} /* namespace OnlineCalibration */

#endif /* _CAM2CAM_OUTPUT_H_ */
