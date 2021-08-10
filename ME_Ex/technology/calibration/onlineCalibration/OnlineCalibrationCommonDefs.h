#ifndef __OC_COMMON_DEFS_H
#define __OC_COMMON_DEFS_H

#include "technology/calibration/onlineCalibration/OnlineCalibrationDefs.h"
#include "functionality/calibration/cameraInformationProperTypes.h"
#include "utilities/clipextIO/clipextIO.h"
#include "technology/worldModel/roadModel/roadModelDefs.h"
#include "technology/worldModel/egoMotion/egoMotionExternalDefs.h"

namespace OnlineCalibration {

using Targets = std::vector<CoordSys>;
using TargetsLists = std::vector<Targets>;

// map from CoordsSys to CameraInstance
CameraInfo::CameraInstance coordsToInstance(CoordSys coords);

// map from CoordsSys to basicTypes CoordinateSystem
MEtypes::CoordinateSystem coordsToMECoords(CoordSys coords);

const char* coordsToStr(CoordSys coords);

} // namespace OnlineCalibration

#endif //__OC_COMMON_DEFS_H
