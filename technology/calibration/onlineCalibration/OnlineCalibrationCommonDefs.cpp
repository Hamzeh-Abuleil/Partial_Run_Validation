#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"

namespace OnlineCalibration
{

CameraInfo::CameraInstance coordsToInstance(CoordSys coords)
{
    assert(coords < NUM_OF_CAMS);
    static const CameraInfo::CameraInstance coordsToInst[NUM_OF_CAMS] = {
        CameraInfo::e_FORWARD,
        CameraInfo::e_FORWARD_NARROW,
        CameraInfo::e_REAR,
        CameraInfo::e_REAR_CORNER_LEFT,
        CameraInfo::e_REAR_CORNER_RIGHT,
        CameraInfo::e_FRONT_CORNER_LEFT,
        CameraInfo::e_FRONT_CORNER_RIGHT};
    return coordsToInst[coords];
}

MEtypes::CoordinateSystem coordsToMECoords(CoordSys coords)
{
    assert(coords < NUM_OF_CAMS);
    static const MEtypes::CoordinateSystem coordsToME[NUM_OF_CAMS] = {
        MEtypes::CoordinateSystem::CoS_FORWARD,
        MEtypes::CoordinateSystem::CoS_FORWARD_NARROW,
        MEtypes::CoordinateSystem::CoS_REAR,
        MEtypes::CoordinateSystem::CoS_REAR_CORNER_LEFT,
        MEtypes::CoordinateSystem::CoS_REAR_CORNER_RIGHT,
        MEtypes::CoordinateSystem::CoS_FRONT_CORNER_LEFT,
        MEtypes::CoordinateSystem::CoS_FRONT_CORNER_RIGHT};
    return coordsToME[coords];
}

const char* coordsToStr(CoordSys coords)
{
    assert(coords < NUM_OF_CAMS);
    switch (coords)
    {
    case FORWARD:
        return "main";
    case NARROW:
        return "narrow";
    case REAR:
        return "rear";
    case REAR_CORNER_LEFT:
        return "rearCornerLeft";
    case REAR_CORNER_RIGHT:
        return "rearCornerRight";
    case FRONT_CORNER_LEFT:
        return "frontCornerLeft";
    case FRONT_CORNER_RIGHT:
        return "frontCornerRight";
    default:
        return "";
    }
}

} // namespace OnlineCalibration
