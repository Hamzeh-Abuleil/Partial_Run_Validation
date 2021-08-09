#ifndef __OC_DEFS_H
#define __OC_DEFS_H

namespace OnlineCalibration {

enum class RunConfig {
    DONT_RUN, RUN, INVALID
};
enum State {
    GOOD, UNVALIDATED, SUSPECTED, BAD, NUM_OF_STATES
};
enum ConfLevel {
    LOW, MED, HIGH, HIGH_STRING, NUM_OF_CONF
};
enum class StateDegradeCause {
	NO_DEGRADE = 0, EM_INVALID = 1, RM_INVALID = 2, CALIB_CHANGED = 4, INTERNAL = 8, TIMEOUT=16, HIGH_VARIANCE=32
};
enum CoordSys {
    FORWARD = 0, NARROW, REAR, REAR_CORNER_LEFT, REAR_CORNER_RIGHT, FRONT_CORNER_LEFT, FRONT_CORNER_RIGHT, NUM_OF_CAMS, WORLD, NUM_OF_COORDS
};
enum CalibrationStatus {
    NOT_AVAILABLE = 0, RUNNING, CONVERGED, CalibrationStatus_ERROR
};
enum SPCError {
	SPCError_NONE = 0, SPCError_TIMEOUT, SPCError_OUT_OF_RANGE, SPCError_INTERNAL
};
enum SPCInvalidSignal {
	SPCInvalidSignal_NONE = 0, YAW = 1, PITCH = 2, ROLL = 4, CAM_H = 8
};
enum SPCInvalidReason {
    NO_PAUSE = 0, MIN_SPEED = 1, MAX_SPEED = 2, MAX_ACCELERATION = 4, MAX_YAWRATE = 8, MIN_RADIUS = 16, SPCInvalidReason_INTERNAL = 32
};
const unsigned int MAP_SIZE = CoordSys::NUM_OF_COORDS * CoordSys::NUM_OF_COORDS;
const unsigned int NUM_CAM2CAM = ((CoordSys::NUM_OF_CAMS * CoordSys::NUM_OF_CAMS) - CoordSys::NUM_OF_CAMS)/2; // (n^2-n)/2

} // namespace OnlineCalibration

#endif //__OC_DEFS_H
