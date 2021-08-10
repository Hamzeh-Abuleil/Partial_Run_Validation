#ifndef __OC_CAM2wORLD_SOURCES_H_
#define __OC_CAM2wORLD_SOURCES_H_
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "utilities/clipextIO/clipextIO.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"

namespace WorldModel {
  namespace VRM {
    class SegmentModel;
  }
}

namespace WorldModel {
  namespace EgoMotion {
    struct RoadModelStorage;
    class EmData;
  }
}

namespace OnlineCalibration {
namespace Cam2World {

  struct ParamsRaw {
    ParamsRaw() : spcMode(false), slowMode(false) {}

    bool spcMode;
    bool slowMode;
  };

  struct CameraRawData {
    CameraRawData() : focalLm2(0.f), foeFull(0, 0), foeAfix(0, 0) {}

    float focalLm2;
    CalibUtils::PixelLm2 foeFull; // yawFull/horizonFull from etc/camera.conf
    CalibUtils::PixelLm2 foeAfix; // autofix_yaw/horizon from etc/camera.conf
    CalibUtils::PixelLm2 minFoe; // bottom-left of tolerance rect from etc/camera.conf
    CalibUtils::PixelLm2 maxFoe; // top-right of tolerance rect from etc/camera.conf
    float maxRoll; // roll tolerance from etc/camera.conf
    float roll;
    float camh;
  };

  struct VehicleRawData {
    VehicleRawData() : isYawRateAvailable(false), yawRateInPixels(0.f),
    speedAvailable(false), speed(0.f), dt(0.f) {}

    bool isYawRateAvailable;
    float yawRateInPixels;
    bool speedAvailable;
    float speed;
    float dt;
  };

  struct EmRawData {
    EmRawData() : valid(false), emVisionMeasurementData(nullptr) {}

    bool valid;
    const WorldModel::EgoMotion::EmData* emVisionMeasurementData;
  };

  struct RmRawData {
    RmRawData() : valid(false), model(nullptr), storage(nullptr) {}

    bool valid;
    const WorldModel::VRM::SegmentModel *model;
    const WorldModel::EgoMotion::RoadModelStorage *storage;
  };

struct Cam2WorldSources {
    void dumpData(ClipextIO::ClipextWriter& writer) const {}

    ParamsRaw params;
    CameraRawData cam;
    VehicleRawData vh;
    EmRawData em;
    RmRawData rm;
};

}
}

#endif
