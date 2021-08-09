#ifndef __OC_PLANE_SYNC_H
#define __OC_PLANE_SYNC_H

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wData.h"
#include "basicTypes/containers/fastCyclicVector.h"

namespace OnlineCalibration {
  namespace Cam2World {

    const float futurePlaneFac = 0.45;
    const float maxDistFromLocation = 3;
    
    struct Plane {
      Plane(): N(Float::MEmath::zeros<3, float>()), d(0.f), location(0.f), valid(false) {}
      Float::MEmath::Vec<3,float> N;
      float d;
      float location;
      bool valid;
    };

      class PlaneSync {
      public:
        PlaneSync();
        ~PlaneSync() {}
        
        void reset();
        bool run(const Cam2WorldData *data);
        void calcRollAndCamH(const float totalDistance, float &roll, float &camh, float &location);
        
      private:
        void updatePlanes();
        void addPlaneToBuffer();
        bool calcUnderlyingPlane();
        
        MEtypes::FastCyclicVector<Plane> _planes;
        Plane _underlyingPlane;
        const Cam2WorldData *_data;
        
      };
    
  } // Cam2World
} // OnlineCalibration

#endif // __OC_PLANE_SYNC_H

