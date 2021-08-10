#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/planeSync.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wOutputIF.h"

namespace OnlineCalibration {
  namespace Cam2World {
    
    PlaneSync::PlaneSync() : _planes(PLANE_BUFFER_CAPACITY), _data(nullptr)
    {
      reset();
    }

    void PlaneSync::reset() {
      _planes.clear();
      for (int i=0; i<3; ++i) {
        _underlyingPlane.N[i]=0; // set to 0 0 0
      }
      _underlyingPlane.d=0.f;
      _underlyingPlane.location=0;
      _underlyingPlane.valid=false;
    }

    bool PlaneSync::run(const Cam2WorldData *data) {
      _data = data;
      if (_data->em.valid) {
        updatePlanes(); //update all the planes that are already in the buffer: move them from the last frame to the current frame using R and t from Ego Motion
      } else {
        _planes.clear(); //em is not valid so we can't keep moving the planes forward. we have to start over.
      }
      if (_data->rm.valid & _data->rm.planeValidity) {
        addPlaneToBuffer(); //add the road model data from the current frame to the plane buffer
      }
      bool validPlane=calcUnderlyingPlane(); //calculate the underlying plane for the current frame
      Cam2WorldOutputIF::instance().toItrkPlaneBuffer(&_planes);
      return validPlane;
    }

    //here we update all the planes in the buffer to the current coordinate system, using a newly arrived ego
    //motion R,t matrix. this is the math:
    //suppose the previous plane representation is N,d and the current is N',d', i.e.:
    //NTv=d for v in prev coor.sys, and N'Tw=d' for w in current. we can write w=Rv+T and we get:
    //N'T(Rv+t)=d', or: N'TRv=d'-N'Tt
    //so we get: N'TR = NT => (N'TR)T = N => RTN'=N => N'=RN (since R is a rotation matrix)
    //and: d = d'-N'Tt => d' = d + N'Tt
    //(we can develop the last equation more but there is no need since we just computed N')
    //comments about signs and so: we expect N to be close to (0,1,0) since
    //it is the normal to road, and d to be negative, since the plane is under
    //the vehicle. if this is not the case this is a problem.
    //TODO: once basicTypes planes include this api, move to it
    void PlaneSync::updatePlanes() {
      Float::MEmath::Mat<3,3,float> R = _data->em.R;
      Float::MEmath::Vec<3,float> t = _data->em.t;
      for (MEtypes::FastCyclicVector<Plane>::iterator iter = _planes.begin(); iter!=_planes.end(); iter++) {
        iter->N = R*(iter->N);//this is N' = RN
        iter->d = iter->d+iter->N*t;//note that iter->N here is N'
      }
    }

    void PlaneSync::addPlaneToBuffer() {
      Plane plane;
      plane.N = _data->rm.N;
      plane.d = -_data->rm.camh;
      plane.location = _data->vehicle.totalDistance + _data->rm.Zstart + futurePlaneFac*(_data->rm.Zend-_data->rm.Zstart)/2;
      plane.valid = _data->rm.valid;
      _planes.push_back(plane);
    }

    bool PlaneSync::calcUnderlyingPlane() {
      bool validPlane=false;
      float dSum=0;
      Float::MEmath::Vec<3,float> Nsum;
      for (int i=0; i<3; ++i) {
        Nsum[i]=0;
      }
      int count=0;
      float totalDistance = _data->vehicle.totalDistance;
      
      for (MEtypes::FastCyclicVector<Plane>::iterator iter = _planes.begin(); iter!=_planes.end(); iter++) {
        Plane currPlane = *iter;

        if (me_fabs(currPlane.location-totalDistance)<maxDistFromLocation) {
          dSum+=currPlane.d;
          Nsum+=currPlane.N;
          count++;
        }

      }
      if (count>0) {
        _underlyingPlane.d=dSum/count;
        _underlyingPlane.N=Nsum/count;
        _underlyingPlane.location=totalDistance;
        validPlane=true;
        // _underlyingPlane.valid=_planes.front().valid;
      }
      
      return validPlane;
    
    }

    void PlaneSync::calcRollAndCamH(const float totalDistance, float &roll, float &camh, float &location) {
      roll=me_atanf(-_underlyingPlane.N[0]/_underlyingPlane.N[1]);
      camh=-_underlyingPlane.d;
      location=_underlyingPlane.location;
    }


  } // Cam2World
} // OnlineCalibration
