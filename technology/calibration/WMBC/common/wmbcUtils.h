#ifndef _WMBC_UTILS_H_
#define _WMBC_UTILS_H_

#include "utilities/cameraInformation/cameraInformation_API.h"
 #include "technology/mobilib/float/common/MEmath/mat.h"
 #include "technology/mobilib/float/common/MEmath/vec.h"

namespace Float {
  namespace MEmath {
    template <int ROWS, class T> class Vec;
    template <int ROWS, int COLS, class T> class Mat;
  }
}


namespace WMBC {

  struct VehicleToCam;

  bool transformVehicleToCam(CameraInfo::CameraInstance instI, const VehicleToCam& v2cPrimary, VehicleToCam& v2c);
  
  struct VehicleToCam;

  bool transformVehicleToCam(CameraInfo::CameraInstance instI, const VehicleToCam& v2cPrimary, VehicleToCam& v2c);

  Float::MEmath::Mat<3, 3, float> matd2f(Float::MEmath::Mat<3, 3, double> matd); 
  Float::MEmath::Vec<3, float> vecd2f(Float::MEmath::Vec<3, double> vecd); 
  float linearActivation(float x, float a, float b, bool zeroToOne=true);
  float sigmoidActivation(float x, float a, float b, float w=1.f);

#ifndef EYEQ_HW_IMPL
  class FrameRangeData {
  public:
    FrameRangeData();
    
    bool loadFile(std::string path);
    int getStartFrame(unsigned int i) const;
    int getEndFrame(unsigned int i) const;
    int getFrameNum() const { return _frameNum;}

  private:
    std::vector<int> _startFramesVec, _endFramesVec;
    int _frameNum;
  };
#endif // EYEQ_HW_IMPL

}

#endif // _WMBC_UTILS_H_

