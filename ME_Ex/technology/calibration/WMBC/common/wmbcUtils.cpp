#include "technology/calibration/WMBC/common/wmbcUtils.h"
#include "technology/calibration/WMBC/common/wmbcTypes.h"
#include "technology/calibration/WMBC/common/wmbc_dbg.h"
#include "basicTypes/transformationMatrices_API.h"
#include "technology/calibration/utilities/cameraProjections/distortionCorrectionAPI.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "technology/calibration/cameraToCamera/cameraToCamera_API_internal.h"

namespace WMBC {

  const int MAX_FRAME_NUM = 50;

  bool transformVehicleToCam(CameraInfo::CameraInstance inst, const VehicleToCam& v2cPrimary, VehicleToCam& v2c) {
    if (!CameraInfo::exists(inst)) {
      return false;
    }

    Mat4d cam2cam = CameraToCameraAPI::getTransformationMatrix(CameraInfo::e_FORWARD, inst);
    
    Mat3d cam2camR;
    Vec3d cam2camT;
    WorldModel::decomposeTransformationMatrixIntoRt(cam2cam, cam2camR, cam2camT);

    Vec3d Nin;
    for (int i = 0; i < 3; ++i) {
      Nin[i] = v2cPrimary.R(i, 1);
    }
    Vec3d N = cam2camR*Nin;
    
    v2c.update(cam2camR * v2cPrimary.R, -cam2camT*N + v2cPrimary.camh); // R' = C2C*R, H' = H - T*N

    return true;
  }

  Float::MEmath::Mat<3, 3, float> matd2f(Float::MEmath::Mat<3, 3, double> matd) {
    Float::MEmath::Mat<3, 3, float> matf;
    for (int i=0; i<3; ++i) {
      for (int j=0; j<3; ++j) {
        matf(i, j) = (float)matd(i, j);
      }
    }
    return matf;
  }

  Float::MEmath::Vec<3, float> vecd2f(Float::MEmath::Vec<3, double> vecd) {
    Float::MEmath::Vec<3, float> vecf;
    for (int i=0; i<3; ++i) {      
      vecf[i]= (float)vecd[i];
    }
    return vecf;
  }

  float linearActivation(float x, float a, float b, bool zeroToOne) {
    // zeroToOne - return function from 0 to 1, false - from -1 to 1
    float y  = 0.f;

    if (me_fabs(b-a) < EPS) {
      return y;
    }

    y = zeroToOne ? x-a : a+b-2*x;
    y /= b-a;
    float y0 = zeroToOne ? 0.f : -1.f;
    y = std::max(y0, std::min(1, y));

    return y;
  }

  float sigmoidActivation(float x, float a, float b, float w) {
    float y = linearActivation(x, a, b, false);
    return 1.f/(1.f + me_expf(-w*y));
  }

#ifndef EYEQ_HW_IMPL

  FrameRangeData::FrameRangeData() : _startFramesVec(MAX_FRAME_NUM), _endFramesVec(MAX_FRAME_NUM) {}

  bool FrameRangeData::loadFile(std::string path) {
    // std::ifstream file("/mobileye/algo_STEREO/urilo/wmbc/cfg/pls/bmwTunnelPls.jump");
    std::ifstream file("/mobileye/algo_STEREO/urilo/wmbc/cfg/plsbmw/bmwplstunnelpath.jump");
    // file.open("/homes/urilo/work/sfmTac/list/bmw_16-02-10/bmw_tunnel.jump", std::ios_base::in);

    if (!file.is_open()) {
      return false;
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.compare(0, 1, "#") == 0) {
        continue;
      }

      std::stringstream  lineStream(line);
      std::string clipname;
      int startframe, endframe;
      lineStream >> clipname >> startframe >> endframe;
      _startFramesVec.push_back(startframe);
      _endFramesVec.push_back(endframe);
    }

    if (_startFramesVec.size() != _endFramesVec.size()) {
      return false;
    }

    _frameNum = _startFramesVec.size();

    if (_frameNum == 0) {
      return false;
    }

    return true;
  }

  int FrameRangeData::getStartFrame(unsigned int i) const {
    if (/*i < 0 ||*/ i >= _startFramesVec.size()) {
      return -1;
    }

    return _startFramesVec[i];
  }

  int FrameRangeData::getEndFrame(unsigned int i) const {
    if (/*i < 0 ||*/ i >= _endFramesVec.size()) {
      return -1;
    }

    return _endFramesVec[i];
  }
#endif
}

