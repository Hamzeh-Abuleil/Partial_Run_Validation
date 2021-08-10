/**
 * \file  c2wUtils.cpp
 * \brief Math Utilities
 *
 * \author Uri London
 * \date Jul 11, 2019
 */

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wUtils.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wConsts.h"
#include "basicTypes/transformationMatrices_API.h"
#include "technology/calibration/utilities/cameraProjections/distortionCorrection_algoAPI.h"

namespace OnlineCalibration {
  namespace Cam2World {

    Float::MEmath::Mat<3, 3, float> composeRotationMatrixFromSensorAngles(const float yaw,
                                                                           const float pitch,
                                                                           const float roll) {
      // /homes/urilo/Documents/math/rotation/vehicle2cam.pdf
      Float::MEmath::Vec<3, float> t;
      Float::MEmath::Vec<3, float> N;
      float ty = me_tanf(yaw);
      float tp = me_tanf(pitch);
      float tr = me_tanf(roll);

      t[2] = 1/me_sqrtf(1 + ty*ty+ tp*tp);
      t[0] = ty*t[2];
      t[1] = tp*t[2];

      N[1] = 1/me_sqrtf(1 + tr*tr + (ty*tr-tp)*(ty*tr-tp));
      N[0] = -tr*N[1];
      N[2] = (ty*tr-tp)*N[1];

      Float::MEmath::Vec<3, float> crossNt = N/t;

      Float::MEmath::Mat<3, 3, float> R;
      for (int i = 0; i < 3; ++i) {
        R(i, 0) = crossNt[i];
        R(i, 1) = N[i];
        R(i, 2) = t[i];
      }

      return R;
    }

    Float::MEmath::Mat<3, 3, double> composeRotationMatrixFromSensorAngles(const double yaw,
                                                                           const double pitch,
                                                                           const double roll) {
      Float::MEmath::Vec<3, double> t;
      Float::MEmath::Vec<3, double> N;
      double ty = me_tan(yaw);
      double tp = me_tan(pitch);
      double tr = me_tan(roll);

      t[2] = 1/me_sqrtd(1 + ty*ty+ tp*tp);
      t[0] = ty*t[2];
      t[1] = tp*t[2];

      N[1] = 1/me_sqrtd(1 + tr*tr + (ty*tr-tp)*(ty*tr-tp));
      N[0] = -tr*N[1];
      N[2] = (ty*tr-tp)*N[1];

      Float::MEmath::Vec<3, double> crossNt = N/t;

      Float::MEmath::Mat<3, 3, double> R;
      for (int i = 0; i < 3; ++i) {
        R(i, 0) = crossNt[i];
        R(i, 1) = N[i];
        R(i, 2) = t[i];
      }

      return R;
    }

    void decomposeRotationMatrixIntoSensorAngles(const Float::MEmath::Mat<3, 3, float> R, float& yaw, float& pitch, float& roll) {
      // yaw   = (float)me_atan(R(0, 2)/R(2, 2)); // tx/tz
      // pitch = (float)me_atan(R(1, 2)/R(2, 2)); // ty/tz
      // roll  = (float)me_atan(-R(0, 1)/R(1, 1)); // -nx/ny
      yaw   = me_atanf(R(0, 2)/R(2, 2)); // tx/tz
      pitch = me_atanf(R(1, 2)/R(2, 2)); // ty/tz
      roll  = me_atanf(-R(0, 1)/R(1, 1)); // -nx/ny
    }

    bool anglesToPixel(float yaw, float pitch, CalibUtils::PixelLm2_f& foe) {
      float focalLm2 = CameraInfo::focalLength(CameraInfo::e_FORWARD)*L0_TO_LM2_SCALE;
      float x = yaw*focalLm2;
      float y = pitch*focalLm2;

      bool ok = DistortionCorrectionAPI::isDistortionValid(CameraInfo::e_FORWARD);
      if (ok) {
        ok = DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, x, y, foe[0], foe[1]);
      }

      return ok;
    }

    void foeBoundariesFromImageCenter2PP(CalibUtils::PixelLm2 minFoe_cen, CalibUtils::PixelLm2 maxFoe_cen, CalibUtils::PixelLm2& minFoe_pp, CalibUtils::PixelLm2& maxFoe_pp){
      // relevant camera instance:
      CameraInfo::CameraInstance inst = CameraInfo::CameraInstance(CameraInfo::e_FORWARD);
      // get allocated image center coors
      int distLimitLm2X = PrepSys_API::getLM2ROI(inst).allocatedWidth(); // this block is a modified version of what appears in wmbc.cpp/setCameraDataInit
      int distLimitLm2Y = PrepSys_API::getLM2ROI(inst).allocatedHeight();
      CalibUtils::PixelLm2_f distNominalLm2(distLimitLm2X/2, distLimitLm2Y/2);
      // get pp coordinates in allocated image
      // CalibUtils::PixelLm2_f PP(963.24884, 359.615967); //TODO: ****THIS IS TEMP FRO DEBUG!!!! change something normal from came info******
      CalibUtils::PixelLm2_f PP(CameraInfo::getPreciseCamK(CameraInfo::e_FORWARD)(0, 2), CameraInfo::getPreciseCamK(CameraInfo::e_FORWARD)(1, 2));
      // set vector from camera center to PP:
      CalibUtils::PixelLm2_f pp2center = PP-distNominalLm2;

      // fix the foe to relative postion of pp
      minFoe_pp[0] = minFoe_cen.X() - pp2center.X();
      minFoe_pp[1] = minFoe_cen.Y() - pp2center.Y();
      maxFoe_pp[0] = maxFoe_cen.X() - pp2center.X();
      maxFoe_pp[1] = maxFoe_cen.Y() - pp2center.Y();
    }

    bool pixelToAngles(const CalibUtils::PixelLm2_f foe, float& yaw, float& pitch) {
      float focalLm2 = CameraInfo::focalLength(CameraInfo::e_FORWARD)*L0_TO_LM2_SCALE;
      float x = 0.f;
      float y = 0.f;
      bool ok = DistortionCorrectionAPI::isDistortionValid(CameraInfo::e_FORWARD);
      if (ok) {
        ok = DistortionCorrectionAPI::rectifySafe(CameraInfo::e_FORWARD, -2, foe[0], foe[1], x, y);
        yaw = me_atanf(x/focalLm2);
        pitch = me_atanf(y/focalLm2);
      }

      return ok;
    }

    bool pixelToRodriguezRotationMatrix(const CalibUtils::PixelLm2_f foe, float roll, Float::MEmath::Mat<3, 3, float> &R) {
      float yaw = 0.f;
      float pitch = 0.f;
      Float::MEmath::Mat<3, 3, double> R_d;
      bool ok = pixelToAngles(foe, yaw, pitch);
      Float::MEmath::ypr2R((double)yaw, (double)pitch, (double)roll, R_d);
      R = matd2f(R_d);
      return ok;
    }

    bool rotationMatrixToSensorPixel(const Float::MEmath::Mat<3, 3, float> R, CalibUtils::PixelLm2_f& foe, float& roll) {
      float yaw = 0.f;
      float pitch = 0.f;
      decomposeRotationMatrixIntoSensorAngles(R, yaw, pitch, roll);
      return anglesToPixel(yaw, pitch, foe);
    }

    bool rotationMatrixToRodriguezPixel(const Float::MEmath::Mat<3, 3, float> R, CalibUtils::PixelLm2_f& foe, float& roll) {
      double yaw_d   = 0.0;
      double pitch_d = 0.0;
      double roll_d  = 0.0;
      Float::MEmath::Mat<3, 3, double> R_d = matf2d(R);
      Float::MEmath::R2ypr(R_d, yaw_d, pitch_d, roll_d);
      roll = (float)roll_d;
      return anglesToPixel((float)yaw_d, (float)pitch_d, foe);
    }

    Float::MEmath::Mat<3, 3, double> matf2d(Float::MEmath::Mat<3, 3, float> matf) {
      Float::MEmath::Mat<3, 3, double> matd;
      for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
          matd(i, j) = (double)matf(i, j);
        }
      }
      return matd;
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

    void matf2arr(const Float::MEmath::Mat<3, 3, float> matf, float arr[]) {
      for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
          arr[i+3*j] = matf(i, j);
        }
      }
    }

    void vecf2arr(const Float::MEmath::Vec<3, float> vecf, float arr[]) {
      for (int i=0; i<3; ++i) {
          arr[i] = vecf[i];
      }
    }

    void copyArr(int size, const float arrIn[], float arr[]) {
      for (int i=0; i<size; ++i) {
        arr[i] = arrIn[i];
      }
    }

  } // namespace Cam2World
} // namespace OnlineCalibration

