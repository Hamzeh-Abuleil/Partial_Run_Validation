/*
 * wmbcAutofixTypes.h
 *
 *  Created on: Nov 12, 2019
 *      Author: urilo
 */

#ifndef WMBC_AUTOFIX_TYPES_H_
#define WMBC_AUTOFIX_TYPES_H_

#include "basicTypes/geo/geo2D/Pixel.h"
#include "basicTypes/geo/GeometryDefinitions.h"

namespace WMBC {

  typedef MEtypes::DynPixel<int,MEtypes::DistortionModel::DM_ORIGINAL,MEtypes::PixLevel::PIXLEVEL_0> PixelL0_i;
  static const float MIN_AUTOFIX_LARGE_DELTA = 2.5f; // should match the same var in utilities/autoFix/common/AutoFix.cpp

  struct AutofixOutput {
    AutofixOutput() { foeL0.X()=foeL0.Y()=0; foeAngle[0]=foeAngle[1]=0.f;}
    AutofixOutput(const AutofixOutput& a);
    AutofixOutput& operator=(const AutofixOutput& rhs);
    AutofixOutput& operator+=(const AutofixOutput& that);
    AutofixOutput& operator-=(const AutofixOutput& that);
    void update(CameraInfo::CameraInstance inst, float invFocalLm2, const PixelL0_i foeL0In);
    void update(CameraInfo::CameraInstance inst, float focalLm2, const float foeAngleIn[2]);

    PixelL0_i foeL0;
    float foeAngle[2];
  };

  AutofixOutput operator+(const AutofixOutput& a0, const AutofixOutput& a1);
  AutofixOutput operator-(const AutofixOutput& a0, const AutofixOutput& a1);

  struct AutofixLimits {
    PixelL0_i foeL0Init; // init calibration (TAC/SPC...) of yaw/hor in level 0 (this is relative to nominal center of image)
    PixelL0_i foeAutofixL0Init; // value of autofix yaw/hor read from etc/can on ignition in level 0 (this is relative to yawL0Init)

    PixelL0_i foeUpperLimitL0; // max value of yaw/hor (level 0) above which out of calibration failsafe is raised
    PixelL0_i foeLowerLimitL0; // min value of yaw/hor (level 0) below which out of calibration failsafe is raised
    bool foeOutOfLimit[2];

    float currIsOkThreshold; // MIN_AUTOFIX_LARGE_DELTA in AutoFix
    bool foeCurrIsOk[2]; // is the current value converging close enough to current calibration in FFS
    PixelL0_i foeAutofixDeltaL0LastUpdated; // current calibration in FFS
    int foeCounterOOL[2]; // delay counter for out of limit

    AutofixLimits() : currIsOkThreshold(2.5f) {
      for (int i=0; i<2; ++i) {
        foeL0Init[i] = 0;
        foeAutofixL0Init[i] = 0;
        foeUpperLimitL0[i] = 0;
        foeLowerLimitL0[i] = 0;
        foeOutOfLimit[i] = false;
        foeCurrIsOk[i] = false;
        foeAutofixDeltaL0LastUpdated[i] = 0;
        foeCounterOOL[i] = 0;
      }
    }
  };

  struct AutofixState {
    bool foeOutOfLimit[2];
    bool foeCurrIsOk[2]; // is the current value converging close enough to current calibration in FFS

    AutofixLimits limits[CameraInfo::e_NUM_OF_CAM_INSTANCES];
    AutofixOutput output[CameraInfo::e_NUM_OF_CAM_INSTANCES];
    AutofixOutput outputPrevConv[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  };
} // namespace WMBC

#endif // WMBC_AUTOFIX_TYPES_H_

