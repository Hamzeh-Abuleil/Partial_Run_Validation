/*
 * wmbcProperties.h
 *
 *  Created on: Jun 08, 2016
 *      Author: urilo
 */

#ifndef _WMBC_PROPERTIES_
#define _WMBC_PROPERTIES_

#include "functionality/calibration/Properties.h"
#include "functionality/calibration/SeriesProperties.h"
#include "technology/calibration/WMBC/common/slcTypes.h"

// namespace WMBC {

  class WmbcProperties : public Properties {
  public:
    enum RunningMode {
      e_Run_Autofix,
      e_Run_SPC,  // Service Point Calibration
      e_Run_SLC, // Stationless
      e_Run_Mode_Num
    };

    WmbcProperties();
    ~WmbcProperties() {}

    virtual void validateValues();

    RWProperT<bool> wmbcRun;
    RWProperT<int> wmbcMode;
    RWProperT<int> wmbcCalcSwitch;
    RWProperT<int> wmbcMinRadius;
    RWProperT<int> wmbcMinSpeed;
    RWProperT<int> wmbcMaxSpeed;
    RWProperT<float> wmbcMaxAccel;
    RWProperT<float> wmbcMaxYawRate;
    RWProperT<Series<float, 3> > wmbcMaxRotation;
    RWProperT<float> wmbcMaxPitchDiff;
    RWProperT<float> wmbcMaxCrownAngDiff;
    RWProperT<bool> wmbcUseSpeedHighPrecision;
    RWProperT<bool> wmbcUseDynamicSuspension;
    RWProperT<bool> wmbc8MP;
    // RWProperT<float> wmbcMaxSpeedPrecisionError;
    RWProperT<Series<float, 4> > wmbcConvMaxStd;
    RWProperT<int> wmbcConvSample;
    RWProperT<int> wmbcConvNightSnowSample;
    RWProperT<int> wmbcConvStable;
    RWProperT<Series<float, 4> > wmbcConvStableMaxDiff;

    RWProperT<bool>  wmbcUseSpecialLimits;
    RWProperT<float> wmbcMinYawAngle;
    RWProperT<float> wmbcMaxYawAngle;
    RWProperT<float> wmbcMinPitchAngle;
    RWProperT<float> wmbcMaxPitchAngle;
    RWProperT<float> wmbcMinRollAngle;
    RWProperT<float> wmbcMaxRollAngle;
    RWProperT<float> wmbcMinCamHeight;
    RWProperT<float> wmbcMaxCamHeight;

    RWProperT<float> wmbcMinQuality;
    RWProperT<float> wmbcTimeoutTime;
    RWProperT<float> wmbcTimeoutDist;
    RWProperT<int> wmbcTimeoutValidFrames;
    RWProperT<float> wmbcTimeoutTime_minSpeed;

    RWProperT<unsigned int> wmbcDebugPrint;

    RWProperT<bool> afixUseMovingSampleTh;
    RWProperT<Series<float, 4> > afixConvFoe;
    RWProperT<Series<float, 4> > afixConvRoll;
    RWProperT<Series<int, 4> > afixConvSample;
    RWProperT<float> afixFoeClippedJump;
    RWProperT<float> afixRollClippedJump;

    RWProperT<Series<float, 4> > afixfoeDeltaFS;
    RWProperT<int> afixConvSampleFS;
    RWProperT<int> afixConvStableFS;

    RWProperT<float> camHeightRangeSevere;

    //OBSOLETE
    //RWProperT<float> camHeightRangePlausible;

    RWProperT<int> spcMaxAttemps;

    RWProperT<float> slcTargetHeight;
    RWProperT<float> slcTargetWidth;
    RWProperT<Series<float, 2> > slcTargetRect;
    RWProperT<Series<int, 2> > slcTargetDim;
    RWProperT<Series<int, WMBC::SLC_MAX_TARGET_NUM> > slcBottomLeftSquare;
    RWProperT<float> slcPatternMatchingThreshold;
    RWProperT<int> slcShiftWinX;
    RWProperT<int> slcSearchWinRadX;
    RWProperT<int> slcSearchWinRadY;
    RWProperT<bool> slcUseCalibShiftWin;
    RWProperT<int> slcCamhConvSample;
    RWProperT<float> slcCamhMaxDistance;
    RWProperT<float> slcCamhMinDistance;
    RWProperT<float> slcCameraHeight;
    RWProperT<float> slcMaxTravelDistance;
    RWProperT<float> slcMaxDistance;
    RWProperT<float> slcMinDistance;
    RWProperT<float> slcVehicleRoll;
  };

  // } // namespace WMBC

#endif // _WMBC_PROPERTIES_
  
