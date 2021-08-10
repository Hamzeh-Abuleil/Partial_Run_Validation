/*
 * wmbcProperties.cpp
 *
 *  Created on: Jun 08, 2016
 *      Author: urilo
 */

#include "technology/calibration/WMBC/common/wmbcProperties.h"
#include "technology/calibration/WMBC/common/wmbcTypes.h"
// #include "technology/calibration/WMBC/common/slcTypes.h"
#include "functionality/calibration/PropertiesValidators.h"
// #include <string>

#define WMBC_SET_PARAM(name, desc, varType, valDef, valMin, valMax) name(_me, valDef, #name, desc, Property::DEFAULT_ME_FLAGS, nullptr, RangeValidator<varType>(valMin, valMax, this))
#define WMBC_SET_PARAM_B(name, desc, valDef) name(_me, valDef, #name, desc, Property::DEFAULT_ME_FLAGS, nullptr, TrueFalseValidator(this))
#define WMBC_SET_PARAM_B_NAME(name, fname, desc, valDef) name(_me, valDef, fname, desc, Property::DEFAULT_ME_FLAGS, nullptr, TrueFalseValidator(this))
#define WMBC_SET_PARAM_ARR(name, desc, varType, dim, valDef, valRange) name(_me, Series<varType, dim>(valDef), #name, desc, Property::DEFAULT_ME_FLAGS, nullptr, SeriesRangeValidator<varType, dim>(valRange, this, NULL, false))
#define WMBC_SET_PARAM_ARR_FS(name, desc, varType, dim, valDef, valRange) name(_me, Series<varType, dim>(valDef), #name, desc, Property::DEFAULT_ME_FLAGS, nullptr, SeriesRangeValidator<varType, dim>(valRange))

using namespace WMBC;
// namespace WMBC {

  static const float MAX_ROTATION_DEFAULT[]   = {0.004, 0.004, 0.004};
  static const float MAX_ROTATION_RANGE[]   = {0,0.2, 0,0.2, 0,0.2}; // [rad]
  static const float MAX_STD_DEFAULT[] = {2.0f, 2.0f, 2.5f, 0.06f}; // [deg, m]
  static const float MAX_STD_RANGE[] = {0.f, 20.f, 0.f, 20.f, 0.f, 20.f, 0.f, 5.f};
  static const float STABLE_MAX_MEDIAN_DIFF_DEFAULT[] = {0.2f, 0.2f, 0.3f, 0.01f};
  static const float STABLE_MAX_MEDIAN_DIFF_RANGE[] = {-1.f, 1.f, -1.f, 1.f, -1.f, 1.f, -1.f, 1.f};

  static const float AFIX_CONV_FOE_VAL_DEFAULT[] = {0.01f, 0.1f, 0.3f, 0.75f};
  static const float AFIX_CONV_FOE_VAL_RANGE[] = {0.01f,10.f, 0.01f,10.f, 0.01f,10.f, 0.01f,10.f};
  static const float AFIX_CONV_ROLL_VAL_DEFAULT[] = {0.01f, 0.2f, 1.f, 1.25f};
  static const float AFIX_CONV_ROLL_VAL_RANGE[] = {0.01f,10.f, 0.01f,10.f, 0.01f,10.f, 0.01f,10.f};
  static const int AFIX_CONV_SAMPLE_DEFAULT[] = {100, 100, 100, 50};
  static const int AFIX_CONV_SAMPLE_RANGE[] = {0,10000, 0,10000, 0,10000, 0,10000};

  static const float FS_DELTA_DEFAULT[] = {0.7, 0.7, 1, 2}; // yaw [deg], hor [deg], roll [deg], camH [m]
  static const float FS_DELTA_RANGE[] = {0,20, 0,20, 0,20, 0,10};

  static const float SLC_TARGET_RECT_DEFAULT[] = {0.1, 0.1};
  static const float SLC_TARGET_RECT_RANGE[] = {0.01, 1.0, 0.01, 1.0};
  static const int SLC_TARGET_DIM_DEFAULT[] = {14, 2};
  static const int SLC_TARGET_DIM_RANGE[] = {10, 25, 2, 5};
  static const int SLC_BOTTOM_LEFT_SQUARE_DEFAULT[] = {WHITE, WHITE};
  static const int SLC_BOTTOM_LEFT_SQUARE_RANGE[] = {WHITE, BLACK, WHITE, BLACK};

  WmbcProperties::WmbcProperties() :
    Properties("[WMBC_CONF]", Property::DEFAULT_FLAGS),
    WMBC_SET_PARAM_B_NAME(wmbcRun, "runWMBC", "runWMBC", false),
    WMBC_SET_PARAM(wmbcMode, "Running mode (SPC / Autofix / Stationless)", int, e_Run_Autofix, 0, e_Run_Mode_Num-1),
    WMBC_SET_PARAM(wmbcCalcSwitch, "Bit mask to switch on/off calc of foe/roll/camh", int, 7, 0, 7),

    WMBC_SET_PARAM(wmbcMinRadius, "WMBC min curvature radius [m]", int, 600, 200, 3000),
    WMBC_SET_PARAM(wmbcMinSpeed, "WMBC min speed [m/sec]", int, 5, 0, 50),
    WMBC_SET_PARAM(wmbcMaxSpeed, "WMBC max speed [m/sec]", int, 70, 1, 150),
    WMBC_SET_PARAM(wmbcMaxAccel, "WMBC max acceleration [m/sec^2]", float, 0.3f, 0.f, 50.f),
    WMBC_SET_PARAM(wmbcMaxYawRate, "WMBC max yawRate [rad/sec]", float, 1.5f*DEG2RAD, 0.f, 10.f*DEG2RAD),
    WMBC_SET_PARAM_ARR(wmbcMaxRotation, "WBMC max yaw,pitch,roll [rad/frame]", float, 3, MAX_ROTATION_DEFAULT, MAX_ROTATION_RANGE), // add section: for internal use only
    WMBC_SET_PARAM(wmbcMaxPitchDiff, "WMBC max allowed pitch discrepancy between em and rm [rad]", float, 1.2f*DEG2RAD, -1.f, 10.f*DEG2RAD),
    WMBC_SET_PARAM(wmbcMaxCrownAngDiff, "WMBC max allowed road-plane angle change across lateral pos (crown) [deg]", float, 2.f, -1.f, 10.f),
    WMBC_SET_PARAM_B(wmbcUseSpeedHighPrecision, "WMBC-Autofix use high precision speed signal for camera hight", false),
    WMBC_SET_PARAM_B(wmbcUseDynamicSuspension, "WMBC use dynamic camera height signal for reset", false),
    WMBC_SET_PARAM_B(wmbc8MP, "WMBC special origin calc for 8MP sensors", false),
    // WMBC_SET_PARAM(wmbcMaxSpeedPrecisionError, "WMBC max speed error compared to high precision signal [m/sec]", float, -1, -1, 100), // -1 for disabling
    WMBC_SET_PARAM_ARR_FS(wmbcConvMaxStd, "WBMC convergence max standard deviation [deg, m]", float, 4, MAX_STD_DEFAULT, MAX_STD_RANGE), // internal
    WMBC_SET_PARAM(wmbcConvSample, "WMBC convergence sample frame num", int, 100, 10, 10000), // internal
    WMBC_SET_PARAM(wmbcConvNightSnowSample, "WMBC convergence sample frame num", int, 200, 10, 10000), // internal  //TODO: CHANGE VALUE TO STICKEY
    WMBC_SET_PARAM(wmbcConvStable, "WMBC convergence stable frame num", int, 20, 5, 1000), // internal
    WMBC_SET_PARAM_ARR_FS(wmbcConvStableMaxDiff, "WMBC convergence stable frame factor", float, 4, STABLE_MAX_MEDIAN_DIFF_DEFAULT, STABLE_MAX_MEDIAN_DIFF_RANGE), // internal

    WMBC_SET_PARAM_B(wmbcUseSpecialLimits, "WMBC use own oor limits instead of system limits", false),
    WMBC_SET_PARAM(wmbcMinYawAngle, "", float, -5*DEG2RAD, -7*DEG2RAD, 7*DEG2RAD), // ask yossi, check how affect histogram
    WMBC_SET_PARAM(wmbcMaxYawAngle, "", float, 5*DEG2RAD, -7*DEG2RAD, 7*DEG2RAD),
    WMBC_SET_PARAM(wmbcMinPitchAngle, "", float, -5*DEG2RAD, -7*DEG2RAD, 7*DEG2RAD),
    WMBC_SET_PARAM(wmbcMaxPitchAngle, "", float, 5*DEG2RAD, -7*DEG2RAD, 7*DEG2RAD),
    WMBC_SET_PARAM(wmbcMinRollAngle, "", float, -3*DEG2RAD, -7*DEG2RAD, 7*DEG2RAD),
    WMBC_SET_PARAM(wmbcMaxRollAngle, "", float, 3*DEG2RAD, -7*DEG2RAD, 7*DEG2RAD),
    WMBC_SET_PARAM(wmbcMinCamHeight, "", float, 0.8, 0.f, 5.f),
    WMBC_SET_PARAM(wmbcMaxCamHeight, "", float, 1.8f, 0.f, 5.f),

    WMBC_SET_PARAM(wmbcMinQuality, "", float, 0.f, 0.f, 1.f),
    WMBC_SET_PARAM(wmbcTimeoutTime, "", float, -1.f, -1.f, 1200.f),
    WMBC_SET_PARAM(wmbcTimeoutDist, "", float, -1.f, -1.f, 10000.f),
    WMBC_SET_PARAM(wmbcTimeoutValidFrames, "", float, -1.f, -1.f, 100000.f), // ask eitan if can be deleted
    WMBC_SET_PARAM(wmbcTimeoutTime_minSpeed, "Min speed for timeout-time counter [m/sec]", float, 1.f, 0.f, 300.f*KPH2MPS),

    WMBC_SET_PARAM(wmbcDebugPrint, "Use debug prints", unsigned int, 0, 0, 65535), // internal

    WMBC_SET_PARAM_B(afixUseMovingSampleTh, "WMBC-Autofix use moving sample threshold for convergence", true),
    WMBC_SET_PARAM_ARR_FS(afixConvFoe, "WMBC-Autofix convergence marks yaw/horizon correction in degrees", float, 4, AFIX_CONV_FOE_VAL_DEFAULT, AFIX_CONV_FOE_VAL_RANGE),
    WMBC_SET_PARAM_ARR_FS(afixConvRoll, "WMBC-Autofix convergence marks roll correction in degrees", float, 4, AFIX_CONV_ROLL_VAL_DEFAULT, AFIX_CONV_ROLL_VAL_RANGE),
    WMBC_SET_PARAM_ARR_FS(afixConvSample, "WMBC-Autofix convergence frame num threshold for mark correction", int, 4, AFIX_CONV_SAMPLE_DEFAULT, AFIX_CONV_SAMPLE_RANGE),
    WMBC_SET_PARAM(afixFoeClippedJump, "WMBC-Autofix maximum value of yaw/horizon which can be set in a single convergence in degrees", float, 1.5f /*0.2f*/, -1.f, 10.f),
    WMBC_SET_PARAM(afixRollClippedJump, "WMBC-Autofix maximum value of roll which can be set in a single convergence in degrees", float, -1.f, -1.f, 10.f),

    WMBC_SET_PARAM_ARR_FS(afixfoeDeltaFS, "WMBC-Autofix delta threshold for failsafe", float, 4, FS_DELTA_DEFAULT, FS_DELTA_RANGE),
    WMBC_SET_PARAM(afixConvSampleFS, "WMBC-Autofix convergence sample frame num for failsafe", int, 100, 10, 10000), 
    WMBC_SET_PARAM(afixConvStableFS, "WMBC-Autofix convergence stable frame num for failsafe", int, 10, 5, 1000), 

    WMBC_SET_PARAM(camHeightRangeSevere, "WMBC-Autofix camera height permitted range for safety, represents percentages of deviation from the base", float, 0.06, 0, 1),

    //OBSOLETE
    //WMBC_SET_PARAM(camHeightRangePlausible, "WMBC-Autofix camera height permitted range for safety, represents percentages of deviation from the base", float, 0.04, 0, 1),

    WMBC_SET_PARAM(spcMaxAttemps, "WMBC-SPC max session attempts before timeout", int, 5, 0, 100),

    WMBC_SET_PARAM(slcTargetHeight, "SLC height of lowest saddle point from the floor", float, 0.9f, 0.5, 3.0), 
    WMBC_SET_PARAM(slcTargetWidth, "SLC distance between target centers", float, 0, 0, 5.0), 
    WMBC_SET_PARAM_ARR(slcTargetRect, "SLC single rectangle dims: width x height", float, 2, SLC_TARGET_RECT_DEFAULT, SLC_TARGET_RECT_RANGE),
    WMBC_SET_PARAM_ARR_FS(slcTargetDim, "SLC target dimension: rows x columns", int, 2, SLC_TARGET_DIM_DEFAULT, SLC_TARGET_DIM_RANGE),
    WMBC_SET_PARAM_ARR(slcBottomLeftSquare, "SLC color of bottom left square for each target. 0 = white, 1 = black", int, SLC_MAX_TARGET_NUM, SLC_BOTTOM_LEFT_SQUARE_DEFAULT, SLC_BOTTOM_LEFT_SQUARE_RANGE),
    WMBC_SET_PARAM(slcPatternMatchingThreshold, "SLC threshold of saddle point pattern matching", float, 0.6f, 0.f, 1.0f),
    WMBC_SET_PARAM(slcShiftWinX, "", int, 0, -640, 640),
    WMBC_SET_PARAM(slcSearchWinRadX, "", int, 50, 0, 320),
    WMBC_SET_PARAM(slcSearchWinRadY, "", int, 110, 0, 480),
    WMBC_SET_PARAM_B(slcUseCalibShiftWin, "", true),
    WMBC_SET_PARAM(slcCamhConvSample, "SLC convergence sample frame num for camera height", int, 8, 0, 100),
    WMBC_SET_PARAM(slcCamhMaxDistance, "SLC max distance from targets where camera height calc is done", float, 10.f, 0.5f, 30.f),
    WMBC_SET_PARAM(slcCamhMinDistance, "SLC min distance from targets where camera height calc is done", float, 6.f, 0.5f, 30.f),
    WMBC_SET_PARAM(slcCameraHeight, "SLC camera height input guess [m]", float, 1.54f, 0.f, 5.f), 
    WMBC_SET_PARAM(slcMaxTravelDistance, "SLC max travel distance for timeout", float, -1.f, -1.f, 10000.f),
    WMBC_SET_PARAM(slcMaxDistance, "SLC max distance from targets (start distance)", float, 15.f, 0.5f, 100.f),
    WMBC_SET_PARAM(slcMinDistance, "SLC min distance from targets (stop distance)", float, 3.5f, 0.5f, 100.f),
    WMBC_SET_PARAM(slcVehicleRoll, "SLC vehicle roll [rad]", float, 0.f, -10*DEG2RAD, 10*DEG2RAD)
  {}

void WmbcProperties::validateValues() {
  if (wmbcMaxSpeed() < wmbcMinSpeed()) {
    std::stringstream stream;
    stream << wmbcMinSpeed();

    MEtl::string keyval = stream.str().c_str();
    MEtl::string keyname("wmbcMinSpeed");
    MEtl::string whyNot("min speed is greater than max speed");
    setRejected(keyname, keyval, *this, whyNot);

    stream.str("");
    stream.clear();

    stream << wmbcMaxSpeed();
    keyval = stream.str().c_str();
    keyname = "wmbcMaxSpeed";
    Properties::setRejected(keyname, keyval, *this, whyNot);
  }
}

// } // namespace WMBC
