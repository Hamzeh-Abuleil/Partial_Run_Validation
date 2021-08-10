/*
 * wmbcIF.h
 *
 *  Created on: Jun 08, 2016
 *      Author: urilo
 */

#ifndef WMBC_IF_H_
#define WMBC_IF_H_

#include "technology/calibration/WMBC/common/wmbcTypes.h"
#include "technology/calibration/WMBC/common/slcTypes.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "technology/calibration/utilities/cameraProjections/distortionCorrection_algoAPI.h"

// DO NOT REMOVE OR DELETE THIS COMMENT
// PARTIAL RUN IF-START
struct WmbcIF {
  WmbcIF() : wmFoeReset(WMBC::e_WMRESET_NONE), status(WMBC::e_CALIB_CAL), pauseReason(0), progress(0), runMode(WMBC::e_Autofix), quality(0),
             confidence(0.f), confidenceGrade(0), totalDistance(0.f), validDistance(0.f), totalTime(0.f), validTime(0.f), validAlgoFrameNum(0),
             validVehicleFrameNum(0), slcValidAlgoFrameNum(0), slcTarget(), debugShowData(), convergedNum(0), lastConvFrame(-1),
             yawLm2Primary_f(0.f), horizonLm2Primary_f(0.f), yawDeltaLm2FromeEtcPrimary_f(0.f), horizonDeltaLm2FromeEtcPrimary_f(0.f),
             rollPrimary(0), camhPrimary(0)
  {
    for (int i = 0; i < CameraInfo::e_NUM_OF_CAM_INSTANCES; ++i) {
      CameraInfo::CameraInstance inst = (CameraInfo::CameraInstance)i;
      autoFix_yaw[i] = 0;
      autoFix_horizon[i] = 0;
      autoFixOnlineLm2_yaw[i] = 0;
      autoFixOnlineLm2_horizon[i] = 0;

      yawDeltaL0FromEtc[i] = 0;
      horizonDeltaL0FromEtc[i] = 0;
      yawDeltaL0FromLastConv[i] = 0;
      horizonDeltaL0FromLastConv[i] = 0;
      yawDeltaLm2FromEtc_f[i] = 0.f;
      horizonDeltaLm2FromEtc_f[i] = 0.f;

      yawLm2[i] = 0;
      horizonLm2[i] = 0;

      yawL0[i] = 0;
      horizonL0[i] = 0;
      roll[i] = 0;
      cameraHeight[i] = 0;

      yawAngle[i] = 0.f;
      pitchAngle[i] = 0.f;

      interimYawLm2[i] = 0;
      interimHorizonLm2[i] = 0;
      interimYawAngle[i] = 0.f;
      interimPitchAngle[i] = 0.f;
      interimRoll[i] = 0.f;
      interimCameraHeight[i] = 0.f;
      interimYawDeltaL0FromEtc[i] = 0;
      interimHorizonDeltaL0FromEtc[i] = 0;
      interimYawDeltaL0FromLastConv[i] = 0;
      interimHorizonDeltaL0FromLastConv[i] = 0;
      interimYawDeltaLm2FromEtc_f[i] = 0.f;
      interimHorizonDeltaLm2FromEtc_f[i] = 0.f;

      interimAutoFixOnlineLm2_yaw[i] = 0;
      interimAutoFixOnlineLm2_horizon[i] = 0;

      if (CameraInfo::exists(inst)) {
        autoFix_yaw[i] = CameraInfo::initialAutoFix_yaw(inst);
        autoFix_horizon[i] = CameraInfo::initialAutoFix_horizon(inst);
        yawLm2[i] = CameraInfo::yawFull(inst) + autoFix_yaw[i]*4;
        horizonLm2[i] = CameraInfo::horizonFull(inst) + autoFix_horizon[i]*4;
        yawL0[i] = (yawLm2[i])>>2;
        horizonL0[i] = (horizonLm2[i])>>2;
        roll[i] = CameraInfo::rollAngle(inst);
        cameraHeight[i] = CameraInfo::cameraHeight(inst);

        float focal = 4.f*CameraInfo::focalLength(inst);
        bool ok = (DistortionCorrectionAPI::isDistortionValid(inst) && focal > 0.1);
        if (ok) {
          ok = DistortionCorrectionAPI::rectifySafe(CameraInfo::e_FORWARD, -2, 0.f, 0.f, yawAngle[i], pitchAngle[i]);
          yawAngle[i] /= focal;
          pitchAngle[i] /= focal;
        }
      }
    }

    for (int i = WMBC::e_YAW; i < WMBC::e_CALIB_DOF_NUM; ++i) {
      singleStatus[i] = WMBC::e_CORE_INIT;
      singleError[i] = WMBC::e_CALIB_GENERAL;
      singleProgress[i] = 0;
      singleConverged[i] = false;
      singleConvFailReason[i] = 0;
      singleQuality[i] = 0;
      singleConfidence[i] = 0.f;
    }
  }

  // For external use
  short yawL0[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0]
  short horizonL0[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0]
  float roll[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [rad]
  float cameraHeight[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [m]

  short yawDeltaL0FromEtc[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to etc
  short horizonDeltaL0FromEtc[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to etc
  float yawDeltaLm2FromEtc_f[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  float horizonDeltaLm2FromEtc_f[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  short yawDeltaL0FromLastConv[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to previous convergence
  short horizonDeltaL0FromLastConv[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to previous convergence
  float interimYawDeltaLm2FromEtc_f[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  float interimHorizonDeltaLm2FromEtc_f[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  
  short yawLm2[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px lm2]
  short horizonLm2[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px lm2]
  float yawAngle[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [rad] (corresponds to rectified im)
  float pitchAngle[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [rad] (corresponds to rectified im)

  int autoFix_yaw[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level0] the value of autofix_yaw in init (etc)
  int autoFix_horizon[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level0] the value of autofix_horizon in init (etc)
  int autoFixOnlineLm2_yaw[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level -2] new "autoFix" value - new foe_x relative to static calibration (yawFull)
  int autoFixOnlineLm2_horizon[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level -2] new "autoFix" values - new foe_y relative to static calibration (horizonFull)

  // temp values which upadate regardless of convergence - for persistent clients
  short interimYawLm2[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  short interimHorizonLm2[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  float interimYawAngle[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  float interimPitchAngle[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  float interimRoll[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  float interimCameraHeight[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  short interimYawDeltaL0FromEtc[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to etc
  short interimHorizonDeltaL0FromEtc[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to etc
  short interimYawDeltaL0FromLastConv[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to previous convergence
  short interimHorizonDeltaL0FromLastConv[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to previous convergence

  int interimAutoFixOnlineLm2_yaw[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level -2] new "autoFix" value - new foe_x relative to static calibration (yawFull)
  int interimAutoFixOnlineLm2_horizon[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level -2] new "autoFix" values - new foe_y relative to static calibration (horizonFull)

  // for ego motion internal use
  WMBC::WM_ResetSignal wmFoeReset; // signal to worldmodel to take recent foe
  WMBC::VehicleToCam vehicleToCam[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  WMBC::VehicleToCam interimVehicleToCam[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  WMBC::VehicleToCam vehicleToCamWmReset;

  // meta public
  WMBC::CalibStatus status;
  WMBC::CalibCoreStatus singleStatus[WMBC::e_CALIB_DOF_NUM];
  WMBC::CalibStatus singleError[WMBC::e_CALIB_DOF_NUM];
  int pauseReason; // 12 bit mask
  int singleProgress[WMBC::e_CALIB_DOF_NUM]; // independed progress for each value, order: yaw, horizon, roll, camh
  int progress; // total progress
  bool singleConverged[WMBC::e_CALIB_DOF_NUM]; // independed convergence of each value, order: yaw, horizon, roll, camh
  int singleConvFailReason[WMBC::e_CALIB_DOF_NUM]; // 5bit mask of convergence test (1 passed, 0 failed): variance, distribution symmetry, sample size, stable median, quality
  WMBC::CalibMode runMode; // WMBC flavour - SPC/autoFix/stationless

  int singleQuality[WMBC::e_CALIB_DOF_NUM];
  int quality;
  float singleConfidence[WMBC::e_CALIB_DOF_NUM];
  float confidence;
  int confidenceGrade;
  float totalDistance;
  float validDistance;
  float totalTime;
  float validTime;
  int validAlgoFrameNum;
  int validVehicleFrameNum;
  int slcValidAlgoFrameNum;
  WMBC::TargetDrawInfo slcTarget;

  // meta internal
  WMBC::DebugShowData debugShowData; 
  int convergedNum; // how many times already converged (autofix quick mode for failsafe)
  int lastConvFrame; // global frame index of last convergence

  // raw data - candidates for deletion
  float yawLm2Primary_f; // [px level -2] primary cam, on distorted (without rounding)
  float horizonLm2Primary_f; // [px level -2] primary cam, on distorted (without rounding)
  float yawDeltaLm2FromeEtcPrimary_f; // [px lm2] primary cam, on distorted, relative to etc (without rounding)
  float horizonDeltaLm2FromeEtcPrimary_f; // [px lm2] primary cam, on distorted, relative to etc (without rounding)
  float rollPrimary; // [rad] primary cam
  float camhPrimary; // [m]
};
// PARTIAL RUN IF-END
// DO NOT REMOVE OR DELETE THIS COMMENT
struct EWmbcIF {
  EWmbcIF() {
    std::memset(this, 0, sizeof(*this));
    available = false;
    for (int i = 0; i < CameraInfo::e_NUM_OF_CAM_INSTANCES; ++i) {
      yawL0[i] = 0;
      horizonL0[i] = 0;
      roll[i] = 0.f;
      cameraHeight[i] = 0.f;
      yawLm2[i] = 0;
      horizonLm2[i] = 0;
      yawAngle[i] = 0.f;
      pitchAngle[i] = 0.f;

      autoFix_yaw[i] = 0;
      autoFix_horizon[i] = 0;

      autoFix_yaw[i] = 0;
      autoFix_horizon[i] = 0;
      autoFixOnlineLm2_yaw[i] = 0;
      autoFixOnlineLm2_horizon[i] = 0;

      yawDeltaL0FromEtc[i] = 0;
      horizonDeltaL0FromEtc[i] = 0;
      yawDeltaL0FromLastConv[i] = 0;
      horizonDeltaL0FromLastConv[i] = 0;

      interimYawLm2[i] = 0;
      interimHorizonLm2[i] = 0;
      interimYawAngle[i] = 0.f;
      interimPitchAngle[i] = 0.f;
      interimRoll[i] = 0.f;
      interimCameraHeight[i] = 0.f;
      interimYawDeltaL0FromEtc[i] = 0;
      interimHorizonDeltaL0FromEtc[i] = 0;
      interimYawDeltaL0FromLastConv[i] = 0;
      interimHorizonDeltaL0FromLastConv[i] = 0;

      interimAutoFixOnlineLm2_yaw[i] = 0;
      interimAutoFixOnlineLm2_horizon[i] = 0;
    }
    yawDeltaLm2FromeEtcPrimary_f = 0.f;
    horizonDeltaLm2FromeEtcPrimary_f = 0.f;
    for (int i = WMBC::e_YAW; i < WMBC::e_CALIB_DOF_NUM; ++i) {
      singleStatus[i] = WMBC::e_CORE_INIT;
      singleError[i] = WMBC::e_CALIB_GENERAL;
      singleConverged[i] = false;
      singleProgress[i] = 0;
      singleQuality[i] = 0;
      singleConvFailReason[i] = 0;
    }
    status = WMBC::e_CALIB_CAL;
    pauseReason = 0;
    progress = 0;
    runMode = WMBC::e_Autofix;
    quality = 0;
    totalDistance = 0.f;
    validDistance = 0.f;
    totalTime = 0.f;
    validTime = 0.f;
    validAlgoFrameNum = 0;
    validVehicleFrameNum = 0;
    slcValidAlgoFrameNum = 0;
  }

  bool available;
  short yawL0[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px l0]
  short horizonL0[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px l0]
  float roll[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [rad]
  float cameraHeight[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [m]

  short yawDeltaL0FromEtc[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to etc
  short horizonDeltaL0FromEtc[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to etc
  short yawDeltaL0FromLastConv[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to previous convergence
  short horizonDeltaL0FromLastConv[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to previous convergence

  short yawLm2[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px lm2]
  short horizonLm2[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px lm2]
  float yawAngle[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [rad] (corresponds to rectified im)
  float pitchAngle[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [rad]

  int autoFix_yaw[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level0] the value of autofix_yaw in init (etc)
  int autoFix_horizon[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level0] the value of autofix_horizon in init (etc)
  int autoFixOnlineLm2_yaw[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level -2] new "autoFix" value - new foe_x relative to static calibration (yawFull)
  int autoFixOnlineLm2_horizon[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level -2] new "autoFix" values - new foe_y relative to static calibration (horizonFull)

  short interimYawLm2[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px lm2]
  short interimHorizonLm2[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px lm2]
  float interimYawAngle[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [rad]
  float interimPitchAngle[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [rad]
  float interimRoll[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [rad]
  float interimCameraHeight[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [m]
  float yawDeltaLm2FromeEtcPrimary_f; // [px lm2] primary cam, on distorted, relative to etc (without rounding)
  float horizonDeltaLm2FromeEtcPrimary_f; // [px lm2] primary cam, on distorted, relative to etc (without rounding)
  short interimYawDeltaL0FromEtc[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to etc
  short interimHorizonDeltaL0FromEtc[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to etc
  short interimYawDeltaL0FromLastConv[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to previous convergence
  short interimHorizonDeltaL0FromLastConv[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level 0] relative to previous convergence

  int interimAutoFixOnlineLm2_yaw[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level -2] new "autoFix" value - new foe_x relative to static calibration (yawFull)
  int interimAutoFixOnlineLm2_horizon[CameraInfo::e_NUM_OF_CAM_INSTANCES]; // [px level -2] new "autoFix" values - new foe_y relative to static calibration (horizonFull)

  WMBC::CalibStatus status;
  WMBC::CalibCoreStatus singleStatus[WMBC::e_CALIB_DOF_NUM];
  WMBC::CalibStatus singleError[WMBC::e_CALIB_DOF_NUM];
  int pauseReason;
  int singleProgress[WMBC::e_CALIB_DOF_NUM];
  int progress;
  bool singleConverged[WMBC::e_CALIB_DOF_NUM];
  int singleConvFailReason[WMBC::e_CALIB_DOF_NUM];
  WMBC::CalibMode runMode;
  int convergedNum;
  int singleQuality[WMBC::e_CALIB_DOF_NUM];
  int quality;
  float totalDistance;
  float validDistance;
  float totalTime;
  float validTime;
  int validAlgoFrameNum;
  int validVehicleFrameNum;
  int slcValidAlgoFrameNum;
  WMBC::TargetDrawInfo slcTarget;
};

#endif // WMBC_IF_H_

