#include "technology/calibration/WMBC/common/wmbcOutputIF.h"
#include "basicTypes/mfl/common/itrkWriter.h"
#include "basicTypes/mfl/itrkWriter_API.h"
#include "technology/calibration/WMBC/interface/wmbcIF.h"
#include "technology/calibration/WMBC/common/wmbc.h"
#include "technology/calibration/WMBC/common/wmbcTypes.h"
#include "technology/calibration/WMBC/common/wmbcUtils.h"
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "technology/calibration/utilities/cameraProjections/distortionCorrection_algoAPI.h"

#ifdef MEwin
#include "utilities/clipextIO/clipextIO.h"
#endif

namespace WMBC {

  WmbcOutputIF& WmbcOutputIF::instance() {
    static WmbcOutputIF& single = *new WmbcOutputIF;
    return single;
  }

  WmbcOutputIF::WmbcOutputIF() {
    for (int i = 0; i < WmbcItrkType::e_ITRK_NUM_TYPES; ++i) { // TODO: types
      _itrkHandle[i] = nullptr;
    }
    _camInst = CameraInfo::e_FORWARD;
  }

  void WmbcOutputIF::toItrkHeaders() {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    if (_itrkHandle[WmbcItrkType::e_FINAL]) {
      return;
    }

    _itrkHandle[WmbcItrkType::e_INPUT_DATA] = itrkWriter::registerItrkFormat("WMBC InputData focal doX doY roX roY ndoX ndoY tx ty tz dyaw dpitch droll emYaw emPitch "
                                                                             "foeRectX foeRectY foeDistX foeDistY emValid emStatus emConf emConfT emConfR emValidFrames rmsValid "
                                                                             "Nx Ny Nz d roll rmPitch rmValid rmStatus rmNumOfInliers rmPlaneIdx rmMessage "
                                                                             "crZ crX0 crX1 crX2 crX3 crX4 crA0 crA1 crA2 crA3 crA4 crAstd0 crAstd1 crAstd2 crAstd3 "
                                                                             "crAstd4 crADiff crADiff1 crdDiff1 pitchDiff radius speed yawRate dt acceleration reverseGear "
                                                                             "vehicleKneeling speedHighPrecisionValid speedHighPrecision speedPrecisionError "
                                                                             "dynamicCameraHeightValid dynamicCameraHeight dynamicCameraHeightDeltaValid dynamicCameraHeightDelta "
                                                                             "dynamicCameraHeightChangeActiveValid dynamicCameraHeightChangeActive dynamicCameraHeightDeltaAvailable "
                                                                             "accumulatedDistance quickMode");
    _itrkHandle[WmbcItrkType::e_INPUT_DATA_CONF] = itrkWriter::registerItrkFormat("WMBC InputDataConf rmsPts rmsInliers epiValid epiInvalidPercent epiQualityFrameNum epiQuality epiErr epiErrAng "
                                                                                  "epiCovXX epiCovYY epiCovXY foeRectLsX foeRectLsY epiYawAngle epiPitchAngle "
                                                                                  "resConfMeanPitch resConfMeanStringentPitch residualPtNumPitch residualMeanPitch "
                                                                                  "residualVarPitch residualMinPitch residualMaxPitch resConfMeanRoll resConfMeanStringentRoll "
                                                                                  "residualPtNumRoll residualMeanRoll residualVarRoll residualMinRoll residualMaxRoll "
                                                                                  "resConfMeanCamh resConfMeanStringentCamh residualPtNumCamh residualMeanCamh residualVarCamh "
                                                                                  "residualMinCamh residualMaxCamh rmAtA00 rmAtA01 rmAtA02 rmAtA11 rmAtA12 rmAtA22 "
                                                                                  "rmNAtA00 rmNAtA01 rmNAtA02 rmNAtA11 rmNAtA12 rmNAtA22 rmResidual rmCovValid "
                                                                                  "rmCov00 rmCov01 rmCov02 rmCov11 rmCov12 rmCov22 snow night snowNightMode nonSnowNightCounter");
    _itrkHandle[WmbcItrkType::e_FRAME_VALIDATION] = itrkWriter::registerItrkFormat("WMBC FrameValidation validFrame validFrameYaw validFrameHorizon validFrameRoll validFrameCamh "
                                                                                   "validFrameYawNum validFrameHorizonNum validFrameRollNum validFrameCamhNum "
                                                                                   "speedOk drivingOk radiusOk accelOk yawRateOk dynamicSuspensionOk straightDrive pitchdiffOk " 
                                                                                   "crownOk speedPrecOk pauseReason delayValidCounter "
                                                                                   "minSpeed maxSpeed minRadius thDyaw thDpitch thDroll maxAccel maxYawRate maxPitchDiff " 
                                                                                   "maxCrownAngDiff useSpeedHighPrecision useDynamicSuspension delayValidThreshold "
                                                                                   "emValidStraightFrameNum emEpiQualityFrameNum crADiffMean crADiffVar crADiffMin crADiffMax quickMode");
    _itrkHandle[WmbcItrkType::e_FOE] = itrkWriter::registerItrkFormat("WMBC Foe isDistortionValid yawLm2Rect horLm2Rect yawLm2Dist horLm2Dist "
                                                                      "yawFrameLm2Rect horFrameLm2Rect yawFrameLm2Dist horFrameLm2Dist quickMode");
    _itrkHandle[WmbcItrkType::e_ROLL] = itrkWriter::registerItrkFormat("WMBC Roll frameRoll medianRoll frameCamh medianCamh quickMode");
    _itrkHandle[WmbcItrkType::e_CALIB_VAR] = itrkWriter::registerItrkFormat("WMBC CalibVariable id status sf median var validFrame validFrameNum validDistance validTime "
                                                                            "invalidFrameMask conv convNum convLastFrame convMovingSampleNumThreshold progress "
                                                                            "conf0 conf1 conf2 nonConvReason stableMedianCount maxMedianDiff stableMedianNumThreshold " 
                                                                            "maxVar minSample inRange emLastValidFrameNum emLastEpiQualityFrameNum crownQuality quickMode");
    _itrkHandle[WmbcItrkType::e_HIST] = itrkWriter::registerItrkFormat("WMBC Histogram id mean var median prevMedian min max minBin maxBin currValExact currVal currBin sampleNum binSize " 
                                                                       "meanDecay varDecay decayFac startDecay iqr valid autocov autocorr");
    _itrkHandle[WmbcItrkType::e_PLANE] = itrkWriter::registerItrkFormat("WMBC Plane valid roll camh location startFrame endFrame "
                                                                        "rmsPts epiErrAng epiCovXX epiCovYY rmResidual residualMeanPitch residualPtNumPitch residualMeanRoll residualPtNumRoll "
                                                                        "residualMeanCamh residualPtNumCamh rmValidCov rmCov00 rmCov11 rmCov22 pitchDiff crADiff");
    _itrkHandle[WmbcItrkType::e_PLANE_BUFFER] = itrkWriter::registerItrkFormat("WMBC PlaneBuffer bufferSize d1 d2 d3 d4 d5 d6 d7 d8 loc1 loc2 loc3 loc4 loc5 loc6 "
                                                                               "loc7 loc8 f1 f2 f3 f4 f5 f6 f7 f8");
    _itrkHandle[WmbcItrkType::e_CONVERGENCE] = itrkWriter::registerItrkFormat("WMBC Convergence calibInd conv totalConv smallVar isGauss largeSample "
                                                                              "medianStable highQuality stableMedianCount quality var mean median prevMedian iqr sampleNum binSize "
                                                                              "meanDecay varDecay decayFac startDecay frameVal quickMode");
    _itrkHandle[WmbcItrkType::e_SLC_TARGETS] = itrkWriter::registerItrkFormat("WMBC SlcTargets valid spNum xBL yBL xTL yTL xBR yBR xTR yTR imHeight imWidth distToTargets " 
                                                                              "height witdh rectW rectH rows cols colorBLL colorBLR patternMatchingThreshold " 
                                                                              "xSearch0 xSearch1 xSearch2 xSearch3 ySearch0 ySearch1 ySearch2 ySearch3 targetsLevel");
    _itrkHandle[WmbcItrkType::e_SLC_CAMH] = itrkWriter::registerItrkFormat("WMBC SlcCamHeight dataFrame valid chosen camh camhErr invImageTargetWidth distance "
                                                                           "targetCenterX targetCenterY foerX foerY camhErrMean restConverged isAccumulate isCalc "
                                                                           "camDataSize minValidFramesCamh");
    _itrkHandle[WmbcItrkType::e_SLC_CONVERGENCE] = itrkWriter::registerItrkFormat("WMBC SlcConvergence totalConv isEndTraj accumulatedDistance validFramesNum minValidFrames maxRunFrames "
                                                                                  "maxTravelDistance maxDistance minDistance camhMaxDistance "
                                                                                  "totalProgress progressYaw progressHorizon progressRoll progressCamh "
                                                                                  "totalQuality qualityYaw qualityHorizon qualityRoll qualityCamh");
    _itrkHandle[WmbcItrkType::e_DEBUG] = itrkWriter::registerItrkFormat("WMBC Debug yawAngleDeg pitchAngleDeg rollAngleDeg cameraHeight "
                                                                        "yawLm2Primary_f horizonLm2Primary_f yawDeltaLm2Primary_f horizonDeltaLm2Primary_f runMode status "
                                                                        "convYaw convHorizon convRoll convCamh totalConv totalConvNum totalConvLastFrame "
                                                                        "qualityYaw qualityHorizon qualityRoll qualityCamh totalQuality "
                                                                        "confYaw0 confYaw1 confYaw2 confHorizon0 confHorizon1 confHorizon2 "
                                                                        "confRoll0 confRoll1 confRoll2 confCamh0 confCamh1 confCamh2 "
                                                                        "conf0 conf1 conf2 totalConfidence totalConfGrade "
                                                                        "progressYaw progressHorizon progressRoll progressCamh totalProgress "
                                                                        "validFrameYaw validFrameHorizon validFrameRoll validFrameCamh "
                                                                        "validFrameYawNum validFrameHorizonNum validFrameRollNum validFrameCamhNum "
                                                                        "focalLm2 radius speed yawRate dt acceleration distance "
                                                                        "validYawDist validHorizonDist validRollDist validCamhDist "
                                                                        "emStatus rmStatus emConf emValidFrames distortionValid wmFoeReset resetMask " 
                                                                        "yawDeltaAngDegWmR pitchDeltaAngDegWmR rollDeltaAngDegWmR camhDeltaWmR quickMode");
    _itrkHandle[WmbcItrkType::e_FINAL] = itrkWriter::registerItrkFormat("WMBC Final yawL0 horizonL0 roll cameraHeight yawDeltaL0FromEtc horizonDeltaL0FromEtc "
"yawDeltaLm2FromEtc_f horizonDeltaLm2FromEtc_f yawDeltaL0FromLastConv horizonDeltaL0FromLastConv "
"interimYawDeltaLm2FromEtc_f interimHorizonDeltaLm2FromEtc_f yawLm2 horizonLm2 yawAngle pitchAngle "
"autoFix_yaw autoFix_horizon autoFixOnlineLm2_yaw autoFixOnlineLm2_horizon interimYawLm2 interimHorizonLm2 "
"interimYawAngle interimPitchAngle interimRoll interimCameraHeight interimYawDeltaL0FromEtc "
"interimHorizonDeltaL0FromEtc interimYawDeltaL0FromLastConv interimHorizonDeltaL0FromLastConv "
"interimAutoFixOnlineLm2_yaw interimAutoFixOnlineLm2_horizon wmFoeReset status singleStatus_Yaw "
"singleStatus_Pitch singleStatus_Roll singleStatus_Camh singleError_Yaw singleError_Pitch "
"singleError_Roll singleError_Camh pauseReason singleProgess_Yaw singleProgress_Pitch "
"singleProgress_Roll singleProgress_Camh progress singleConverged_Yaw singleConverged_Pitch "
"singleConverged_Roll singleConverged_Camh singleConvFailReason_Yaw singleConvFailReason_Pitch "
"singleConvFailReason_Roll singleConvFailReason_Camh runMode singleQuality_Yaw singleQuality_Pitch "
"singleQuality_Roll singleQuality_Camh quality singleConfidence_Yaw singleConfidence_Pitch "
"singleConfidence_Roll singleConfidence_Camh confidence confidenceGrade "
"totalDistance validDistance totalTime validTime validAlgoFrameNum validVehicleFrameNum slcValidAlgoFrameNum "
"convergedNum lastConvFrame quickMode");
    _itrkHandle[WmbcItrkType::e_FPA] = itrkWriter::registerItrkFormat("WMBC OutputValidation yawAngleDeg pitchAngleDeg rollAngleDeg cameraHeight yawLm2_f horizonLm2_f "
                                                                      "convYaw convHorizon convRoll convCamh totalConv focalLm2");
_itrkHandle[WmbcItrkType::e_AFIX] = itrkWriter::registerItrkFormat("WMBC Autofix yawL0Init horizonL0Init yawAutofixL0Init horizonAutofixL0Init "
                                                                       "yawUpperLimitL0 yawLowerLimitL0 horizonUpperLimitL0 horizonLowerLimitL0 yawOutOfLimit "
                                                                       "horizonOutOfLimit currIsOkThreshold yawCurrIsOk horizonCurrIsOk "
                                                                       "yawAutofixDeltaL0LastUpdated horizonAutofixDeltaL0LastUpdated yawCounterOOL "
                                                                       "horizonCounterOOL yawL0 horizonL0 yawAngle pitchAngle yawL0_prev horizonL0_prev "
                                                                       "yawAngle_prev pitchAngle_prev convYaw convLastFrameYaw convHorizon convLastFrameHorizon "
                                                                       "yawAngleRaw pitchAngleRaw newYaw newHorizon");
  }

  //void WmbcOutputIF::toItrkInputData(const WorldModelData *wm, const VehicleData *vh, const OriginData *od, bool quickMode) const {
  void WmbcOutputIF::toItrkInputData(const WmbcData *d) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    const float yawLm2Rect = d->em.yaw * d->camera.focalLm2;
    const float horLm2Rect = d->em.pitch * d-> camera.focalLm2;
    float yawLm2Dist = yawLm2Rect;
    float horLm2Dist = horLm2Rect;

    if (DistortionCorrectionAPI::isDistortionValid()) {
      DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, yawLm2Rect, horLm2Rect, yawLm2Dist, horLm2Dist);
    }

    itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_INPUT_DATA],
                            /* WMBC InputData                          */
                            /* 04 camPort                              */ _camInst,
                            /* 05 focal                                */ d->camera.focalLm2,
                            /* 06 doX                                  */ d->camera.origin[_camInst].distLm2.X(),
                            /* 07 doY                                  */ d->camera.origin[_camInst].distLm2.Y(),
                            /* 08 roX                                  */ d->camera.origin[_camInst].rectLm2.X(),
                            /* 09 roY                                  */ d->camera.origin[_camInst].rectLm2.Y(),
                            /* 10 ndoX                                 */ d->camera.origin[_camInst].distNominalLm2.X(),
                            /* 11 ndoY                                 */ d->camera.origin[_camInst].distNominalLm2.Y(),
                            /* 12 tx                                   */ d->em.t[0],
                            /* 13 ty                                   */ d->em.t[1],
                            /* 14 tz                                   */ d->em.t[2],
                            /* 15 dyaw                                 */ d->em.ypr[0],
                            /* 16 dpitch                               */ d->em.ypr[1],
                            /* 17 droll                                */ d->em.ypr[2],
                            /* 18 emYaw                                */ d->em.yaw,
                            /* 19 emPitch                              */ d->em.pitch,
                            /* 20 foeRectX                             */ yawLm2Rect,
                            /* 21 foeRectY                             */ horLm2Rect,
                            /* 22 foeDistX                             */ yawLm2Dist,
                            /* 23 foeDistY                             */ horLm2Dist,
                            /* 24 emValid                              */ d->em.valid,
                            /* 25 emStatus                             */ d->em.status,
                            /* 26 emConf                               */ d->em.conf,
                            /* 27 emConfT                              */ d->em.confT,
                            /* 28 emConfR                              */ d->em.confR,
                            /* 29 emValidFrames                        */ d->em.validFrames,
                            /* 30 rmsValid                             */ d->em.rmsValid,
                            /* 31 Nx                                   */ d->rm.N[0],
                            /* 32 Ny                                   */ d->rm.N[1],
                            /* 33 Nz                                   */ d->rm.N[2],
                            /* 34 d                                    */ d->rm.d,
                            /* 35 roll                                 */ d->rm.roll,
                            /* 36 rmPitch                              */ d->rm.pitch,
                            /* 37 rmValid                              */ d->rm.valid,
                            /* 38 rmStatus                             */ d->rm.status,
                            /* 39 rmNumOfInliers                       */ d->rm.numOfInliers,
                            /* 40 rmPlaneIdx                           */ d->rm.planeIdx,
                            /* 41 rmMessage                            */ d->rm.message,
                            /* 42 crZ                                  */ d->rm.crownZ,
                            /* 43 crX0                                 */ d->rm.crownX[0],
                            /* 44 crX1                                 */ d->rm.crownX[1],
                            /* 45 crX2                                 */ d->rm.crownX[2],
                            /* 46 crX3                                 */ d->rm.crownX[3],
                            /* 47 crX4                                 */ d->rm.crownX[4],
                            /* 48 crA0                                 */ d->rm.crownAng[0],
                            /* 49 crA1                                 */ d->rm.crownAng[1],
                            /* 50 crA2                                 */ d->rm.crownAng[2],
                            /* 51 crA3                                 */ d->rm.crownAng[3],
                            /* 52 crA4                                 */ d->rm.crownAng[4],
                            /* 53 crAstd0                              */ d->rm.crownAngStd[0],
                            /* 54 crAstd1                              */ d->rm.crownAngStd[1],
                            /* 55 crAstd2                              */ d->rm.crownAngStd[2],
                            /* 56 crAstd3                              */ d->rm.crownAngStd[3],
                            /* 57 crAstd4                              */ d->rm.crownAngStd[4],
                            /* 58 crADiff                              */ d->rm.crownAngDiff,
                            /* 59 crADiff1                             */ d->rm.crownAngDiff1,
                            /* 60 crdDiff1                             */ d->rm.crownDistDiff1,
                            /* 61 pitchDiff                            */ d->rm.pitchDiff,
                            /* 62 radius                               */ d->vehicle.radius,
                            /* 63 speed                                */ d->vehicle.speed,
                            /* 64 yawRate                              */ d->vehicle.yawRate,
                            /* 65 dt                                   */ d->vehicle.dt,
                            /* 66 acceleration                         */ d->vehicle.accel,
                            /* 67 reverseGear                          */ d->vehicle.reverseGear,
                            /* 68 vehicleKneeling                      */ d->vehicle.vehicleKneeling,
                            /* 69 speedHighPrecisionValid              */ d->vehicle.speedHighPrecisionValid,
                            /* 70 speedHighPrecision                   */ d->vehicle.speedHighPrecision,
                            /* 71 speedPrecisionError                  */ d->vehicle.speedPrecisionError,
                            /* 72 dynamicCameraHeightValid             */ d->vehicle.dynamicCameraHeightValid,
                            /* 73 dynamicCameraHeight                  */ d->vehicle.dynamicCameraHeight,
                            /* 74 dynamicCameraHeightDeltaValid        */ d->vehicle.dynamicCameraHeightDeltaValid,
                            /* 75 dynamicCameraHeightDelta             */ d->vehicle.dynamicCameraHeightDelta,
                            /* 76 dynamicCameraHeightChangeActiveValid */ d->vehicle. dynamicCameraHeightChangeActiveValid,
                            /* 77 dynamicCameraHeightChangeActive      */ d->vehicle.dynamicCameraHeightChangeActive,
                            /* 78 dynamicCameraHeightDeltaAvailable    */ d->vehicle.dynamicCameraHeightDeltaAvailable,
                            /* 79 accumulatedDistance                  */ d->vehicle.trajLength,
                            /* 80 quickMode                            */ d->algo.quickMode
                            );

    itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_INPUT_DATA_CONF],
                            /* WMBC InputDataConf                      */
                            /* 04 camPort                              */ _camInst,
                            /* 05 rmsPts                               */ d->em.rmsPtNum[0],
                            /* 06 rmsInliers                           */ d->em.rmsPtNum[1],
                            /* 07 epiValid                             */ d->em.epiValid,
                            /* 08 epiInvalidPercent                    */ d->em.epiInvalidPercent,
                            /* 09 epiQualityFrameNum                   */ d->em.epiQualityFrameNum,
                            /* 10 epiQuality                           */ d->em.epiQuality,
                            /* 11 epiErr                               */ d->em.epiErr,
                            /* 12 epiErrAng                            */ d->em.epiErrAng,
                            /* 13 epiCovXX                             */ d->em.epiCov[0],
                            /* 14 epiCovYY                             */ d->em.epiCov[1],
                            /* 15 epiCovXY                             */ d->em.epiCov[2],
                            /* 16 foeRectLsX                           */ d->em.foeRectLm2.X(),
                            /* 17 foeRectLsY                           */ d->em.foeRectLm2.Y(),
                            /* 18 epiYawAngle                          */ d->em.epiFoeAngle[0],
                            /* 19 epiPitchAngle                        */ d->em.epiFoeAngle[1],
                            /* 20 resConfMeanPitch                     */ d->rm.resConfMean[e_HORIZON],
                            /* 21 resConfMeanStringentPitch            */ d->rm.resConfMeanStringent[e_HORIZON],
                            /* 22 residualPtNumPitch                   */ d->rm.residualPtNum[e_HORIZON],
                            /* 23 residualMeanPitch                    */ d->rm.residualMean[e_HORIZON],
                            /* 24 residualVarPitch                     */ d->rm.residualVar[e_HORIZON],
                            /* 25 residualMinPitch                     */ d->rm.residualMin[e_HORIZON],
                            /* 26 residualMaxPitch                     */ d->rm.residualMax[e_HORIZON],
                            /* 27 resConfMeanRoll                      */ d->rm.resConfMean[e_ROLL],
                            /* 28 resConfMeanStringentRoll             */ d->rm.resConfMeanStringent[e_ROLL],
                            /* 29 residualPtNumRoll                    */ d->rm.residualPtNum[e_ROLL],
                            /* 30 residualMeanRoll                     */ d->rm.residualMean[e_ROLL],
                            /* 31 residualVarRoll                      */ d->rm.residualVar[e_ROLL],
                            /* 32 residualMinRoll                      */ d->rm.residualMin[e_ROLL],
                            /* 33 residualMaxRoll                      */ d->rm.residualMax[e_ROLL],
                            /* 34 resConfMeanCamh                      */ d->rm.resConfMean[e_CAM_HEIGHT],
                            /* 35 resConfMeanStringentCamh             */ d->rm.resConfMeanStringent[e_CAM_HEIGHT],
                            /* 36 residualPtNumCamh                    */ d->rm.residualPtNum[e_CAM_HEIGHT],
                            /* 37 residualMeanCamh                     */ d->rm.residualMean[e_CAM_HEIGHT],
                            /* 38 residualVarCamh                      */ d->rm.residualVar[e_CAM_HEIGHT],
                            /* 39 residualMinCamh                      */ d->rm.residualMin[e_CAM_HEIGHT],
                            /* 40 residualMaxCamh                      */ d->rm.residualMax[e_CAM_HEIGHT],
                            /* 41 rmAtA00                              */ *(d->rm.AtA(0, 0)),
                            /* 42 rmAtA01                              */ *(d->rm.AtA(0, 1)),
                            /* 43 rmAtA02                              */ *(d->rm.AtA(0, 2)),
                            /* 44 rmAtA11                              */ *(d->rm.AtA(1, 1)),
                            /* 45 rmAtA12                              */ *(d->rm.AtA(1, 2)),
                            /* 46 rmAtA22                              */ *(d->rm.AtA(2, 2)),
                            /* 47 rmNAtA00                             */ *(d->rm.nAtA(0, 0)),
                            /* 48 rmNAtA01                             */ *(d->rm.nAtA(0, 1)),
                            /* 49 rmNAtA02                             */ *(d->rm.nAtA(0, 2)),
                            /* 50 rmNAtA11                             */ *(d->rm.nAtA(1, 1)),
                            /* 51 rmNAtA12                             */ *(d->rm.nAtA(1, 2)),
                            /* 52 rmNAtA22                             */ *(d->rm.nAtA(2, 2)),
                            /* 53 rmResidual                           */ d->rm.rmResidual,
                            /* 54 rmCovValid                           */ d->rm.validCov,
                            /* 55 rmCov00                              */ *(d->rm.cov(0, 0)),
                            /* 56 rmCov01                              */ *(d->rm.cov(0, 1)),
                            /* 57 rmCov02                              */ *(d->rm.cov(0, 2)),
                            /* 58 rmCov11                              */ *(d->rm.cov(1, 1)),
                            /* 59 rmCov12                              */ *(d->rm.cov(1, 2)),
                            /* 60 rmCov22                              */ *(d->rm.cov(2, 2)),
                            /* 61 snow                                 */ d->camera.snow,
                            /* 62 night                                */ d->camera.night,
                            /* 63 snowNightMode                        */ d->camera.snowNightMode,
                            /* 64 nonSnowNightCounter                  */ d->camera.nonSnowNightCounter
                           );

  }

  void WmbcOutputIF::toItrkFrameValidation(const WmbcData* d) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_FRAME_VALIDATION],
                            /* WMBC FrameValidation     */
                            /* 04 camPort               */ _camInst,
                            /* 05 validFrame            */ d->algo.total.validFrame,
                            /* 06 validFrameYaw         */ d->algo.single[e_YAW].validFrame,
                            /* 07 validFrameHorizon     */ d->algo.single[e_HORIZON].validFrame,
                            /* 08 validFrameRoll        */ d->algo.single[e_ROLL].validFrame,
                            /* 09 validFrameCamh        */ d->algo.single[e_CAM_HEIGHT].validFrame,
                            /* 10 validFrameYawNum      */ d->algo.single[e_YAW].validFrameNum,
                            /* 11 validFrameHorizonNum  */ d->algo.single[e_HORIZON].validFrameNum,
                            /* 12 validFrameRollNum     */ d->algo.single[e_ROLL].validFrameNum,
                            /* 13 validFrameCamhNum     */ d->algo.single[e_CAM_HEIGHT].validFrameNum,
                            /* 14 speedOk               */ (d->algo.pauseReason & (e_SPEED_TOO_LOW | e_SPEED_TOO_HIGH)) == 0,
                            /* 15 drivingOk             */ (d->algo.pauseReason & e_REVERSE_IS_ON)                      == 0,
                            /* 16 radiusOk              */ (d->algo.pauseReason & e_RADIUS_TOO_SMALL)                   == 0,
                            /* 17 accelOk               */ (d->algo.pauseReason & e_ACCELERATION_TOO_HIGH)              == 0,
                            /* 18 yawRateOk             */ (d->algo.pauseReason & e_YAWRATE_TOO_HIGH)                   == 0,
                            /* 19 dynamicSuspensionOk   */ (d->algo.pauseReason & e_DYNAMIC_SUSPENSION_ACTIVE)          == 0,
                            /* 20 straightDrive         */ (d->algo.pauseReason & e_EM_STRAIGHT)                        == 0,
                            /* 21 pitchdiffOk           */ (d->algo.pauseReason & e_EM_RM_PITCHDIFF)                    == 0,
                            /* 22 crownOk               */ (d->algo.pauseReason & e_ROAD_CROWN)                         == 0,
                            /* 23 speedPrecOk           */ (d->algo.pauseReason & e_SPEED_HP_INVALID)                   == 0,
                            /* 24 pauseReason           */ d->algo.pauseReason,
                            /* 25 delayValidCounter     */ 0,
                            /* 26 minSpeed              */ d->validParams.minSpeed,
                            /* 27 maxSpeed              */ d->validParams.maxSpeed,
                            /* 28 minRadius             */ d->validParams.minRadius,
                            /* 29 thDyaw                */ d->validParams.rotThreshold[0],
                            /* 30 thDpitch              */ d->validParams.rotThreshold[1],
                            /* 31 thDroll               */ d->validParams.rotThreshold[2],
                            /* 32 maxAccel              */ d->validParams.maxAccel,
                            /* 33 maxYawRate            */ d->validParams.maxYawRate,
                            /* 34 maxPitchDiff          */ d->validParams.maxPitchDiff,
                            /* 35 maxCrownAngDiff       */ d->validParams.maxCrownAngDiff,
                            /* 36 useSpeedHighPrecision */ d->validParams.useSpeedHighPrecision,
                            /* 37 useDynamicSuspension  */ d->validParams.useDynamicSuspension,
                            /* 38 delayValidThreshold   */ d->validParams.delayValidThreshold,
                            /* 39 emValidStraightFrameNum  */ d->em.validStraightFrameNum,
                            /* 40 emEpiQualityFrameNum  */ d->em.epiQualityFrameNum,
                            /* xx crADiffMean           */ d->rm.crownAngDiffMean,
                            /* xx crADiffVar            */ d->rm.crownAngDiffVar,
                            /* xx crADiffMin            */ d->rm.crownAngDiffMin,
                            /* xx crADiffMax            */ d->rm.crownAngDiffMax,
                            /* 41 quickMode             */ d->algo.quickMode
                            );
  }

  void WmbcOutputIF::toItrkFoe(const HistogramOneDim *histYaw, const HistogramOneDim *histHorizon, float focalLm2, bool quickMode) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    float yawLm2Rect       = histYaw->median() * focalLm2;
    float horLm2Rect       = histHorizon->median() * focalLm2;
    float yawLm2Dist       = yawLm2Rect;
    float horLm2Dist       = horLm2Rect;
    float yawFrameLm2Rect  = histYaw->lastVal() * focalLm2;
    float horFrameLm2Rect  = histHorizon->lastVal() * focalLm2;
    float yawFrameLm2Dist  = yawFrameLm2Rect;
    float horFrameLm2Dist  = horFrameLm2Rect;
    bool isDistortionValid = DistortionCorrectionAPI::isDistortionValid();

    if (isDistortionValid) {
      DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, yawLm2Rect, horLm2Rect, yawLm2Dist, horLm2Dist);
      DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, yawFrameLm2Rect, horFrameLm2Rect, yawFrameLm2Dist, horFrameLm2Dist);
    }

    itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_FOE],
                            /* WMBC Foe             */
                            /* 04 camPort           */ _camInst,
                            /* 05 isDistortionValid */ isDistortionValid,
                            /* 06 yawLm2Rect        */ yawLm2Rect,
                            /* 07 horLm2Rect        */ horLm2Rect,
                            /* 08 yawLm2Dist        */ yawLm2Dist,
                            /* 09 horLm2Dist        */ horLm2Dist,
                            /* 10 yawFrameLm2Rect   */ yawFrameLm2Rect,
                            /* 11 horFrameLm2Rect   */ horFrameLm2Rect,
                            /* 12 yawFrameLm2Dist   */ yawFrameLm2Dist,
                            /* 13 horFrameLm2Dist   */ horFrameLm2Dist,
                            /* 14 quickMode         */ quickMode
                            );
  }

  void WmbcOutputIF::toItrkRoll(const HistogramOneDim *histRoll, const HistogramOneDim *histCamh, bool quickMode) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_ROLL],
                            /* WMBC Roll     */
                            /* 04 camPort    */ _camInst,
                            /* 05 frameRoll  */ histRoll->lastVal(),
                            /* 06 medianRoll */ histRoll->median(),
                            /* 07 frameCamh  */ histCamh->lastVal(),
                            /* 08 medianCamh */ histCamh->median(),
                            /* 09 quickMode  */ quickMode
                            );
  }

  void WmbcOutputIF::toItrkCalibVariable(const CalibVariable *c, const WmbcData* d) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_CALIB_VAR],
                            /* WMBC CalibVariable              */
                            /* 04 camPort                      */ _camInst,
                            /* 05 id                           */ c->id(),
                            /* 06 status                       */ c->status(),
                            /* 07 sf                           */ c->sf(),
                            /* 08 median                       */ c->resultInterim(),
                            /* 09 var                          */ c->hist().varDecay(),
                            /* 10 validFrame                   */ c->validFrame(),
                            /* 11 validFrameNum                */ c->validFrameNum(),
                            /* 12 validDistance                */ c->validDistance(),
                            /* 13 validTime                    */ c->validTime(),
                            /* 14 invalidFrameMask             */ c->invalidFrameMask(),
                            /* 15 conv                         */ c->conv(),
                            /* 16 convNum                      */ c->convNum(),
                            /* 17 convLastFrame                */ c->convLastFrame(),
                            /* 18 convMovingSampleNumThreshold */ c->convMovingSampleNumThreshold(),
                            /* 19 progress                     */ c->progress(),
                            /* 20 conf0                        */ c->confidence(0),
                            /* 21 conf1                        */ c->confidence(1),
                            /* 22 conf2                        */ c->confidence(2),
                            /* 23 nonConvReason                */ c->nonConvReason(),
                            /* 24 stableMedianCount            */ c->stableMedianCount(),
                            /* 25 maxMedianDiff                */ d->convParams.convStableMaxDiff[c->id()],
                            /* 26 stableMedianNumThreshold     */ d->convParams.convStableNumThreshold,
                            /* 27 maxVar                       */ d->convParams.convMaxVar[c->id()],
                            /* 28 minSample                    */ c->convMovingSampleNumThreshold(),
                            /* 29 inRange                      */ c->inRange(),
                            /* 29 emLastValidFrameNum          */ c->emLastValidFrameNum(),
                            /* 29 emLastEpiQualityFrameNum     */ c->emLastEpiQualityFrameNum(),
                            /* xx crownQuality                 */ c->crownQuality(),
                            /* 30 quickMode                    */ 0
                              );

    itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_HIST],
                            /* WMBC Histogram  */
                            /* 04 camPort      */ _camInst,
                            /* 05 id           */ c->id(),
                            /* 06 mean         */ c->hist().mean(),
                            /* 07 var          */ c->hist().var(),
                            /* 08 median       */ c->hist().median(),
                            /* 09 prevMedian   */ c->hist().prevMedian(),
                            /* 10 min          */ c->hist().min(),
                            /* 11 max          */ c->hist().max(),
                            /* 12 minBin       */ c->hist().minBin(),
                            /* 13 maxBin       */ c->hist().maxBin(),
                            /* 14 currValExact */ c->hist().lastVal(),
                            /* 15 currVal      */ c->hist().lastValHist(),
                            /* 16 currBin      */ c->hist().lastBin(),
                            /* 17 sampleNum    */ c->hist().sampleNum(),
                            /* 18 binSize      */ c->hist().binSize(),
                            /* 19 meanDecay    */ c->hist().meanDecay(),
                            /* 20 varDecay     */ c->hist().varDecay(),
                            /* 21 decayFac     */ c->hist().decayFac(),
                            /* 22 startDecay   */ c->hist().startDecay(),
                            /* 23 iqr          */ c->hist().iqr(),
                            /* 24 valid        */ c->hist().valid(),
                            /* 25 autocov      */ c->hist().autocov(),
                            /* 26 autocorr     */ c->hist().autocorr()
                            );
  }

   void WmbcOutputIF::toItrkPlane(const WmbcData *d) const {
   if (!itrkWriter::isItrkActive()) {
     return;
   }
   
   itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_PLANE],
                           /* WMBC Plane              */
                           /* 04 camPort               */ _camInst,
                           /* 05 valid                 */ d->plane.valid,
                           /* 06 roll                  */ d->plane.roll,
                           /* 07 camh                  */ d->plane.camh,
                           /* 08 location              */ d->plane.location,
                           /* 08 startFrame            */ d->plane.startFrame,
                           /* 09 endFrame              */ d->plane.endFrame,
                           /* xx rmsPts                */ d->plane.confData.rmStorageSize,
                           /* xx epiErrAng             */ d->plane.confData.epiErrAng,
                           /* xx epiCovXX              */ d->plane.confData.epiCovXX,
                           /* xx epiCovYY              */ d->plane.confData.epiCovYY,
                           /* xx rmResidual            */ d->plane.confData.rmResidual,
                           /* xx residualMeanPitch     */ d->plane.confData.residualMean[e_HORIZON],
                           /* xx residualPtNumPitch    */ d->plane.confData.residualPtNum[e_HORIZON],
                           /* xx residualMeanRoll      */ d->plane.confData.residualMean[e_ROLL],
                           /* xx residualPtNumRoll     */ d->plane.confData.residualPtNum[e_ROLL],
                           /* xx residualMeanCamh      */ d->plane.confData.residualMean[e_CAM_HEIGHT],
                           /* xx residualPtNumCamh     */ d->plane.confData.residualPtNum[e_CAM_HEIGHT],
                           /* xx rmValidCov            */ d->plane.confData.rmValidCov,
                           /* xx rmCov00               */ *(d->plane.confData.rmCov(0, 0)),
                           /* xx rmCov11               */ *(d->plane.confData.rmCov(1, 1)),
                           /* xx rmCov22               */ *(d->plane.confData.rmCov(2, 2)),
                           /* xx pitchDiff             */ d->plane.confData.pitchDiff,
                           /* xx crADiff               */ d->plane.confData.crownAngDiff
                           );
 }
             

   void WmbcOutputIF::toItrkPlaneBuffer(const MEtypes::FastCyclicVector<Plane> *p) const {
   if (!itrkWriter::isItrkActive()) {
     return;
   }

   float d[PLANE_BUFFER_CAPACITY];
   float loc[PLANE_BUFFER_CAPACITY];
   int   f[PLANE_BUFFER_CAPACITY];
   for (int i=0; i<PLANE_BUFFER_CAPACITY; ++i) {
     d[i]=0.f;
     loc[i]=0.f;
     f[i] = 0;
   }
   int i=0;
   for (MEtypes::FastCyclicVector<Plane>::const_iterator iter = p->begin(); iter!= p->end(); iter++) {
     d[i]=iter->d;
     loc[i]=iter->location;
     f[i]=iter->startFrame;
     i++;
   }
   
   itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_PLANE_BUFFER],
                           /* WMBC Plane              */
                           /* 04 camPort               */ _camInst,
                           /* 05 bufferSize            */ p->size(),
                           /* 06 d1                    */ d[0],
                           /* 08 d2                    */ d[1],
                           /* 09 d3                    */ d[2],
                           /* 10 d4                    */ d[3],
                           /* 11 d5                    */ d[4],
                           /* 12 d6                    */ d[5],
                           /* 13 d7                    */ d[6],
                           /* 14 d8                    */ d[7],
                           /* 15 loc1                  */ loc[0],
                           /* 16 loc2                  */ loc[1],
                           /* 17 loc3                  */ loc[2],
                           /* 18 loc4                  */ loc[3],
                           /* 19 loc5                  */ loc[4],
                           /* 20 loc6                  */ loc[5],
                           /* 21 loc7                  */ loc[6],
                           /* 23 loc8                  */ loc[7],
                           /* 24 f0                    */ f[0],
                           /* 25 f1                    */ f[1],
                           /* 26 f2                    */ f[2],
                           /* 27 f3                    */ f[3],
                           /* 28 f4                    */ f[4],
                           /* 29 f5                    */ f[5],
                           /* 30 f6                    */ f[6],
                           /* 31 f7                    */ f[7]
                           );
 }

   void WmbcOutputIF::toItrkConvergence(int calibInd) const { // tmp for itrk to run
     if (!itrkWriter::isItrkActive()) {
       return;
     }

//      bool cond[5];
//      for (int i = 0; i < 3; ++i) {
//        cond[i] = condIn[i];
//      }
//      cond[3] = quickMode ? 1 : condIn[3];
//      cond[4] = quickMode ? 1 : condIn[4];

     // TODO: assert cond size
     itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_CONVERGENCE],
                             /* WMBC Convergence     */
                             /* 04 camPort           */ _camInst,
                             /* 05 calibInd          */ calibInd,
                             /* 06 conv              */ 0, //conv,
                             /* 07 totalConv         */ 0, //cd->totalConv,
                             /* 08 smallVar          */ 0, //cond[0],
                             /* 09 isGauss           */ 0, //cond[1],
                             /* 10 largeSample       */ 0, //cond[2],
                             /* 11 medianStable      */ 0, //cond[3],
                             /* 12 highQuality       */ 0, //cond[4],
                             /* 13 stableMedianCount */ 0, //stableMedianCount,
                             /* 14 quality           */ 0, //cd->quality[calibInd],
                             /* 15 var               */ 0, //hist->getVar(),
                             /* 16 mean              */ 0, //hist->getMean(),
                             /* 17 median            */ 0, //hist->getMedian(),
                             /* 18 prevMedian        */ 0, //hist->getPrevMedian(),
                             /* 19 iqr               */ 0, //hist->getIqr(),
                             /* 20 sampleNum         */ 0, //hist->getSampleNum(),
                             /* 21 binSize           */ 0, //hist->getBinSize(),
                             /* 22 meanDecay         */ 0, //hist->getMeanDecay(),
                             /* 23 varDecay          */ 0, //hist->getVarDecay(),
                             /* 24 decayFac          */ 0, //hist->getDecayFac(),
                             /* 25 startDecay        */ 0, //hist->getStartDecay(),
                             /* 26 frameVal          */ 0, //hist->getLastVal(),
                             /* 27 quickMode         */ 0 //quickMode
                             );
   }

   void WmbcOutputIF::toItrkSlcTargets(bool valid, SPInfoVec *sp, float imHeight, float imWidth, float distToTargets, const TargetParams *tparams, 
                                       const Fix::MEimage::Rect *targetSearchWindow, int targetsLevel) const {
     if (!itrkWriter::isItrkActive()) {
       return;
     }

     float x[4], y[4];
     unsigned int n = tparams->spNumInTarget;

     unsigned int ind[4] = {0, n-1, n, 2*n-1}; // bottom-left, top-left, bottom-right, top-right
     for (int i = 0; i < 4; ++i) {
       x[i] = sp->size() > ind[i] ? (*sp)[ind[i]].xSub : INVALID_VAL;
       y[i] = sp->size() > ind[i] ? (*sp)[ind[i]].ySub : INVALID_VAL;
     }

     itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_SLC_TARGETS],
                             /* WMBC SlcTargets             */
                             /* 04 camPort                  */ _camInst,
                             /* 05 valid                    */ valid,
                             /* 06 spNum                    */ sp->size(),
                             /* 07 xBL                      */ x[0],
                             /* 08 yBL                      */ y[0],
                             /* 09 xTL                      */ x[1],
                             /* 10 yTL                      */ y[1],
                             /* 11 xBR                      */ x[2],
                             /* 12 yBR                      */ y[2],
                             /* 13 xTR                      */ x[3],
                             /* 14 yTR                      */ y[3],
                             /* 15 imHeight                 */ imHeight,
                             /* 16 imWidth                  */ imWidth,
                             /* 17 distToTargets            */ distToTargets,
                             /* 18 height                   */ tparams->height,
                             /* 19 witdh                    */ tparams->width,
                             /* 20 rectW                    */ tparams->rect[0],
                             /* 21 rectH                    */ tparams->rect[1],
                             /* 22 rows                     */ tparams->dim[0],
                             /* 23 cols                     */ tparams->dim[1],
                             /* 24 colorBLL                 */ tparams->bottomLeftSquare[0],
                             /* 25 colorBLR                 */ tparams->bottomLeftSquare[1],
                             /* 26 patternMatchingThreshold */ tparams->patternMatchingThreshold,
                             /* 27 xSearch0                 */ targetSearchWindow[0].left(),
                             /* 28 xSearch1                 */ targetSearchWindow[0].right(),
                             /* 29 xSearch2                 */ targetSearchWindow[1].left(),
                             /* 30 xSearch3                 */ targetSearchWindow[1].right(),
                             /* 31 ySearch0                 */ targetSearchWindow[0].bottom(),
                             /* 32 ySearch1                 */ targetSearchWindow[0].top(),
                             /* 33 ySearch2                 */ targetSearchWindow[1].bottom(),
                             /* 34 ySearch3                 */ targetSearchWindow[1].bottom(),
                             /* 35 targetsLevel             */ targetsLevel
                             );
   }
 
   void WmbcOutputIF::toItrkSlcCamHeight(CamhDataVec *cd, SlcConvParams *cp, bool *phaseStates, float camhErrMean) const {
     if (!itrkWriter::isItrkActive()) {
       return;
     }
     
     // bool inCalcPhase = (restConverged && (int)cd->size() > cp->minValidFramesCamh); // correlate with condition inside FOEFinder_Stationless::calcCamHeight()
     CamhDataIter begin = cd->begin();
     CamhDataIter end = cd->end();
 
     for(CamhDataIter it = begin; it != end; ++it) {
       itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_SLC_CAMH],
                               /* WMBC SlcCamHeight      */
                               /* 04 camPort             */ _camInst,
                               /* 05 dataFrame           */ it->frame,
                               /* 06 valid               */ it->valid,
                               /* 07 chosen              */ it->chosen,
                               /* 08 camh                */ it->camh,
                               /* 09 camhErr             */ it->camhErr,
                               /* 10 invImageTargetWidth */ it->invImageTargetWidth,
                               /* 11 distance            */ it->distance,
                               /* 12 targetCenterX       */ it->targetCornersY[0],
                               /* 13 targetCenterY       */ it->targetCornersY[2],
                               /* 14 foerX               */ it->foer[0],
                               /* 15 foerY               */ it->foer[1],
                               /* 16 camhErrMean         */ camhErrMean,
                               /* 17 restConverged       */ phaseStates[0],
                               /* 18 isAccumulate        */ phaseStates[1],
                               /* 19 isCalc              */ phaseStates[2],
                               /* 20 camDataSize         */ cd->size(),
                               /* 21 minValidFramesCamh  */ cp->minValidFramesCamh
                               );
     }
   }
 
   void WmbcOutputIF::toItrkSlcConvergence(bool isEndTraj, int validFramesNum, const WmbcData *d, const SlcConvParams *cparams , const TargetParams *tparams) const {
     if (!itrkWriter::isItrkActive()) {
       return;
     }
 
     itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_SLC_CONVERGENCE],
                             /* WMBC SlcConvergence    */
                             /* 04 camPort             */ _camInst,
                             /* 05 totalConv           */ d->algo.total.conv,
                             /* 06 isEndTraj           */ isEndTraj,
                             /* 07 accumulatedDistance */ d->vehicle.trajLength,
                             /* 08 validFramesNum      */ validFramesNum,
                             /* 09 minValidFrames      */ cparams->minValidFrames,
                             /* 10 maxRunFrames        */ cparams->maxRunFrames,
                             /* 11 maxTravelDistance   */ cparams->maxTravelDist,
                             /* 12 maxDistance         */ tparams->maxDistance,
                             /* 13 minDistance         */ tparams->minDistance,
                             /* 14 camhMaxDistance     */ tparams->camhMaxDistance,
                             /* 15 totalProgress       */ d->algo.total.progress,
                             /* 16 progressYaw         */ d->algo.single[e_YAW].progress,
                             /* 17 progressYaw         */ d->algo.single[e_HORIZON].progress,
                             /* 18 progressYaw         */ d->algo.single[e_ROLL].progress,
                             /* 19 progressYaw         */ d->algo.single[e_CAM_HEIGHT].progress,
                             /* 20 totalQuality        */ d->algo.total.quality,
                             /* 21 qualityYaw          */ d->algo.single[e_YAW].quality,
                             /* 22 qualityHorizon      */ d->algo.single[e_HORIZON].quality,
                             /* 23 qualityRoll         */ d->algo.single[e_ROLL].quality,
                             /* 24 qualityCamh         */ d->algo.single[e_CAM_HEIGHT].quality
                             );
   }

  void WmbcOutputIF::toItrkDebug(const WmbcData* d) const {  
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    bool isDistortionValid = DistortionCorrectionAPI::isDistortionValid();

    itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_DEBUG],
                            /* WMBC Debug                  */
                            /* 04 camPort                  */ _camInst,
                            /* 05 yawAngleDeg              */ d->results.curr.angles[0]*RAD2DEG,//foe->yawAnglePrimary*RAD2DEG,
                            /* 06 pitchAngleDeg            */ d->results.curr.angles[1]*RAD2DEG,//foe->pitchAnglePrimary*RAD2DEG,
                            /* 07 rollAngleDeg             */ d->results.curr.angles[2]*RAD2DEG,//foe->rollPrimary*RAD2DEG,
                            /* 08 cameraHeight             */ d->results.curr.camh,//foe->camhPrimary,
                            /* 09 yawLm2Primary_f          */ d->results.curr.foeLm2.X() + d->camera.origin[_camInst].distDeltaLm2.X(),//foe->yawLm2Primary_f,
                            /* 10 horizonLm2Primary_f      */ d->results.curr.foeLm2.Y() + d->camera.origin[_camInst].distDeltaLm2.Y(),//foe->horizonLm2Primary_f,
                            /* 11 yawDeltaLm2Primary_f     */ d->results.curr.foeLm2.X(),//foe->yawDeltaLm2FromEtcPrimary_f,
                            /* 12 horizonDeltaLm2Primary_f */ d->results.curr.foeLm2.Y(),//foe->horizonDeltaLm2FromEtcPrimary_f,
                            /* 13 runMode                  */ d->metaParams.runMode,
                            /* 14 status                   */ d->algo.total.status,
                            /* 15 convYaw                  */ d->algo.single[e_YAW].conv,
                            /* 16 convHorizon              */ d->algo.single[e_HORIZON].conv,
                            /* 17 convRoll                 */ d->algo.single[e_ROLL].conv,
                            /* 18 convCamh                 */ d->algo.single[e_CAM_HEIGHT].conv,
                            /* 19 totalConv                */ d->algo.total.conv,
                            /* 20 totalConvNum             */ d->algo.total.convNum,
                            /* 21 totalConvLastFrame       */ d->algo.total.convLastFrame,
                            /* 22 qualityYaw               */ d->algo.single[e_YAW].quality,
                            /* 23 qualityHorizon           */ d->algo.single[e_HORIZON].quality,
                            /* 24 qualityRoll              */ d->algo.single[e_ROLL].quality,
                            /* 25 qualityCamh              */ d->algo.single[e_CAM_HEIGHT].quality,
                            /* 26 totalQuality             */ d->algo.total.quality,
                            /* 27 confYaw0                 */ d->algo.single[e_YAW].confidence[0],
                            /* 28 confYaw1                 */ d->algo.single[e_YAW].confidence[1],
                            /* 29 confYaw2                 */ d->algo.single[e_YAW].confidence[2],
                            /* 30 confHorizon0             */ d->algo.single[e_HORIZON].confidence[0],
                            /* 31 confHorizon1             */ d->algo.single[e_HORIZON].confidence[1],
                            /* 32 confHorizon2             */ d->algo.single[e_HORIZON].confidence[2],
                            /* 33 confRoll0                */ d->algo.single[e_ROLL].confidence[0],
                            /* 34 confRoll1                */ d->algo.single[e_ROLL].confidence[1],
                            /* 35 confRoll2                */ d->algo.single[e_ROLL].confidence[2],
                            /* 36 confCamh0                */ d->algo.single[e_CAM_HEIGHT].confidence[0],
                            /* 37 confCamh1                */ d->algo.single[e_CAM_HEIGHT].confidence[1],
                            /* 38 confCamh2                */ d->algo.single[e_CAM_HEIGHT].confidence[2],
                            /* 39 conf0                    */ d->algo.total.confidence[0],
                            /* 40 conf1                    */ d->algo.total.confidence[1],
                            /* 41 conf2                    */ d->algo.total.confidence[2],
                            /* 42 totalConf                */ d->algo.total.totalConfidence,
                            /* 43 totalConfGrade           */ d->algo.total.totalConfGrade,
                            /* 44 progressYaw              */ d->algo.single[e_YAW].progress,
                            /* 45 progressHorizon          */ d->algo.single[e_HORIZON].progress,
                            /* 46 progressRoll             */ d->algo.single[e_ROLL].progress,
                            /* 47 progressCamh             */ d->algo.single[e_CAM_HEIGHT].progress,
                            /* 48 totalProgress            */ d->algo.total.progress,
                            /* 49 validFrameYaw            */ d->algo.single[e_YAW].validFrame,
                            /* 50 validFrameHorizon        */ d->algo.single[e_HORIZON].validFrame,
                            /* 51 validFrameRoll           */ d->algo.single[e_ROLL].validFrame,
                            /* 52 validFrameCamh           */ d->algo.single[e_CAM_HEIGHT].validFrame,
                            /* 53 validFrameYawNum         */ d->algo.single[e_YAW].validFrameNum,
                            /* 54 validFrameHorizonNum     */ d->algo.single[e_HORIZON].validFrameNum,
                            /* 55 validFrameRollNum        */ d->algo.single[e_ROLL].validFrameNum,
                            /* 56 validFrameCamhNum        */ d->algo.single[e_CAM_HEIGHT].validFrameNum,
                            /* 57 focalLm2                 */ d->camera.focalLm2,
                            /* 58 radius                   */ d->vehicle.radius,
                            /* 59 speed                    */ d->vehicle.speed,
                            /* 60 yawRate                  */ d->vehicle.yawRate,
                            /* 61 dt                       */ d->vehicle.dt,
                            /* 62 acceleration             */ d->vehicle.accel,
                            /* 63 distance                 */ d->vehicle.trajLength,
                            /* 64 validYawDist             */ d->algo.single[e_YAW].validDistance,
                            /* 65 validHorizonDist         */ d->algo.single[e_HORIZON].validDistance,
                            /* 67 validRollDist            */ d->algo.single[e_ROLL].validDistance,
                            /* 68 validCamhDist            */ d->algo.single[e_CAM_HEIGHT].validDistance,
                            /* 69 emStatus                 */ d->em.status,
                            /* 70 rmStatus                 */ d->rm.status,
                            /* 71 emConf                   */ d->em.conf,
                            /* 72 emValidFrames            */ d->em.validFrames,
                            /* 73 distortionValid          */ isDistortionValid,
                            /* 74 wmFoeReset               */ d->algo.wmFoeReset,
                            /* 75 resetMask                */ d->algo.resetMask,
                            /* 76 yawDeltaAngDegWmR        */ d->results.deltaWmReset.angles[e_YAW]*RAD2DEG,
                            /* 77 pitchDeltaAngDegWmR      */ d->results.deltaWmReset.angles[e_HORIZON]*RAD2DEG,
                            /* 78 rollDeltaAngDegWmR       */ d->results.deltaWmReset.angles[e_ROLL]*RAD2DEG,
                            /* 79 camhDeltaWmR             */ d->results.deltaWmReset.angles[e_CAM_HEIGHT],
                            /* 80 quickMode                */ d->algo.quickMode
                            );
  }


  void WmbcOutputIF::toItrkFinal(const WmbcIF *wmbcIF, const WmbcData* d) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    for (int i=0 ; i < CameraInfo::e_NUM_OF_CAM_INSTANCES ; i++) {
      CameraInfo::CameraInstance inst = (CameraInfo::CameraInstance)i;
      if (!CameraInfo::exists(inst)) {
        continue;
      }
      itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_FINAL],
                              /* WMBC Final                           */
                              /* 04 camPort                           */ inst,
                              /* 05 yawL0                             */ wmbcIF->yawL0[i],
                              /* 06 horizonL0                         */ wmbcIF->horizonL0[i],
                              /* 07 roll                              */ wmbcIF->roll[i],
                              /* 08 cameraHeight                      */ wmbcIF->cameraHeight[i],
                              /* 09 yawDeltaL0FromEtc                 */ wmbcIF->yawDeltaL0FromEtc[i],
                              /* 10 horizonDeltaL0FromEtc             */ wmbcIF->horizonDeltaL0FromEtc[i],
                              /* 11 yawDeltaLm2FromEtc_f              */ wmbcIF->yawDeltaLm2FromEtc_f[i],
                              /* 12 horizonDeltaLm2FromEtc_f          */ wmbcIF->horizonDeltaLm2FromEtc_f[i],
                              /* 13 yawDeltaL0FromLastConv            */ wmbcIF->yawDeltaL0FromLastConv[i],
                              /* 14 horizonDeltaL0FromLastConv        */ wmbcIF->horizonDeltaL0FromLastConv[i],
                              /* 15 interimYawDeltaLm2FromEtc_f       */ wmbcIF->interimYawDeltaLm2FromEtc_f[i],
                              /* 16 interimHorizonDeltaLm2FromEtc_f   */ wmbcIF->interimHorizonDeltaLm2FromEtc_f[i],
                              /* 17 yawLm2                            */ wmbcIF->yawLm2[i],
                              /* 18 horizonLm2                        */ wmbcIF->horizonLm2[i],
                              /* 19 yawAngle                          */ wmbcIF->yawAngle[i],
                              /* 20 pitchAngle                        */ wmbcIF->pitchAngle[i],
                              /* 21 autoFix_yaw                       */ wmbcIF->autoFix_yaw[i],
                              /* 22 autoFix_horizon                   */ wmbcIF->autoFix_horizon[i],
                              /* 23 autoFixOnlineLm2_yaw              */ wmbcIF->autoFixOnlineLm2_yaw[i],
                              /* 24 autoFixOnlineLm2_horizon          */ wmbcIF->autoFixOnlineLm2_horizon[i],
                              /* 25 interimYawLm2                     */ wmbcIF->interimYawLm2[i],
                              /* 26 interimHorizonLm2                 */ wmbcIF->interimHorizonLm2[i],
                              /* 27 interimYawAngle                   */ wmbcIF->interimYawAngle[i],
                              /* 28 interimPitchAngle                 */ wmbcIF->interimPitchAngle[i],
                              /* 29 interimRoll                       */ wmbcIF->interimRoll[i],
                              /* 30 interimCameraHeight               */ wmbcIF->interimCameraHeight[i],
                              /* 31 interimYawDeltaL0FromEtc          */ wmbcIF->interimYawDeltaL0FromEtc[i],
                              /* 32 interimHorizonDeltaL0FromEtc      */ wmbcIF->interimHorizonDeltaL0FromEtc[i],
                              /* 33 interimYawDeltaL0FromLastConv     */ wmbcIF->interimYawDeltaL0FromLastConv[i],
                              /* 34 interimHorizonDeltaL0FromLastConv */ wmbcIF->interimHorizonDeltaL0FromLastConv[i],
                              /* 35 interimAutoFixOnlineLm2_yaw       */ wmbcIF->interimAutoFixOnlineLm2_yaw[i],
                              /* 36 interimAutoFixOnlineLm2_horizon   */ wmbcIF->interimAutoFixOnlineLm2_horizon[i],
                              /* 37 wmFoeReset                        */ wmbcIF->wmFoeReset,
                              /* 38 status                            */ wmbcIF->status,
                              /* 39 singleStatus_Yaw                  */ wmbcIF->singleStatus[e_YAW],
                              /* 40 singleStatus_Pitch                */ wmbcIF->singleStatus[e_HORIZON],
                              /* 41 singleStatus_Roll                 */ wmbcIF->singleStatus[e_ROLL],
                              /* 42 singleStatus_Camh                 */ wmbcIF->singleStatus[e_CAM_HEIGHT],
                              /* 43 singleError_Yaw                   */ wmbcIF->singleError[e_YAW],
                              /* 44 singleError_Pitch                 */ wmbcIF->singleError[e_HORIZON],
                              /* 45 singleError_Roll                  */ wmbcIF->singleError[e_ROLL],
                              /* 46 singleError_Camh                  */ wmbcIF->singleError[e_CAM_HEIGHT],
                              /* 47 pauseReason                       */ wmbcIF->pauseReason,
                              /* 48 singleProgess_Yaw                 */ wmbcIF->singleProgress[e_YAW],
                              /* 49 singleProgress_Pitch              */ wmbcIF->singleProgress[e_HORIZON],
                              /* 50 singleProgress_Roll               */ wmbcIF->singleProgress[e_ROLL],
                              /* 51 singleProgress_Camh               */ wmbcIF->singleProgress[e_CAM_HEIGHT],
                              /* 52 progress                          */ wmbcIF->progress,
                              /* 53 singleConverged_Yaw               */ wmbcIF->singleConverged[e_YAW],
                              /* 54 singleConverged_Pitch             */ wmbcIF->singleConverged[e_HORIZON],
                              /* 55 singleConverged_Roll              */ wmbcIF->singleConverged[e_ROLL],
                              /* 56 singleConverged_Camh              */ wmbcIF->singleConverged[e_CAM_HEIGHT],
                              /* 57 singleConvFailReason_Yaw          */ wmbcIF->singleConvFailReason[e_YAW],
                              /* 58 singleConvFailReason_Pitch        */ wmbcIF->singleConvFailReason[e_HORIZON],
                              /* 59 singleConvFailReason_Roll         */ wmbcIF->singleConvFailReason[e_ROLL],
                              /* 60 singleConvFailReason_Camh         */ wmbcIF->singleConvFailReason[e_CAM_HEIGHT],
                              /* 61 runMode                           */ wmbcIF->runMode,
                              /* 62 singleQuality_Yaw                 */ wmbcIF->singleQuality[e_YAW],
                              /* 63 singleQuality_Pitch               */ wmbcIF->singleQuality[e_HORIZON],
                              /* 64 singleQuality_Roll                */ wmbcIF->singleQuality[e_ROLL],
                              /* 65 singleQuality_Camh                */ wmbcIF->singleQuality[e_CAM_HEIGHT],
                              /* 66 quality                           */ wmbcIF->quality,
                              /* 67 singleConfidence_Yaw              */ wmbcIF->singleConfidence[e_YAW],
                              /* 68 singleConfidence_Pitch            */ wmbcIF->singleConfidence[e_HORIZON],
                              /* 69 singleConfidence_Roll             */ wmbcIF->singleConfidence[e_ROLL],
                              /* 70 singleConfidence_Camh             */ wmbcIF->singleConfidence[e_CAM_HEIGHT],
                              /* 71 confidence                        */ wmbcIF->confidence,
                              /* 72 confidenceGrade                   */ wmbcIF->confidenceGrade,
                              /* 73 totalDistance                     */ wmbcIF->totalDistance,
                              /* 74 validDistance                     */ wmbcIF->validDistance,
                              /* 75 totalTime                         */ wmbcIF->totalTime,
                              /* 76 validTime                         */ wmbcIF->validTime,
                              /* 77 validAlgoFrameNum                 */ wmbcIF->validAlgoFrameNum,
                              /* 78 validVehicleFrameNum              */ wmbcIF->validVehicleFrameNum,
                              /* 79 slcValidAlgoFrameNum              */ wmbcIF->slcValidAlgoFrameNum,
                              /* 80 convergedNum                      */ wmbcIF->convergedNum,
                              /* 81 lastConvFrame                     */ wmbcIF->lastConvFrame,
                              /* 82 quickMode                         */ 0
                              );
    }

      itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_FPA],
                              /* WMBC OutputValidation */
                              /* 04 camPort            */ _camInst,
                              /* 05 yawAngleDeg        */ d->results.curr.angles[e_YAW]*RAD2DEG,
                              /* 06 pitchAngleDeg      */ d->results.curr.angles[e_HORIZON]*RAD2DEG,
                              /* 07 rollAngleDeg       */ d->results.curr.angles[e_ROLL]*RAD2DEG,
                              /* 08 cameraHeight       */ d->results.curr.camh,
                              /* 09 yawLm2_f           */ d->results.curr.foeLm2.X() + d->camera.origin[_camInst].distDeltaLm2.X(),
                              /* 10 horizonLm2_f       */ d->results.curr.foeLm2.Y() + d->camera.origin[_camInst].distDeltaLm2.Y(),
                              /* 11 convYaw            */ d->algo.single[e_YAW].conv,
                              /* 12 convHorizon        */ d->algo.single[e_HORIZON].conv,
                              /* 13 convRoll           */ d->algo.single[e_ROLL].conv,
                              /* 14 convCamh           */ d->algo.single[e_CAM_HEIGHT].conv,
                              /* 15 totalConv          */ d->algo.total.conv,
                              /* 16 focalLm2           */ d->camera.focalLm2
                             );

  }

  void WmbcOutputIF::toItrkAutofix(int idx, int newYaw, int newHorizon, bool convYaw, bool convHorizon, 
                                   const AutofixState *as, const WmbcData* d) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    itrkWriter::writeRecord(_itrkHandle[WmbcItrkType::e_AFIX],
                            /* WMBC Autofix                        */
                            /* 04 camPort                          */ (CameraInfo::CameraInstance)idx,
                            /* 05 yawL0Init                        */ as->limits[idx].foeL0Init[e_YAW],
                            /* 06 horizonL0Init                    */ as->limits[idx].foeL0Init[e_HORIZON],
                            /* 07 yawAutofixL0Init                 */ as->limits[idx].foeAutofixL0Init[e_YAW],
                            /* 08 horizonAutofixL0Init             */ as->limits[idx].foeAutofixL0Init[e_HORIZON],
                            /* 09 yawUpperLimitL0                  */ as->limits[idx].foeUpperLimitL0[e_YAW],
                            /* 10 yawLowerLimitL0                  */ as->limits[idx].foeLowerLimitL0[e_YAW],
                            /* 11 horizonUpperLimitL0              */ as->limits[idx].foeUpperLimitL0[e_HORIZON],
                            /* 12 horizonLowerLimitL0              */ as->limits[idx].foeLowerLimitL0[e_HORIZON],
                            /* 13 yawOutOfLimit                    */ as->limits[idx].foeOutOfLimit[e_YAW],
                            /* 14 horizonOutOfLimit                */ as->limits[idx].foeOutOfLimit[e_HORIZON],
                            /* 15 currIsOkThreshold                */ as->limits[idx].currIsOkThreshold,
                            /* 16 yawCurrIsOk                      */ as->limits[idx].foeCurrIsOk[e_YAW],
                            /* 17 horizonCurrIsOk                  */ as->limits[idx].foeCurrIsOk[e_HORIZON],
                            /* 18 yawAutofixDeltaL0LastUpdated     */ as->limits[idx].foeAutofixDeltaL0LastUpdated[e_YAW],
                            /* 19 horizonAutofixDeltaL0LastUpdated */ as->limits[idx].foeAutofixDeltaL0LastUpdated[e_HORIZON],
                            /* 20 yawCounterOOL                    */ as->limits[idx].foeCounterOOL[e_YAW],
                            /* 21 horizonCounterOOL                */ as->limits[idx].foeCounterOOL[e_HORIZON],
                            /* 22 yawL0                            */ as->output[idx].foeL0[e_YAW],
                            /* 23 horizonL0                        */ as->output[idx].foeL0[e_HORIZON],
                            /* 24 yawAngle                         */ as->output[idx].foeAngle[e_YAW],
                            /* 25 pitchAngle                       */ as->output[idx].foeAngle[e_HORIZON],
                            /* 26 yawL0_prev                       */ as->outputPrevConv[idx].foeL0[e_YAW],
                            /* 27 horizonL0_prev                   */ as->outputPrevConv[idx].foeL0[e_HORIZON],
                            /* 28 yawAngle_prev                    */ as->outputPrevConv[idx].foeAngle[e_YAW],
                            /* 29 pitchAngle_prev                  */ as->outputPrevConv[idx].foeAngle[e_HORIZON],
                            /* 30 convYaw                          */ convYaw,
                            /* 31 convLastFrameYaw                 */ d->algo.single[e_YAW].convLastFrame,
                            /* 32 convHorizon                      */ convHorizon,
                            /* 33 convLastFrameHorizon             */ d->algo.single[e_HORIZON].convLastFrame,
                            /* 34 yawAngleRaw                      */ d->results.cams[idx].angles[e_YAW],
                            /* 35 pitchAngleRaw                    */ d->results.cams[idx].angles[e_HORIZON],
                            /* 36 newYaw                           */ newYaw,
                            /* 37 newHorizon                       */ newHorizon
                           );
  }

  void WmbcOutputIF::toCextTargetPts(SPInfoVec *sp, const Prep::SafeImg* img) {
#ifdef MEwin
    static ClipextIO::ClipextWriter writer(".slc");                                                                                                                              
    ClipextIO::ExpID expId = ClipextIO::CEXT_SLOW;
    writer.setExpID(expId);

    int numPts = std::min((int)sp->size(), 2*MAX_SP_NUM_IN_TARGET);

    Float::MEmath::Mat<2, 2*MAX_SP_NUM_IN_TARGET, float> ptsd, ptsr;
    for (int i = 0; i < numPts; ++i) {
      float xd = (*sp)[i].xSub;
      float yd = (*sp)[i].ySub;
      float xr = xd, yr = yd;
      if (DistortionCorrectionAPI::isDistortionValid()) {
        DistortionCorrectionAPI::rectifySafe(CameraInfo::e_FORWARD, -2, xd, yd, xr, yr);
      }
      ptsd(0, i) = xd;
      ptsd(1, i) = yd;
      ptsr(0, i) = xr;
      ptsr(1, i) = yr;
    }

    writer.setData("img", &img->getObj());
    writer.setData("sp", &ptsd);
    writer.setData("spRect", &ptsr);
    writer.flushTuple();
#endif
  }

} // namespace WMBC
