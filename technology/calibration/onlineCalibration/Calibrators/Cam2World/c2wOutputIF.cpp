/**
 * \file  c2wOutputIF.cpp
 * \brief Output Interface to itrk/stdout
 * 
 * \author Uri London
 * \date Jul 11, 2019
 */

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wOutputIF.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wData.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wSignal.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wUtils.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/planeSync.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationModelIF.h"
#include "basicTypes/mfl/common/itrkWriter.h"
#include "basicTypes/mfl/itrkWriter_API.h"
#include <stdarg.h> // va_start, va_end

// TODO: std::string to MEtl::string

namespace OnlineCalibration {
  namespace Cam2World {

  Cam2WorldOutputIF& Cam2WorldOutputIF::instance() {
    static Cam2WorldOutputIF& single = *new Cam2WorldOutputIF;
    return single;
  }

  Cam2WorldOutputIF::Cam2WorldOutputIF() {
    for (int i=0; i<e_ITRK_NUM_TYPES; ++i) {
      _itrkHandle[i] = nullptr;
    }
    _primaryCam = CameraInfo::e_FORWARD;
    _dbgPrintMask = Debug::Args::instance().getStickyValue("-sOCC2W-dbgPrintMask", 0);
    //toItrkHeaders();
  }

  // ------------------------------ itrk      ------------------------------------------
  void Cam2WorldOutputIF::toItrkHeaders() {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    if (_itrkHandle[ItrkType::e_OUTPUT]) {
      return;
    }

    MEtl::string headers[e_ITRK_NUM_TYPES];
    headers[e_INPUT_DATA]       = "OCC2W InputData focalLm2 isYawRateAvailable speedAvailable yawRate "
                                  "speed radius dt ds totalDistance totalTime accel emValid emStatus "
                                  "emConfT emConfR dyaw dpitch droll yaw pitch foeX foeY emValidFrameNum rmValid rmStatus rmInlinerNum "
                                  "planeIdx rmMessage Nx Ny Nz camh roll rmPitch pitchDiff "
                                  "rmAtA00  rmAtA01  rmAtA02  rmAtA11  rmAtA12 rmAtA22 "
                                  "rmNAtA00 rmNAtA01 rmNAtA02 rmNAtA11 rmNAtA12 rmNAtA22";
    headers[e_INPUT_DATA_CONF]  = "OCC2W InputDataConf rmsPts rmsOutliers "
                                  "rmsPtNumEma resConfMeanPitch resConfMeanStringentPitch "
                                  "residualPtNumPitch residualMeanPitch residualVarPitch residualMinPitch residualMaxPitch "
                                  "resConfMeanRoll resConfMeanStringentRoll residualPtNumRoll residualMeanRoll "
                                  "residualVarRoll residualMinRoll residualMaxRoll resConfMeanCamh resConfMeanStringentCamh "
                                  "residualPtNumCamh residualMeanCamh residualVarCamh residualMinCamh residualMaxCamh "
                                  "rmCovValid rmCov00 rmCov01 rmCov02 rmCov11 rmCov12 rmCov22 "
                                  "rmResConfMeanPitch rmResConfMeanRoll rmResConfMeanCamh "
                                  "rmResConfMean2Pitch rmResConfMean2Roll rmResConfMean2Camh "
                                  "maxMedChangeYaw maxMedChangePitch maxMedChangeRoll maxMedChangeCamh "
                                  "maxMedChangeYawStringent maxMedChangePitchStringent maxMedChangeRollStringent maxMedChangeCamhStringent "
                                  "minSteadyThreshYaw minSteadyThreshPitch minSteadyThreshRoll minSteadyThreshCamh "
                                  "minSteadyThreshYawStringent minSteadyThreshPitchStringent minSteadyThreshRollStringent minSteadyThreshCamhStringent "
                                  "confScale_y confScale_p confScale_r confScale_h "
                                  "varFinal_y varFinal_p varFinal_r varFinal_h";
    headers[e_FRAME_VALIDATION] = "OCC2W FrameValidation pauseReason vehicleValidFrame vehicleValidFrameNum "
                                  "vehicleValidDistance vehicleValidTime enforcingFirstValid minSpeed maxSpeed "
                                  "minRadius maxAccel maxYawRate hystEnabled hystMinRadiusRangeMin "
                                  "hystMinRadiusRangeMax hystMinRadiusRangeInc maxDYaw maxDPitch maxDRoll "
                                  "hystMaxDYawMax hystMaxDPitchMax hystMaxDRollMax hystMaxRotMin "
                                  "hystMaxRotInc maxPitchDiff maxCrownAngDiff";
    headers[e_SIGNAL]           = "OCC2W Signal id validFrame validFrameNum validDistance validTime "
                                  "confType confidence0 confidence1 confTimeout confErr confErrStringent sf result inRange invalidFrameMask "
                                  "stableMedianCount0 stableMedianCount1 unstableMedianCount0 unstableMedianCount1 unsteadySigCount0 unsteadySigCount1 degradeCause";
    headers[e_HIST]             = "OCC2W Histogram id mean var median prevMedian min max "
                                  "minBin maxBin currValExact currVal currBin sampleNum binSize iqr "
                                  "smaFast smvFast emaFast emvFast smmFast prevSmmFast autocovFast autocorrFast emaFacFast winSizeFast "
                                  "smaMedium smvMedium emaMedium emvMedium smmMedium prevSmmMedium autocovMedium autocorrMedium emaFacMedium winSizeMedium "
                                  "smaSlow smvSlow emaSlow emvSlow smmSlow prevSmmSlow autocovSlow autocorrSlow emaFacSlow winSizeSlow valid "
                                  "corrTime lagSamps";
    headers[e_PLANE]            = "OCC2W Plane valid roll camh location";
    headers[e_PLANE_BUFFER]     = "OCC2W PlaneBuffer bufferSize d1 d2 d3 d4 d5 d6 d7 d8 loc1 loc2 loc3 loc4 loc5 loc6 loc7 loc8 "
                                  "N01 N02 N03 N04 N05 N06 N07 N08 N11 N12 N13 N14 N15 N16 N17 N18 N21 N22 N23 N24 N25 N26 N27 N28";
    headers[e_RESULT]           = "OCC2W Result Rxx Rxy Rxz Ryx Ryy Ryz Rzx Rzy Rzz tx ty tz "
                                  "foeX_delta foeY_delta foeX_abs foeY_abs yaw pitch roll camh conf stringentConf degradeCause";
    headers[e_OUTPUT]           = "OCC2W Output Rxx Rxy Rxz Ryx Ryy Ryz Rzx Rzy Rzz tx ty tz "
                                  "foeX_delta foeY_delta roll conf stringentConf std inRange "
                                  "state innerState degradeCause";
    headers[e_OUTPUT_CONF]      = "OCC2W OutputConf conf stringentConf confLevel "
                                  "lowConfTh highConfTh highConfThMin highConfThMax "
                                  "highConfThIncrement stringentHighTh stateTimeCounter confLevelTimeCounter "
                                  "StableSig";
    headers[e_SPC]              = "OCC2W SPC spcMode maxAttempts sessionNum sessionFailed "
                                  "invalidSignalMask invalidFrameMask validFrame progress confProgress "
                                  "frameProgress conv status error yawFull horizonFull";
    headers[e_MODEL_IF]         = "OCC2W ModelIF C2W_yaw C2W_pitch C2W_roll C2W_cameraHeight C2W_state "
                                  "C2W_stateDegradeCause FS_C2W_OOR SPC_Status SPC_Progress SPC_Error "
                                  "SPC_Session_Number SPC_Frame_Valid SPC_Invalid_Signal "
                                  "SPC_Invalid_Reason SPC_Baseline_Yaw SPC_Baseline_Pitch "
                                  "SPC_Baseline_Height SPC_Baseline_Roll "
                                  "Rxx Rxy Rxz Ryx Ryy Ryz Rzx Rzy Rzz tx ty tz "
                                  "Rxx_hs Rxy_hs Rxz_hs Ryx_hs Ryy_hs Ryz_hs Rzx_hs Rzy_hs Rzz_hs tx_hs ty_hs tz_hs";

    for (int i=0; i<e_ITRK_NUM_TYPES; ++i) {
      _itrkHandle[i] = itrkWriter::registerItrkFormat(headers[i].c_str());
    }
  }




  void Cam2WorldOutputIF::toItrkInputData(const Cam2WorldData *d) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    CalibUtils::PixelLm2_f foe(0.f, 0.f);
    anglesToPixel(d->em.yaw, d->em.pitch, foe);

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_INPUT_DATA],
                            /* OCC2W InputData        */
                            /* 04 camPort             */ _primaryCam,
                            /* 05 focalLm2            */ d->cam.focalLm2,
                            /* 06 isYawRateAvailable  */ d->vehicle.isYawRateAvailable,
                            /* 07 speedAvailable      */ d->vehicle.speedAvailable,
                            /* 08 yawRate             */ d->vehicle.yawRate,
                            /* 09 speed               */ d->vehicle.speed,
                            /* 10 radius              */ d->vehicle.radius,
                            /* 11 dt                  */ d->vehicle.dt,
                            /* 12 ds                  */ d->vehicle.ds,
                            /* 13 totalDistance       */ d->vehicle.totalDistance,
                            /* 14 totalTime           */ d->vehicle.totalTime,
                            /* 15 accel               */ d->vehicle.accel,
                            /* 16 emValid             */ d->em.valid,
                            /* 17 emStatus            */ d->em.status,
                            /* 18 emConfT             */ d->em.confT,
                            /* 19 emConfR             */ d->em.confR,
                            /* 20 dyaw                */ d->em.rot[e_YAW],
                            /* 21 dpitch              */ d->em.rot[e_PITCH],
                            /* 22 droll               */ d->em.rot[e_ROLL],
                            /* 23 yaw                 */ d->em.yaw,
                            /* 24 pitch               */ d->em.pitch,
                            /* 25 foeX                */ foe[0],
                            /* 26 foeY                */ foe[1],
                            /* 27 emValidFrameNum     */ d->em.validFrameNum,
                            /* 28 rmValid             */ d->rm.valid,
                            /* 29 rmStatus            */ d->rm.status,
                            /* 30 rmInlinerNum        */ d->rm.numOfInliers,
                            /* 31 planeIdx            */ d->rm.planeIdx,
                            /* 32 rmMessage           */ d->rm.message,
                            /* 33 Nx                  */ d->rm.N[0],
                            /* 34 Ny                  */ d->rm.N[1],
                            /* 35 Nz                  */ d->rm.N[2],
                            /* 36 camh                */ d->rm.camh,
                            /* 37 roll                */ d->rm.roll,
                            /* 38 rmPitch             */ d->rm.pitch,
                            /* 39 pitchDiff           */ d->rm.pitchDiff,
                            /* 40 rmAtA00             */ d->rm.AtA(0, 0),
                            /* 41 rmAtA01             */ d->rm.AtA(0, 1),
                            /* 42 rmAtA02             */ d->rm.AtA(0, 2),
                            /* 43 rmAtA11             */ d->rm.AtA(1, 1),
                            /* 44 rmAtA12             */ d->rm.AtA(1, 2),
                            /* 45 rmAtA22             */ d->rm.AtA(2, 2),
                            /* 46 rmNAtA00            */ d->rm.nAtA(0, 0),
                            /* 47 rmNAtA01            */ d->rm.nAtA(0, 1),
                            /* 48 rmNAtA02            */ d->rm.nAtA(0, 2),
                            /* 49 rmNAtA11            */ d->rm.nAtA(1, 1),
                            /* 50 rmNAtA12            */ d->rm.nAtA(1, 2),
                            /* 51 rmNAtA22            */ d->rm.nAtA(2, 2)
                           );
                           
itrkWriter::writeRecord(_itrkHandle[ItrkType::e_INPUT_DATA_CONF],
                            /* OCC2W InputDataConf                      */
                            /* 04 camPort                              */ _primaryCam,
                            /* 05 rmsPts                               */ d->em.rmsPtNum[0],
                            /* 06 rmsOutliers                          */ d->em.rmsPtNum[1],
                            /* 07 rmsPtNumEma                          */ d->em.rmsPtNumEma,
                            /* 08 resConfMeanPitch                     */ d->rm.resConfMean[e_PITCH],
                            /* 09 resConfMeanStringentPitch            */ d->rm.resConfMeanStringent[e_PITCH],
                            /* 10 residualPtNumPitch                   */ d->rm.residualPtNum[e_PITCH],
                            /* 11 residualMeanPitch                    */ d->rm.residualMean[e_PITCH],
                            /* 12 residualVarPitch                     */ d->rm.residualVar[e_PITCH],
                            /* 13 residualMinPitch                     */ d->rm.residualMin[e_PITCH],
                            /* 14 residualMaxPitch                     */ d->rm.residualMax[e_PITCH],
                            /* 15 resConfMeanRoll                      */ d->rm.resConfMean[e_ROLL],
                            /* 16 resConfMeanStringentRoll             */ d->rm.resConfMeanStringent[e_ROLL],
                            /* 17 residualPtNumRoll                    */ d->rm.residualPtNum[e_ROLL],
                            /* 18 residualMeanRoll                     */ d->rm.residualMean[e_ROLL],
                            /* 19 residualVarRoll                      */ d->rm.residualVar[e_ROLL],
                            /* 20 residualMinRoll                      */ d->rm.residualMin[e_ROLL],
                            /* 21 residualMaxRoll                      */ d->rm.residualMax[e_ROLL],
                            /* 22 resConfMeanCamh                      */ d->rm.resConfMean[e_CAM_HEIGHT],
                            /* 23 resConfMeanStringentCamh             */ d->rm.resConfMeanStringent[e_CAM_HEIGHT],
                            /* 24 residualPtNumCamh                    */ d->rm.residualPtNum[e_CAM_HEIGHT],
                            /* 25 residualMeanCamh                     */ d->rm.residualMean[e_CAM_HEIGHT],
                            /* 26 residualVarCamh                      */ d->rm.residualVar[e_CAM_HEIGHT],
                            /* 27 residualMinCamh                      */ d->rm.residualMin[e_CAM_HEIGHT],
                            /* 28 residualMaxCamh                      */ d->rm.residualMax[e_CAM_HEIGHT],
                            /* 29 rmCovValid                           */ d->rm.validCov,
                            /* 30 rmCov00                              */ d->rm.cov(0, 0),
                            /* 31 rmCov01                              */ d->rm.cov(0, 1),
                            /* 32 rmCov02                              */ d->rm.cov(0, 2),
                            /* 33 rmCov11                              */ d->rm.cov(1, 1),
                            /* 34 rmCov12                              */ d->rm.cov(1, 2),
                            /* 35 rmCov22                              */ d->rm.cov(2, 2),
                            /* 36 rmResConfMeanPitch                   */ d->rm.resConfMean[e_PITCH],
                            /* 37 rmResConfMeanRoll                    */ d->rm.resConfMean[e_ROLL],
                            /* 38 rmResConfMeanCamh                    */ d->rm.resConfMean[e_CAM_HEIGHT],
                            /* 39 rmResConfMean2Pitch                  */ d->rm.resConfMeanStringent[e_PITCH],
                            /* 40 rmResConfMean2Roll                   */ d->rm.resConfMeanStringent[e_ROLL],
                            /* 41 rmResConfMean2Camh                   */ d->rm.resConfMeanStringent[e_CAM_HEIGHT],
                            /* 42 maxMedChangeYaw                      */ d->confParams[e_YAW][0].maxMedianChange,
                            /* 43 maxMedChangePitch                    */ d->confParams[e_PITCH][0].maxMedianChange,
                            /* 44 maxMedChangeRoll                     */ d->confParams[e_ROLL][0].maxMedianChange,
                            /* 45 maxMedChangeCamh                     */ d->confParams[e_CAM_HEIGHT][0].maxMedianChange,
                            /* 46 maxMedChangeYawStringent             */ d->confParams[e_YAW][1].maxMedianChange,
                            /* 47 maxMedChangePitchStringent           */ d->confParams[e_PITCH][1].maxMedianChange,
                            /* 48 maxMedChangeRollStringent            */ d->confParams[e_ROLL][1].maxMedianChange,
                            /* 49 maxMedChangeCamhStringent            */ d->confParams[e_CAM_HEIGHT][1].maxMedianChange,
                            /* 50 minSteadyThreshYaw                   */ d->confParams[e_YAW][0].minSteadyThresh,
                            /* 51 minSteadyThreshPitch                 */ d->confParams[e_PITCH][0].minSteadyThresh,
                            /* 52 minSteadyThreshRoll                  */ d->confParams[e_ROLL][0].minSteadyThresh,
                            /* 53 minSteadyThreshCamh                  */ d->confParams[e_CAM_HEIGHT][0].minSteadyThresh,
                            /* 54 minSteadyThreshYawStringent          */ d->confParams[e_YAW][1].minSteadyThresh,
                            /* 55 minSteadyThreshPitchStringent        */ d->confParams[e_PITCH][1].minSteadyThresh,
                            /* 56 minSteadyThreshRollStringent         */ d->confParams[e_ROLL][1].minSteadyThresh,
                            /* 57 minSteadyThreshCamhStringent         */ d->confParams[e_CAM_HEIGHT][1].minSteadyThresh,
                            /* 58 confScale_y                          */ d->confParams[e_YAW][0].confScale,
                            /* 59 confScale_p                          */ d->confParams[e_PITCH][0].confScale,
                            /* 60 confScale_r                          */ d->confParams[e_ROLL][0].confScale,
                            /* 61 confScale_h                          */ d->confParams[e_CAM_HEIGHT][0].confScale,
                            /* 62 varFinal_y                           */ d->em.sigVar[e_YAW],
                            /* 63 varFinal_p                           */ d->em.sigVar[e_PITCH],
                            /* 64 varFinal_r                           */ d->em.sigVar[e_ROLL],
                            /* 65 varFinal_h                           */ d->em.sigVar[e_CAM_HEIGHT]
                           );
  }

  void Cam2WorldOutputIF::toItrkFrameValidation(const Cam2WorldData *d) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_FRAME_VALIDATION],
                            /* OCC2W FrameValidation    */
                            /* 04 camPort               */ _primaryCam,
                            /* 05 pauseReason           */ d->algo.pauseReason,
                            /* 06 vehicleValidFrame     */ d->vehicle.validFrame,
                            /* 07 vehicleValidFrameNum  */ d->vehicle.validFrameNum,
                            /* 08 vehicleValidDistance  */ d->vehicle.validDistance,
                            /* 09 vehicleValidTime      */ d->vehicle.validTime,
                            /* 10 enforcingFirstValid   */ d->algo.enforcingFirstValid,
                            /* 11 minSpeed              */ d->validParams.minSpeed,
                            /* 12 maxSpeed              */ d->validParams.maxSpeed,
                            /* 13 minRadius             */ d->validParams.minRadius,
                            /* 14 maxAccel              */ d->validParams.maxAccel,
                            /* 15 maxYawRate            */ d->validParams.maxYawRate,
                            /* 16 hystEnabled           */ d->validParams.hystEnabled,
                            /* 17 hystMinRadiusRangeMin */ d->validParams.hystMinRadiusRange[0],
                            /* 18 hystMinRadiusRangeMax */ d->validParams.hystMinRadiusRange[1],
                            /* 19 hystMinRadiusRangeInc */ d->validParams.hystMinRadiusRange[2],
                            /* 20 maxDYaw               */ d->validParams.maxEmRot[e_YAW],
                            /* 21 maxDPitch             */ d->validParams.maxEmRot[e_PITCH],
                            /* 22 maxDRoll              */ d->validParams.maxEmRot[e_ROLL],
                            /* 23 hystMaxDYawMax        */ d->validParams.hystRotThMax[e_YAW],
                            /* 24 hystMaxDPitchMax      */ d->validParams.hystRotThMax[e_PITCH],
                            /* 25 hystMaxDRollMax       */ d->validParams.hystRotThMax[e_ROLL],
                            /* 26 hystMaxRotMin         */ d->validParams.hystRotThMin,
                            /* 27 hystMaxRotInc         */ d->validParams.hystRotThInc,
                            /* 28 maxPitchDiff          */ 0.f,
                            /* 29 maxCrownAngDiff       */ 0.f
                           );

  }

  void Cam2WorldOutputIF::toItrkSignal(const Cam2WorldSignal *s) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_SIGNAL],
                            /* OCC2W Signal             */
                            /* 04 camPort               */ _primaryCam,
                            /* 05 id                    */ (float)(s->id()),
                            /* 06 validFrame            */ s->validFrame(),
                            /* 07 validFrameNum         */ s->validFrameNum(),
                            /* 08 validDistance         */ s->validDistance(),
                            /* 09 validTime             */ s->validTime(),
                            /* 10 confType              */ s->confType(),
                            /* 11 confidence0           */ s->confidence(0),
                            /* 12 confidence1           */ s->confidence(1),
                            /* 13 confTimeout           */ s->confTimeout(),
                            /* 14 confErr               */ s->confErr(0),
                            /* 15 confErrStringent      */ s->confErr(1),
                            /* 16 sf                    */ s->sf(),
                            /* 17 result                */ s->result(),
                            /* 18 inRange               */ s->inRange(),
                            /* 19 invalidFrameMask      */ s->invalidFrameMask(),
                            /* 20 stableMedianCount0    */ s->stableMedianCount(0),
                            /* 21 stableMedianCount1    */ s->stableMedianCount(1),
                            /* 22 unstableMedianCount0  */ s->unstableMedianCount(0),
                            /* 23 unstableMedianCount1  */ s->unstableMedianCount(1),
                            /* 24 unsteadySigCount0     */ s->unsteadySigCount(0),
                            /* 25 unsteadySigCount1     */ s->unsteadySigCount(1),
                            /* 26 degradeCause          */ s->degradeCause()
                           );

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_HIST],
                            /* OCC2W Histogram   */
                            /* 04 camPort        */ _primaryCam,
                            /* 05 id             */ s->id(),
                            /* 06 mean           */ s->hist().mean(),
                            /* 07 var            */ s->hist().var(),
                            /* 08 median         */ s->hist().median(),
                            /* 09 prevMedian     */ s->hist().prevMedian(),
                            /* 10 min            */ s->hist().min(),
                            /* 11 max            */ s->hist().max(),
                            /* 12 minBin         */ s->hist().minBin(),
                            /* 13 maxBin         */ s->hist().maxBin(),
                            /* 14 currValExact   */ s->hist().currVal(),
                            /* 15 currVal        */ s->hist().currValHist(),
                            /* 16 currBin        */ s->hist().currBin(),
                            /* 17 sampleNum      */ s->hist().sampleNum(),
                            /* 18 binSize        */ s->hist().binSize(),
                            /* 19 iqr            */ s->hist().iqr(),
                            /* 20 smaFast        */ s->hist().sma(e_FAST),
                            /* 21 smvFast        */ s->hist().smv(e_FAST),
                            /* 22 emaFast        */ s->hist().ema(e_FAST),
                            /* 23 emvFast        */ s->hist().emv(e_FAST),
                            /* 24 smmFast        */ s->hist().smm(e_FAST),
                            /* 25 prevSmmFast    */ s->hist().prevSmm(e_FAST),
                            /* 26 autocovFast    */ s->hist().autocov(e_FAST),
                            /* 27 autocorrFast   */ s->hist().autocorr(e_FAST),
                            /* 28 emaFacFast     */ s->hist().emaFac(e_FAST),
                            /* 29 winSizeFast    */ s->hist().winSize(e_FAST),
                            /* 30 smaMedium      */ s->hist().sma(e_MEDIUM),
                            /* 31 smvMedium      */ s->hist().smv(e_MEDIUM),
                            /* 32 emaMedium      */ s->hist().ema(e_MEDIUM),
                            /* 33 emvMedium      */ s->hist().emv(e_MEDIUM),
                            /* 34 smmMedium      */ s->hist().smm(e_MEDIUM),
                            /* 35 prevSmmMedium  */ s->hist().prevSmm(e_MEDIUM),
                            /* 36 autocovMedium  */ s->hist().autocov(e_MEDIUM),
                            /* 37 autocorrMedium */ s->hist().autocorr(e_MEDIUM),
                            /* 38 emaFacMedium   */ s->hist().emaFac(e_MEDIUM),
                            /* 39 winSizeMedium  */ s->hist().winSize(e_MEDIUM),
                            /* 40 smaSlow        */ s->hist().sma(e_SLOW),
                            /* 41 smvSlow        */ s->hist().smv(e_SLOW),
                            /* 42 emaSlow        */ s->hist().ema(e_SLOW),
                            /* 43 emvSlow        */ s->hist().emv(e_SLOW),
                            /* 44 smmSlow        */ s->hist().smm(e_SLOW),
                            /* 45 prevSmmSlow    */ s->hist().prevSmm(e_SLOW),
                            /* 46 autocovFast    */ s->hist().autocov(e_SLOW),
                            /* 47 autocorrFast   */ s->hist().autocorr(e_SLOW),
                            /* 48 emaFacSlow     */ s->hist().emaFac(e_SLOW),
                            /* 49 winSizeSlow    */ s->hist().winSize(e_SLOW),
                            /* 50 valid          */ s->hist().valid(),
                            /* 51 corrTime       */ s->hist().corrTime(),
                            /* 52 lagSamps       */ s->hist().lagSamps()
                           );
  }

 void Cam2WorldOutputIF::toItrkPlane(const Cam2WorldData *d) const {
   if (!itrkWriter::isItrkActive()) {
     return;
   }
   
   itrkWriter::writeRecord(_itrkHandle[ItrkType::e_PLANE],
                           /* OCC2W Plane              */
                           /* 04 camPort               */ _primaryCam,
                           /* 05 valid                 */ d->plane.valid,
                           /* 06 roll                  */ d->plane.roll,
                           /* 07 camh                  */ d->plane.camh,
                           /* 08 location              */ d->plane.location
                           );
 }

 void Cam2WorldOutputIF::toItrkPlaneBuffer(const MEtypes::FastCyclicVector<Plane> *p) const {
   if (!itrkWriter::isItrkActive()) {
     return;
   }

   float d[PLANE_BUFFER_CAPACITY];
   float loc[PLANE_BUFFER_CAPACITY];
   float N0[PLANE_BUFFER_CAPACITY];
   float N1[PLANE_BUFFER_CAPACITY];
   float N2[PLANE_BUFFER_CAPACITY];
   for (int i=0; i<PLANE_BUFFER_CAPACITY; ++i) {
     d[i]=0;
     loc[i]=0;
     N0[i]=0;
     N1[i]=0;
     N2[i]=0;
   }
   int i=0;
   for (MEtypes::FastCyclicVector<Plane>::const_iterator iter = p->begin(); iter!= p->end(); iter++) {
     d[i]=iter->d;
     loc[i]=iter->location;
     N0[i]=iter->N[0];
     N1[i]=iter->N[1];
     N2[i]=iter->N[2];
     i++;
   }
   
   itrkWriter::writeRecord(_itrkHandle[ItrkType::e_PLANE_BUFFER],
                           /* OCC2W Plane              */
                           /* 04 camPort               */ _primaryCam,
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
                           /* 22 loc8                  */ loc[7],
                           /* 23 N01                   */ N0[0],
                           /* 24 N02                   */ N0[1],
                           /* 25 N03                   */ N0[2],
                           /* 26 N04                   */ N0[3],
                           /* 27 N05                   */ N0[4],
                           /* 28 N06                   */ N0[5],
                           /* 29 N07                   */ N0[6],
                           /* 30 N08                   */ N0[7],
                           /* 31 N11                   */ N1[0],
                           /* 32 N12                   */ N1[1],
                           /* 33 N13                   */ N1[2],
                           /* 34 N14                   */ N1[3],
                           /* 35 N15                   */ N1[4],
                           /* 36 N16                   */ N1[5],
                           /* 37 N17                   */ N1[6],
                           /* 38 N18                   */ N1[7],
                           /* 39 N21                   */ N2[0],
                           /* 40 N22                   */ N2[1],
                           /* 41 N23                   */ N2[2],
                           /* 42 N24                   */ N2[3],
                           /* 43 N25                   */ N2[4],
                           /* 44 N26                   */ N2[5],
                           /* 45 N27                   */ N2[6],
                           /* 46 N28                   */ N2[7]
                           );
 }


  void Cam2WorldOutputIF::toItrkResult(const Cam2WorldData *d) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_RESULT],
                            /* OCC2W Result     */
                            /* 04 camPort       */ _primaryCam,
                            /* 05 Rxx           */ d->results.R(0, 0),
                            /* 06 Rxy           */ d->results.R(0, 1),
                            /* 07 Rxz           */ d->results.R(0, 2),
                            /* 08 Ryx           */ d->results.R(1, 0),
                            /* 09 Ryy           */ d->results.R(1, 1),
                            /* 10 Ryz           */ d->results.R(1, 2),
                            /* 11 Rzx           */ d->results.R(2, 0),
                            /* 12 Rzy           */ d->results.R(2, 1),
                            /* 13 Rzz           */ d->results.R(2, 2),
                            /* 14 tx            */ d->results.t[0],
                            /* 15 ty            */ d->results.t[1],
                            /* 16 tz            */ d->results.t[2],
                            /* 17 foeX_delta    */ d->results.foeDelta.X(),
                            /* 18 foeY_delta    */ d->results.foeDelta.Y(),
                            /* 19 foeX_abs      */ d->results.foe.X(),
                            /* 20 foeY_abs      */ d->results.foe.Y(),
                            /* 21 yaw           */ d->results.yawAngle,
                            /* 22 pitch         */ d->results.pitchAngle,
                            /* 23 roll          */ d->results.rollAngle,
                            /* 24 camh          */ -d->results.t[1],
                            /* 25 conf          */ d->results.confidence,
                            /* 26 stringentConf */ d->results.stringentConfidence,
                            /* 27 degradeCause  */ d->algo.degradeCause
                           );

  }

  void Cam2WorldOutputIF::toItrkOutput(State state, int degradeCause, unsigned int highConfTh,
                                       unsigned int stateTimeCounter, unsigned int confLevelTimeCounter,
                                       unsigned int lowConfTh, unsigned int highConfParams[4],
                                       ConfLevel confLevel, State innerState,
                                       const ExtrinsicCalibration &ec, const Cam2WorldStateInfo &si) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    // float roll = 0.f;
    // CalibUtils::PixelLm2_f foe(0.f, 0.f);
    // rotationMatrixToRodriguezPixel(ec.getR(), foe, roll);

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_OUTPUT],
                            /* OCC2W Output     */
                            /* 04 camPort       */        _primaryCam,
                            /* 05 Rxx           */        ec.getR()[0],
                            /* 06 Rxy           */        ec.getR()[1],
                            /* 07 Rxz           */        ec.getR()[2],
                            /* 08 Ryx           */        ec.getR()[3],
                            /* 09 Ryy           */        ec.getR()[4],
                            /* 10 Ryz           */        ec.getR()[5],
                            /* 11 Rzx           */        ec.getR()[6],
                            /* 12 Rzy           */        ec.getR()[7],
                            /* 13 Rzz           */        ec.getR()[8],
                            /* 14 tx            */        ec.getT()[0],
                            /* 15 ty            */        ec.getT()[1],
                            /* 16 tz            */        ec.getT()[2],
                            /* 17 foeX_delta    */        0.f, // foe[0], //WHAT IS THIS? WHY =0?
                            /* 18 foeY_delta    */        0.f, // foe[1],
                            /* 19 roll          */        0.f, // roll,
                            /* 20 conf          */        si.getConfidence(),
                            /* 21 stringentConf */        si.getStringentConfidence(),
                            /* 22 std           */        si.getStd(),
                            /* 23 inRange       */        si.getInRange(),
                            /* 24 state         */        (float)state,
                            /* 25 innerState    */        (float)innerState,
                            /* 26 degradeCause  */        degradeCause
                           );

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_OUTPUT_CONF],
                            /* OCC2W Output     */
                            /* 04 camPort       */        _primaryCam,
                            /* 05 conf          */        si.getConfidence(),
                            /* 06 stringentConf */        si.getStringentConfidence(),
                            /* 07 confLevel     */        confLevel,
                            /* 08 lowConfTh     */        lowConfTh,
                            /* 09 highConfTh    */        highConfTh,
                            /* 10 highConfThMin */        highConfParams[0],
                            /* 11 highConfThMax */        highConfParams[1],
                            /* 12 highConfThIncrement */  highConfParams[2],
                            /* 13 stringentHighTh */      highConfParams[3],
                            /* 14 stateTimeCounter */     stateTimeCounter,
                            /* 15 confLevelTimeCounter */ confLevelTimeCounter,
                            /* 16 StableSig */            si.getStableSig()
                           );

  }

  void Cam2WorldOutputIF::toItrkSPC(const Cam2WorldData *d) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_SPC],
                            /* OCC2W SPC            */
                            /* 04 camPort           */ _primaryCam,
                            /* 05 spcMode           */ d->spc.spcMode,
                            /* 06 maxAttempts       */ d->spc.maxAttempts,
                            /* 07 sessionNum        */ d->spc.sessionNum,
                            /* 08 sessionFailed     */ d->spc.sessionFailed,
                            /* 09 invalidSignalMask */ d->spc.invalidSignalMask,
                            /* 10 invalidFrameMask  */ d->spc.invalidFrameMask,
                            /* 11 validFrame        */ d->spc.validFrame,
                            /* 12 progress          */ d->spc.progress,
                            /* 13 confProgress      */ d->spc.conf_progress,
                            /* 14 frameProgress     */ d->spc.frame_progress,
                            /* 15 conv              */ d->spc.conv,
                            /* 16 status            */ d->spc.status,
                            /* 17 error             */ d->spc.error,
                            /* 18 yawFull           */ d->spc.foe[0],
                            /* 19 horizonFull       */ d->spc.foe[1]
                           );
  }


  void Cam2WorldOutputIF::toItrkSPC(SpcData spc) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_SPC],
                            /* OCC2W SPC            */
                            /* 04 camPort           */ _primaryCam,
                            /* 05 spcMode           */ spc.spcMode,
                            /* 06 maxAttempts       */ spc.maxAttempts,
                            /* 07 sessionNum        */ spc.sessionNum,
                            /* 08 sessionFailed     */ spc.sessionFailed,
                            /* 09 invalidSignalMask */ spc.invalidSignalMask,
                            /* 10 invalidFrameMask  */ spc.invalidFrameMask,
                            /* 11 validFrame        */ spc.validFrame,
                            /* 12 progress          */ spc.progress,
                            /* 13 confProgress      */ spc.conf_progress,
                            /* 14 frameProgress     */ spc.frame_progress,
                            /* 15 conv              */ spc.conv,
                            /* 16 status            */ spc.status,
                            /* 17 error             */ spc.error,
                            /* 18 yawFull           */ spc.foe[0],
                            /* 19 horizonFull       */ spc.foe[1]
                           );
  }

  void Cam2WorldOutputIF::toItrkModelIF(const OnlineCalibrationModelIF *m) const {
    if (!itrkWriter::isItrkActive()) {
      return;
    }

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_MODEL_IF],
                            /* OCC2W ModelIF            */
                            /* 04 camPort               */ _primaryCam,
                            /* 05 C2W_yaw               */ m->data.C2W_yaw,
                            /* 06 C2W_pitch             */ m->data.C2W_pitch,
                            /* 07 C2W_roll              */ m->data.C2W_roll,
                            /* 08 C2W_cameraHeight      */ m->data.C2W_cameraHeight,
                            /* 09 C2W_state             */ m->data.C2W_state,
                            /* 10 C2W_stateDegradeCause */ m->data.C2W_stateDegradeCause,
                            /* 11 FS_C2W_OOR            */ m->data.FS_C2W_Calibration_Out_Of_Range,
                            /* 12 SPC_Status            */ m->data.SPC_Status,
                            /* 13 SPC_Progress          */ m->data.SPC_Progress,
                            /* 14 SPC_Error             */ m->data.SPC_Error,
                            /* 15 SPC_Session_Number    */ m->data.SPC_Session_Number,
                            /* 16 SPC_Frame_Valid       */ m->data.SPC_Frame_Valid,
                            /* 17 SPC_Invalid_Signal    */ m->data.SPC_Invalid_Signal,
                            /* 18 SPC_Invalid_Reason    */ m->data.SPC_Invalid_Reason,
                            /* 19 SPC_Baseline_Yaw      */ m->data.SPC_Baseline_Yaw,
                            /* 20 SPC_Baseline_Pitch    */ m->data.SPC_Baseline_Pitch,
                            /* 21 SPC_Baseline_Height   */ m->data.SPC_Baseline_Height,
                            /* 22 SPC_Baseline_Roll     */ m->data.SPC_Baseline_Roll,
                            /* 23 Rxx                   */ m->data.C2W_R[0], //(0,0),
                            /* 24 Rxy                   */ m->data.C2W_R[1], //(0,1),
                            /* 25 Rxz                   */ m->data.C2W_R[2], //(0,2),
                            /* 26 Ryx                   */ m->data.C2W_R[3], //(1,0),
                            /* 27 Ryy                   */ m->data.C2W_R[4], //(1,1),
                            /* 28 Ryz                   */ m->data.C2W_R[5], //(1,2),
                            /* 29 Rzx                   */ m->data.C2W_R[6], //(2,0),
                            /* 30 Rzy                   */ m->data.C2W_R[7], //(2,1),
                            /* 31 Rzz                   */ m->data.C2W_R[8], //(2,2),
                            /* 32 tx                    */ m->data.C2W_T[0],
                            /* 33 ty                    */ m->data.C2W_T[1],
                            /* 34 tz                    */ m->data.C2W_T[2],
                            /* 35 Rxx_hs                */ m->data.C2W_HighState_R[0], //(0,0),
                            /* 36 Rxy_hs                */ m->data.C2W_HighState_R[1], //(0,1),
                            /* 37 Rxz_hs                */ m->data.C2W_HighState_R[2], //(0,2),
                            /* 38 Ryx_hs                */ m->data.C2W_HighState_R[3], //(1,0),
                            /* 39 Ryy_hs                */ m->data.C2W_HighState_R[4], //(1,1),
                            /* 40 Ryz_hs                */ m->data.C2W_HighState_R[5], //(1,2),
                            /* 41 Rzx_hs                */ m->data.C2W_HighState_R[6], //(2,0),
                            /* 42 Rzy_hs                */ m->data.C2W_HighState_R[7], //(2,1),
                            /* 43 Rzz_hs                */ m->data.C2W_HighState_R[8], //(2,2),
                            /* 44 tx_hs                 */ m->data.C2W_HighState_T[0],
                            /* 45 ty_hs                 */ m->data.C2W_HighState_T[1],
                            /* 46 tz_hs                 */ m->data.C2W_HighState_T[2]
                           );
  }


  // ------------------------------ printouts ------------------------------------------
  void Cam2WorldOutputIF::tostdout(int mask, int clr, std::string format, ...) {
    if ((_dbgPrintMask & mask) == 0) {
      return;
    }

    va_list args;
    va_start(args, format);
    // c++11
    // format = "\033[" + std::to_string(clr) + "m" + format + "\033[0m\n";
    // vfprintf(stdout, format.c_str(), args);

    // c++98
    std::stringstream ss;
    ss << "\033[" << clr << "m"  << format  << "\033[0m\n";
    vfprintf(stdout, ss.str().c_str(), args);
    va_end(args);
  }

  void Cam2WorldOutputIF::tostdoutRT(int mask, int clr, std::string header,
                                       Float::MEmath::Mat<3,3,float> R,
                                       Float::MEmath::Vec<3,float> t) {
    // TODO: esthetics
    std::string format = header;
    format += "\nR = [%.3f %.3f %.3f\n"
              "     %.3f %.3f %.3f\n"
              "     %.3f %.3f %.3f]\n";
    format += "t = [%.3f %.3f %.3f]";
    tostdout(mask, clr, format,
             R(0,0), R(0,1), R(0,2),
             R(1,0), R(1,1), R(1,2),
             R(2,0), R(2,1), R(2,2),
             t[0], t[1], t[2]);
  }

  void Cam2WorldOutputIF::tostdoutMatf(int mask, int clr, std::string header,
                                       CalibUtils::mat33f M) {
    // TODO: esthetics
    std::string format = header;
    format += "\nM = [%.3f %.3f %.3f\n"
              "     %.3f %.3f %.3f\n"
              "     %.3f %.3f %.3f]\n";
    tostdout(mask, clr, format,
             M(0,0), M(0,1), M(0,2),
             M(1,0), M(1,1), M(1,2),
             M(2,0), M(2,1), M(2,2));
  }

  } // namespace Cam2World
} // namespace OnlineCalibration
