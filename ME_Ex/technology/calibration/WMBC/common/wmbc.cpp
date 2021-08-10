/*
 * WMBC.cpp
 *
 *  Created on: Jun 08, 2016
 *      Author: urilo
 */

#include "wmbc.h"
#include "utilities/vehicleInformation/vehicleInformation_API.h"
#include "technology/mobilib/fix/common/MEXmisc/timeStampUtils.h"
#include "functionality/interface/modelIF.h"
#include <math.h>
#include <numeric>
#include "technology/brain2/prepSys/prepSys_API.h"
#include "utilities/egoMotion/egoMotion_API.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "utilities/cameraInformation/common/cameraProperties.h"
#include "technology/calibration/utilities/cameraModel/cameraModel_API.h"
#include "basicTypes/transformationMatrices_API.h"
#include "technology/calibration/utilities/cameraProjections/distortionModel_API.h"
#include "technology/calibration/utilities/cameraProjections/distortionCorrectionAPI.h"
#include "technology/calibration/WMBC/common/wmbcOutputIF.h"
#include "technology/calibration/WMBC/common/wmbcProperties.h"
#include "technology/mobilib/std/math/sqrt.h"
#include "technology/worldModel/common/worldModelUtils.h"
#include "technology/SFM2/SFM2_API.h"
#include "technology/calibration/WMBC/common/wmbc_dbg.h"
#include "technology/calibration/WMBC/common/wmbcUtils.h"
#include "utilities/autoFix/autoFix_API.h"
#include "utilities/autoFix/common/autoFixProperties.h"
#include "technology/calibration/cameraToCamera/cameraToCamera_API_internal.h"
#include "technology/worldModel/egoMotion/EgomotionPrivate_API.h"
#include "technology/calibration/WMBC/wmbc_ServicesUser.h"
#include "utilities/SDM/safetyDiagnosticManager_API.h"
#include "technology/brain2/brain2_API.h"
#include "technology/failSafes/interface/failSafesIF.h"

#define WMBC_SERIAL_PARAM(x, i) ((int)((x).actualSize) > i ? (x)._seriesArray[i] : (x)._seriesArray[(x).actualSize-1])
#define WMBC_CAM_EXIST(i) (CameraInfo::exists((CameraInfo::CameraInstance)i))
#define WMBC_CAM_CEXIST(i) (CameraInfo::calibrationExists((CameraInfo::CameraInstance)i))
#define WMBC_CAM_OK(i) (WMBC_CAM_EXIST(i) && WMBC_CAM_CEXIST(i))
#define WMBC_CAM_SKIP(i) if (!WMBC_CAM_OK(i)) { continue;}
#define WMBC_CAM_ABORT(i) if (!WMBC_CAM_OK(i)) { return;}


namespace WMBC {

  FOEFinder::FOEFinder(const WmbcProperties *properties) : _wmbcIF_wrapper(&_wmbcIF) {
    setParams(properties);
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      _variable[i].id((CALIB_DOF)i);
      _variable[i].binSize(HIST_BIN_SIZE[i]);
      _variable[i].invalidFrameMask(INVALID_MASK[i]);
      _variable[i].debugPrintMask(_debugPrintMask);
    }
    WmbcOutputIF::instance().toItrkHeaders();
    reset();
    setCameraDataInit();
    _data.results.focalLm2(_data.camera.focalLm2);
    resetResultsToEtc();
  }

  FOEFinder::~FOEFinder() {
    delete &WmbcOutputIF::instance();
  }

  void FOEFinder::reset() {
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      _variable[i].reset();
    }
    _data.reset(); // todo: is there need distinguish reset() and fullReset() are there vars that need to be saved?
    // TODO: move into rm.reset()
    _data.rm.crownAngDiffMean = 0.f;
    _data.rm.crownAngDiffVar  = 0.f;
    _data.rm.crownAngDiffMin  = 1.f;
    _data.rm.crownAngDiffMax  = 0.f;
    _data.em.rmsPtNumMean = 0.f;
  }

  void FOEFinder::reset(int idx) {
    _variable[idx].reset();
    _data.algo.single[idx].reset();
  }

  void FOEFinder_SPC::restartSession() {
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      if (_variable[i].conv()) {
        continue;
      }
      _variable[i].reset();
    }
    _data.algo.total.validFrame = false;
    // _data.algo.total.progress = 0;
    _data.algo.sessionNum++;
  }

  void FOEFinder::initImages(const Prep::SafeImg* imgDist, const Prep::SafeImg* imgTargets, int level) {
    _data.algo.img = imgDist;
    _data.target.img = imgTargets;
    _data.target.level = level;
  }

  void FOEFinder::setParams(const WmbcProperties *properties) { // TODO: use properties and consts
    // int focalLm2 = CameraInfo::focalLength(CameraInfo::e_FORWARD) * 4;

    ValidParams &vp = _data.validParams;
    MetaParams &mp = _data.metaParams;
    ConvParams &cp = _data.convParams;

    // parameters controlling frame validation
    vp.minSpeed = properties->wmbcMinSpeed();
    vp.maxSpeed = properties->wmbcMaxSpeed();
    vp.maxAccel = properties->wmbcMaxAccel();
    vp.maxYawRate = properties->wmbcMaxYawRate();
    vp.minRadius = properties->wmbcMinRadius();
    for (int i = 0; i < 3; ++i) {
      vp.rotThreshold[i] = WMBC_SERIAL_PARAM(properties->wmbcMaxRotation(), i);
    }
    vp.maxPitchDiff = properties->wmbcMaxPitchDiff();
    vp.maxCrownAngDiff = properties->wmbcMaxCrownAngDiff()*DEG2RAD;
    vp.useSpeedHighPrecision = properties->wmbcUseSpeedHighPrecision();
    vp.useDynamicSuspension = properties->wmbcUseDynamicSuspension();

    static int delayValidThreshold = Debug::Args::instance().getStickyValue("-sWmbc-delayValidThreshold", DELAY_VALID_TH_DEFAULT);
    vp.delayValidThreshold = delayValidThreshold;

    vp.hystEnabled = (properties->wmbcMode() != e_SLC && !Debug::Args::instance().existsParameter("-sWMBC-hystDisable"));
    static float hystMinRadiusInc = Debug::Args::instance().getStickyValue("-sWmbc-hystMinRadiusInc", 50);
    static float hystMinRadiusMax = Debug::Args::instance().getStickyValue("-sWmbc-hystMinRadiusMax", 1000);
    vp.hystMinRadiusRange[0] = vp.minRadius;
    vp.hystMinRadiusRange[1] = hystMinRadiusMax;
    vp.hystMinRadiusRange[2] = hystMinRadiusInc;

    static float hystRotThInc = Debug::Args::instance().getStickyValue("-sWmbc-hystRotThInc", 0.00025f);
    static float hystRotThMin = Debug::Args::instance().getStickyValue("-sWmbc-hystRotThMin", 0.001f);
    vp.hystRotThMin = hystRotThMin;
    vp.hystRotThInc = hystRotThInc;
    for (int i = 0; i < 3; ++i) {
      vp.hystRotThMax[i] = vp.rotThreshold[i];
    }
    vp.maxEpiErr = Debug::Args::instance().getStickyValue("-sWmbc-maxEpiErr", 100);
    vp.maxEpiCov = Debug::Args::instance().getStickyValue("-sWmbc-maxEpiCov", 10);
    vp.minEpiSize = Debug::Args::instance().getStickyValue("-sWmbc-minEpiSize", 30);
    static int convNightSnowSamp = Debug::Args::instance().getStickyValue("-sWmbc-convNightSnowSamp", 200); // assign a vlue in case it's snow+night

    // parameters controlling convergence conditions
    cp.convSampleNumThreshold = properties->wmbcConvSample();
    // cp.convSampleNumThresholdNightSnow = properties->wmbcConvNightSnowSample();
    cp.convSampleNumThresholdNightSnow = convNightSnowSamp;
    cp.convStableNumThreshold = properties->wmbcConvStable();
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      // TODO: check that std and maxDiff > binSize
      cp.convMaxVar[i] = WMBC_SERIAL_PARAM(properties->wmbcConvMaxStd(), i);
      cp.convMaxVar[i] *= cp.convMaxVar[i]*(i<e_CAM_HEIGHT ? DEG2RAD_SQR : 1.f);
      cp.convStableMaxDiff[i] = WMBC_SERIAL_PARAM(properties->wmbcConvStableMaxDiff(), i);
      cp.convStableMaxDiff[i] *= (i<e_CAM_HEIGHT ? DEG2RAD : 1.f);
      cp.convMovingSampleNumThreshold[i] = cp.convSampleNumThreshold;
    }
    cp.useConvMoving = properties->afixUseMovingSampleTh();
    for (int i = 0; i < 4; ++i) {
      cp.convMovingMarksFoe[i] = WMBC_SERIAL_PARAM(properties->afixConvFoe(), i)*DEG2RAD;
      cp.convMovingMarksRoll[i] = WMBC_SERIAL_PARAM(properties->afixConvRoll(), i)*DEG2RAD;
      cp.convMovingSamples[i] = WMBC_SERIAL_PARAM(properties->afixConvSample(), i);
    }
    cp.convSampleNumThresholdFS = properties->afixConvSampleFS();
    cp.convStableNumThresholdFS = properties->afixConvStableFS();
    cp.minQuality = properties->wmbcMinQuality();
    cp.minEmEpiQuality = Debug::Args::instance().getStickyValue("-sWmbc-minEpiQuality", 0.45f);
    cp.minCrownQuality = Debug::Args::instance().getStickyValue("-sWmbc-minCrownQuality", 0.5f);

    // parameters controlling result finalizing and decision making
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      mp.foeDeltaThresholdFS[i] = WMBC_SERIAL_PARAM(properties->afixfoeDeltaFS(), i);
      mp.foeDeltaThresholdFS[i] *= (i < e_CAM_HEIGHT) ? DEG2RAD : 1.f;
    }
    mp.afixClippedJump[e_YAW] = properties->afixFoeClippedJump()*DEG2RAD;
    mp.afixClippedJump[e_HORIZON] = mp.afixClippedJump[e_YAW];
    mp.afixClippedJump[e_ROLL] = properties->afixRollClippedJump()*DEG2RAD;

    switch (properties->wmbcMode()) {
      case WmbcProperties::e_Run_Autofix:
        mp.runMode = e_Autofix;
        break;
      case WmbcProperties::e_Run_SPC:
        mp.runMode = e_SPC;
        break;
      case WmbcProperties::e_Run_SLC:
        mp.runMode = e_SLC;
        break;
      default:
        mp.runMode = e_Autofix;
    }
    mp.maxAttempts = properties->spcMaxAttemps();

    mp.useSpecialLimits = properties->wmbcUseSpecialLimits();
    // if using special limits (wmbc internal) then yaw/pitch are in angles
    // otherwise in pixel level -2
    if (mp.useSpecialLimits) {
      mp.foeRange[e_YAW][0] = properties->wmbcMinYawAngle();
      mp.foeRange[e_YAW][1] = properties->wmbcMaxYawAngle();
      mp.foeRange[e_HORIZON][0] = properties->wmbcMinPitchAngle();
      mp.foeRange[e_HORIZON][1] = properties->wmbcMaxPitchAngle();
      mp.foeRange[e_ROLL][1] = properties->wmbcMaxRollAngle();
      mp.foeRange[e_ROLL][0] = -mp.foeRange[e_ROLL][1];
      mp.foeRange[e_CAM_HEIGHT][0] = properties->wmbcMinCamHeight();
      mp.foeRange[e_CAM_HEIGHT][1] = properties->wmbcMaxCamHeight();
    } else {
      // In Autofix mode these values are slightly modified in initAutofixState
      mp.foeRange[e_YAW][0]     = 4*CameraInfo::minYaw(CameraInfo::e_FORWARD);
      mp.foeRange[e_YAW][1]     = 4*CameraInfo::maxYaw(CameraInfo::e_FORWARD);
      mp.foeRange[e_HORIZON][0] = 4*CameraInfo::minHorizon(CameraInfo::e_FORWARD);
      mp.foeRange[e_HORIZON][1] = 4*CameraInfo::maxHorizon(CameraInfo::e_FORWARD);
      mp.foeRange[e_ROLL][1]    = CameraInfo::maxRoll(CameraInfo::e_FORWARD);
      mp.foeRange[e_ROLL][0]    = -mp.foeRange[e_ROLL][1];
    }

    mp.maxTime = properties->wmbcTimeoutTime();
    mp.maxDist = properties->wmbcTimeoutDist();
    mp.maxValidFrames = properties->wmbcTimeoutValidFrames();
    mp.timeoutTimeMinSpeed = properties->wmbcTimeoutTime_minSpeed();

    mp.camHeightRangeSevere = properties->camHeightRangeSevere();
    
    // duplicate first bit (foe to yaw and horizon)
    int calcSwitchMask = properties->wmbcCalcSwitch();
    bool isCalcFoe = ((calcSwitchMask & 1) != 0);
    calcSwitchMask = calcSwitchMask<<1;
    calcSwitchMask += isCalcFoe ? 1 : 0;
    mp.calcSwitchMask = calcSwitchMask;

    mp.sensor8MP = properties->wmbc8MP();

    // init confidecne
    _data.confParams[0] = ConfidenceParams(CONF_PARAMS0);
    _data.confParams[1] = ConfidenceParams(CONF_PARAMS1);
    _data.confParams[2] = ConfidenceParams(CONF_PARAMS2);

    // rm crown
    _data.rm.crownZ = Debug::Args::instance().getStickyValue("-sWmbc-crownDist",  5.f);


    _debugPrintMask = properties->wmbcDebugPrint();
  }

  void FOEFinder::run() {
    updateReset();
    setInputData();
    calcUnderlyingPlaneData();
    validateFrameVehicle();
    validateFrameAlgo(); //TODO - apply all required changes as a result of using plane sync
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      _variable[i].run(&_data);
    }
    setResults();
    setFlags(); // total valid, conv, quality; TODO: make subfunc of setResults
    updateConvergence();
    updateStatus();
    updateFoeResetFlag();
    setDebugShowData();
  }

  void FOEFinder_SPC::run() {
    if (_data.algo.total.isSessionEnded()) {
      return;
    }
    FOEFinder::run();
  }

  void FOEFinder::setInputData() {
    setCameraData();
    setWmEgomotionData();
    setWmRoadModelData();
    setWmEgomotionTracking(); // set conf for em/rm
    setVehicleData();
    WmbcOutputIF::instance().toItrkInputData(&_data);
  }

  void FOEFinder::setInputDataEndFrame() {
    setCameraData();
  }

  void FOEFinder::calcUnderlyingPlaneData() {
    bool validPlane=_planeSync.run(&_data);
    if (validPlane) {
      PlaneData &d = _data.plane;
      const Plane *p = _planeSync.underlyingPlane();
      d.roll       = p->roll();
      d.camh       = -p->d;
      d.valid      = true;
      d.location   = p->location;
      d.startFrame = p->startFrame;
      d.endFrame   = p->endFrame;
      d.confData   = p->confData;
    } else {
      _data.plane.valid=false;
    }
    WmbcOutputIF::instance().toItrkPlane(&_data);
  }


  void FOEFinder::setCameraDataInit() {
    // if (_originsInit) { // TODO: check if using init dependencies right maybe can be moved under ctor
    //   return;
    // }

    _data.camera.focalLm2 = CameraInfo::focalLength(CameraInfo::e_FORWARD) * 4;
    _data.camera.invFocalLm2 = 1.0f / _data.camera.focalLm2;
    //static bool origin8MP = Debug::Args::instance().existsParameter("-sWmbc-origin8MP");

    for(int i = 0; i < CameraInfo::e_NUM_OF_CAM_INSTANCES; ++i) {
      OriginData &o = _data.camera.origin[i];
      if (o.init) {
        continue;
      }

      CameraInfo::CameraInstance inst = CameraInfo::CameraInstance(i);
      if(!WMBC_CAM_OK(i)) {
        o.valid = false;
        o.init = true;
        continue;
      }

      bool validNominal = false;
      if (_data.metaParams.sensor8MP) {
        o.distLimitLm2X = PrepSys_API::getLM2ROI(inst).effectiveWidth();
        o.distLimitLm2Y = PrepSys_API::getLM2ROI(inst).allocatedHeight();
        validNominal = o.distLimitLm2X*o.distLimitLm2Y > 0;
      } else {
        const Prep::SafeImg12* imgexp = PrepSys_API::getExposure(PrepSys::exp_mask::T0, inst);
        o.distLimitLm2X = imgexp ? ((*imgexp)->effectiveWidth()) : 1280;
        o.distLimitLm2Y = imgexp ? ((*imgexp)->allocatedHeight()) : 960;
        validNominal = (imgexp != nullptr);
      }
      o.distNominalLm2.X() =  o.distLimitLm2X/2;
      o.distNominalLm2.Y() =  o.distLimitLm2Y/2;

      //CameraInfo::imageToBuffer(inst, o.distLm2.X(), o.distLm2.Y());
      float x = 0.f;
      float y = 0.f;
      CameraInfo::imageToBuffer(inst, x, y);
      o.distLm2.X() = x;
      o.distLm2.Y() = y;
      bool validEtc = (o.distLm2.X() > 0 && o.distLm2.Y() > 0);

      o.distDeltaLm2.X() = o.distLm2.X() - o.distNominalLm2.X();
      o.distDeltaLm2.Y() = o.distLm2.Y()- o.distNominalLm2.Y();

      //CameraToCameraAPI::getPP(inst, -2, o.rectLm2.X(), o.rectLm2.Y()); // this is not currently used
      CameraToCameraAPI::getPP(inst, -2, x, y);
      o.rectLm2.X() = x;
      o.rectLm2.Y() = y; // this is not currently used


      o.valid = validNominal && validEtc;
      o.init = true;

    }

    // _originsInit = true;

    // printDebugOrigins();
  }

  void FOEFinder::setCameraData() {
    // Night Mode
    const float MAX_DAY_EXPOSURE = 1000.f;
    const dstruct_t* meta = PrepSys_API::getMetadata(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
    float effectiveExposure = Metadata_API::getEffectiveExposure(meta);
    _data.camera.night = (effectiveExposure > MAX_DAY_EXPOSURE);

    // Snow
    Fix::MEimage::Sync < FailSafesIF >* failSafes = Brain2API::getModelIF()->failSafes;
    if (failSafes != nullptr && failSafes->available()) {
      const FailSafesIF &failSafesIF = failSafes->getObj();
      _data.camera.snow = failSafesIF.getFsMode(MEfailsafe::RAIN_BIT) > 70;
    WMBC_PRINT(WmbcDbgStg::e_INPUT, 1, "[setCameraData] fsMode=%d",
               failSafesIF.getFsMode(MEfailsafe::RAIN_BIT));
    }

    _data.camera.nonSnowNightCounter = (_data.camera.snow && _data.camera.night) ? 0 : _data.camera.nonSnowNightCounter+1;
    _data.camera.nonSnowNightCounter = std::min(_data.camera.nonSnowNightCounter, _data.convParams.convSampleNumThreshold+1);
    _data.camera.snowNightMode = _data.camera.nonSnowNightCounter<_data.convParams.convSampleNumThreshold;

    WMBC_PRINT(WmbcDbgStg::e_INPUT, 1, "[setCameraData] effExp=%.0f, night: %d", effectiveExposure, _data.camera.night);
  }

  void FOEFinder::setWmEgomotionData() {
    // TODO: add these checks:
    // wmem_mode is as expected
    // vd3d is on
    // lens.conf exists
    static bool useClassicSFM = Debug::Args::instance().existsParameter("-ssfm-classic");
    EmData &d = _data.em;

    if (useClassicSFM) {
      d.valid = SFM2::isValidEgoMotion();
    } else {
    auto * emApi=WmbcServicesUser::instance().get<WorldModel::EgoMotion::EgoMotionService_API>();
    assert(emApi!=nullptr);
      d.valid =emApi->isOn();
    }

    if (!d.valid) {
      WMBC_PRINT(WmbcDbgStg::e_INPUT, 1, "[setWmEgomotionData] EM: off %s", (useClassicSFM ? " (classic SFM)" : ""));
      d.status = EM_OFF;
      return;
    }

    Float::MEmath::Mat<4,4,double> egoCam;
    const WorldModel::EgoMotion::EmData *emData = nullptr;
    if (useClassicSFM) {
      const Float::MEmath::Mat<4, 4, double> *p_egoCam = SFM2::getEgoCamT0T0();
      d.valid = (p_egoCam != nullptr);
      egoCam = d.valid ? *p_egoCam : Float::MEmath::identity<4, double>();
      d.conf = d.valid ? 0 : 1;
    } else {
      int glfi;
      WorldModel::EgoMotion::Exposure expFrom;
      auto * emApi=WmbcServicesUser::instance().get<WorldModel::EgoMotion::EgoMotionService_API>();
      assert(emApi!=nullptr);
      d.valid = emApi->getMeasurementEgoMotion(CameraInfo::e_FORWARD, WorldModel::EgoMotion::Exposure::T0, glfi, expFrom, egoCam);
      emData = WorldModel::EgoMotion::getEmVisionMeasurement(WorldModel::EgoMotion::Exposure::T0, WorldModel::RealCamInstance::RCI_FORWARD);
    }

    if (!d.valid) {
      WMBC_PRINT(WmbcDbgStg::e_INPUT,1, "[setWmEgomotionData] EM: invalid %s", (useClassicSFM ? " (classic SFM)" : ""));
      d.status = EM_INVALID;
      return;
    }


    Float::MEmath::Mat<3,3,double> R;
    Float::MEmath::Vec<3,double> t;
    WorldModel::decomposeTransformationMatrixIntoRt(egoCam, R, t);
    d.R=matd2f(R);
    d.t=vecd2f(t);
    
    double dyaw, dpitch, droll;
    double tx, ty, tz;
    WorldModel::decomposeTransformationMatrixIntoEulerAngles(egoCam, dyaw, dpitch, droll, tx, ty, tz);

    // alternative test for straight drive: t dot (-inv_t+
    Float::MEmath::Mat<4,4,double> invEgoCam;
    double itx, ity, itz, dummy;
    inv4(egoCam, invEgoCam);
    WorldModel::decomposeTransformationMatrixIntoEulerAngles(invEgoCam, dummy, dummy, dummy, itx, ity, itz);
    float tNorm = sqrt((tx*tx + ty*ty + tz*tz)*(itx*itx + ity*ity + itz*itz));
    d.straightScore = -(tx*itx + ty*ity + tz*itz)/tNorm;
    WMBC_PRINT(WmbcDbgStg::e_INPUT,1, "[setWmEgomotionData] strightScore: %.2f, tx: %.2f vs %.2f, ty: %.2f vs %.2f, tz: %.2f vs %.2f, "
               " foex: %.2f vs %.2f, foey: %.2f vs %.2f",
               d.straightScore, tx, itx, ty, ity, tz, itz, 
               (tx/tz)*RAD2DEG, (itx/itz)*RAD2DEG,
               (ty/tz)*RAD2DEG, (ity/itz)*RAD2DEG);

    // use log(EM) to get tangent to drive
    static bool useLogEgo = Debug::Args::instance().existsParameter("-sWmbcUseLogEgo");
    if (useLogEgo) {
      double dummy;
      Float::MEmath::Mat<4,4,double> logEgoCam;
      log4(egoCam, logEgoCam);
      WorldModel::decomposeTransformationMatrixIntoEulerAngles(logEgoCam, dummy, dummy, dummy, tx, ty, tz);
    }

    d.t[0] = tx;
    d.t[1] = ty;
    d.t[2] = tz;
    d.ypr[0] = dyaw;
    d.ypr[1] = dpitch;
    d.ypr[2] = droll;

    if (emData) {
      const MEtypes::Vector<double, 3> tnew = emData->translation();
      d.valid = (-tnew[2] > EPS);
      d.yaw = d.valid ? tnew[0]/tnew[2] : 0.f;
      d.pitch = d.valid ? tnew[1]/tnew[2] : 0.f;
      d.conf = emData->confidence();
      d.confT = emData->confT();
      d.confR = emData->confR();
    } else {
      d.valid = (-tz > EPS);
      d.yaw = d.valid ? tx/tz : 0.f;
      d.pitch = d.valid ? ty/tz : 0.f;
    }

    d.status = d.valid ? EM_OK : EM_STANDING;
    d.validFrames++;

    WMBC_PRINT(WmbcDbgStg::e_INPUT,2, "[setWmEgomotionData] EM: %s %s", (d.status == EM_OK ? "OK" : "STANDING"),
               (useClassicSFM ? " (classic SFM)" : ""));
  }

  void FOEFinder::setWmRoadModelData() {
    RmData &d = _data.rm;

    auto * rmApi=WmbcServicesUser::instance().get<WorldModel::RM::RoadModelService_API>();
    assert(rmApi!=nullptr);
    d.valid = rmApi->roadModelFound();
    if (!d.valid) {
      d.status = RM_MODEL_NOT_FOUND;
      WMBC_PRINT(WmbcDbgStg::e_INPUT,1, "[setWmRoadModelData] roadmodel not found", 0); // TODO: reslove the farse
      return;
    }

    static bool validSevere = Debug::Args::instance().existsParameter("-sWmbc-rmValidSevere");
    const WorldModel::VRM::SegmentModel& model = rmApi->getClosestPlane();
    // TODO: store only the model, don't copy all the fields like a moron
    WorldModel::EVec3 N = model.N;
    float dist = model.d;
    d.Zstart = model.Zstart;
    d.Zend = model.Zend;
    d.numOfInliers = model.numOfInliers;
    d.planeIdx = model.segmentIndex;
    d.message = model.type;
    d.valid = validSevere ? (d.message == WorldModel::VRM::VALID_PLANE) : (dist < 0);
    d.AtA = model.AtA;
    d.nAtA = model.normalizedAtA;

    //d.AtA.data()[0][2] = d.AtA.data()[2][0]; // workaround a bug in roadModelUtils.cpp
    *(d.AtA(0, 2)) = *(d.AtA(2, 0));
    *(d.nAtA(0, 2)) = *(d.nAtA(2, 0));

    WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_BROWN, "[setWmRoadModelData] validType: %s, d: %.2f, #inliers: %d, planeIdx: %d, Message: %s",
               (validSevere ? "Severe" : "Relaxed"), dist, d.numOfInliers, d.planeIdx,
               WMBC_CASES4(d.message, WorldModel::VRM::INVALID_PLANE, "INVALID_PLANE",
                           WorldModel::VRM::NOT_ENOUGH_POINTS,"NOT_ENOUGH_POINTS",
                           WorldModel::VRM::FAR_FROM_CALIB, "FAR_FROM_CALIB", "VALID_PLANE"));

    if (!d.valid) {
      d.status = RM_CLOSE_PLANE_INVALID;
      WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_RED, "[setWmRoadModelData] close road plane invalid", 0);
      return;
    }

    d.valid = (N[1] > EPS);
    if (!d.valid) {
      d.status = RM_PLANE_OFFSIDE;
      WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_RED, "[setWmRoadModelData] Ny = 0", 0);
      return;
    }

    for (int i = 0; i < 3; ++i) {
      d.N[i] = N[i];
    }
    d.d = -dist; // TODO: rename to camh
    d.roll = me_atanf(-N[0]/N[1]); // TODO: check if atan is necessary
    d.pitch = me_atanf(-N[2]/N[1]); // TODO: check if atan is necessary
    d.status = RM_OK;
    d.valid = true;

    d.pitchDiff = me_fabs(d.pitch - _data.em.pitch);

    WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_GREEN, "[setWmRoadModelData] N = (%.2f, %.2f, %.2f), d = %.2f, "
               "roll = %.4f [rad] (%.2f [deg]), pitch = %.4f [rad] (%.2f [deg]), #inliers=%d",
               N[0], N[1], N[2], dist,
               d.roll, d.roll*RAD2DEG,
               d.pitch, d.pitch*RAD2DEG, d.numOfInliers);

    // crown detection
    // sample the world at different Xs (circa lane width) at a fixed Z
    // at each point get the angle of the tangent plane and check the 
    // difference between the angle at the right sample point to the left sample point.
    static const float X_RAD = Debug::Args::instance().getStickyValue("-sWmbc-crownXRad",  1.9f);
    static const float crownZfac = Debug::Args::instance().getStickyValue("-sWmbc-crownZfac", -1.f);
    if (crownZfac > 0) {
        d.crownZ = d.Zstart + crownZfac*(d.Zend - d.Zstart);
    }
    for (int i = 0; i < CROWN_SAMP_NUM; ++i) {
      Float::MEmath::Vec<3, float> surf_N;
      float surf_d;
      Float::MEmath::Mat<4, 4, double> surf_Cov;
      d.crownX[i] = -X_RAD + (2*i*X_RAD)/(CROWN_SAMP_NUM-1);
      auto * rmApi=WmbcServicesUser::instance().get<WorldModel::RM::RoadModelService_API>();
      assert(rmApi!=nullptr);
      rmApi->XZ2tangentPlane(d.crownX[i], d.crownZ, surf_N, surf_d, surf_Cov, 0);
      d.crownAng[i] = -surf_N[0]/surf_N[1]; // me_atanf(-surf_N[0]/surf_N[1]);
      d.crownAngStd[i] = 0.f; // TODO: implement
      WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_GREEN, "[setWmRoadModelData] surf, Z = %.2f (%.2f, %.2f), X = %f, angle = %.2f [deg], d = %f",
                 d.crownZ, d.Zstart, d.Zend, d.crownX[i],
                 d.crownAng[i]*RAD2DEG, surf_d);
    }
    d.crownAngDiff = me_fabs(d.crownAng[CROWN_SAMP_NUM-1] - d.crownAng[0]);
    WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_GREEN, "[setWmRoadModelData] crADiff = %.2f [deg]", d.crownAngDiff*RAD2DEG);

    // simpler crown calc
    Float::MEmath::Vec<3, float> surf_N1, surf_N2;
    float surf_d1, surf_d2;
    Float::MEmath::Mat<4, 4, double> surf_Cov1, surf_Cov2;
    rmApi->XZ2tangentPlane(-X_RAD, d.crownZ, surf_N1, surf_d1, surf_Cov1, 0);
    rmApi->XZ2tangentPlane(X_RAD,  d.crownZ, surf_N2, surf_d2, surf_Cov2, 0);
    float NN = surf_N1 * surf_N2;
    d.crownAngDiff1 = me_fabs(NN) < 1.0-EPS ? me_acosf(NN) : 0.f;
    d.crownDistDiff1 = surf_d2-surf_d1;
    WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_GREEN, "[setWmRoadModelData] crADiff1 = %.2f [deg], dDiff=%.4f, N1*N2=%.6f, "
               "cov1_00=%.6f, cov1_11=%.6f, cov1_22=%.6f, cov1_33=%.6f, "
               "cov2_00=%.6f, cov2_11=%.6f, cov2_22=%.6f, cov2_33=%.6f",
               d.crownAngDiff1*RAD2DEG, d.crownDistDiff1, NN,
               surf_Cov1(0,0), surf_Cov1(1,1), surf_Cov1(2,2), surf_Cov1(3,3), 
               surf_Cov1(0,0), surf_Cov1(1,1), surf_Cov1(2,2), surf_Cov1(3,3));
  }

  void FOEFinder::setVehicleData() {
    VehicleData &d = _data.vehicle;

    bool isYawRateAvailable = PrepSys_API::getYawRateAvailable(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
    float yawRateInPixels;
    if (isYawRateAvailable) {
      yawRateInPixels = PrepSys_API::getVehicleYawInPixels(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
    } else {
      const Prep::PitchYawInfoHist& pitchYawHist = *PrepSys_API::getImgPitchYaw(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
      yawRateInPixels = EgoMotionAPI::isYawInRadiansValid() ? pitchYawHist[0].pitchYaw.motion()[0] : 0.0; // TODO: is fail default should be zero yaani frame valid?
    }
    // float dyaw = -yawRateInPixels / (_worldModelData.focalLm2 * 0.25); // TODO: check level of getVehicleYawInPixels
    float dyaw = -yawRateInPixels * _data.camera.invFocalLm2 * 4; // TODO: check level of getVehicleYawInPixels

    float speed = VehicleInfo::speedAvailable() ? VehicleInfo::egoSpeed() : 0.0f;
    float dt = dTime(*PrepSys_API::getTimeStamp(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD));
    float ds = speed * dt;
    float k = (ds > EPS) ? me_fabs(0.5*dyaw/ds) : 0.f;

    d.radius = (k > EPS) ? (0.5/k) : 2*_data.validParams.minRadius; // TODO: shove to const
    d.yawRate = (dt > EPS) ? dyaw/dt : 0.f;
    d.dt = dt;
    d.ds = ds;
    d.trajLength += ds; // TODO: change to distance
    d.trajLengthAbs += me_abs(ds);
    d.trajTime += dt; // TODO: change to time
    d.drivingTime += (speed > _data.metaParams.timeoutTimeMinSpeed) ? dt : 0.f;
    d.reverseGear = VehicleInfo::reverseGearAvailable() ? (VehicleInfo::gearState() == VehicleInfo::e_GEAR_REVERSE) : false;
    d.vehicleKneeling = VehicleInfo::vehicleKneelingMode();
    // TODO: maybe make a yawRateAvaliable and speedAvaliable members for vehicleData and decide how it affect frame validity
    if (false) { // (VehicleInfo::longitudinalAccelerationAvailable()) {
      d.accel = VehicleInfo::longitudinalAcceleration();
    } else {
      float dspeed = speed - d.speed;
      d.accel = (dt > EPS && speed > EPS && d.speed > EPS) ? dspeed / dt : INVALID_VAL;
    }
    d.speed = speed;
    d.speedHighPrecisionValid = VehicleInfo::speedHighPrecisionValid();
    d.speedHighPrecision = VehicleInfo::speedHighPrecision();
    d.speedPrecisionError = me_fabs(speed - d.speedHighPrecision);
    
    d.dynamicCameraHeightValid = VehicleInfo::dynamicCameraHeightValid();
    d.dynamicCameraHeight = VehicleInfo::dynamicCameraHeight();
    
    d.dynamicCameraHeightDeltaValid = VehicleInfo::dynamicCameraHeightDeltaValid();
    d.dynamicCameraHeightDelta = VehicleInfo::dynamicCameraHeightDelta();
    d.dynamicCameraHeightChangeActiveValid = VehicleInfo::dynamicCameraHeightChangeActiveValid();
    d.dynamicCameraHeightChangeActive = VehicleInfo::dynamicCameraHeightChangeActive();
    d.dynamicCameraHeightDeltaAvailable = VehicleInfo::dynamicCameraHeightDeltaAvailable();
    
    WMBC_PRINT(WmbcDbgStg::e_INPUT,2, "[setVehicleData] speed: %.1fm/s, dt: %.1fs, radius: %.1fm, yawRate: %.2fdeg/s, accel: %.2fm/s^2, "
               "vhAccelAval: %d, vhAccel: %.2fm/s^2, speedHP: %.2f (%svalid), speedPrecisionError: %.2fm/s",
               speed, dt, d.radius, d.yawRate*RAD2DEG, d.accel, VehicleInfo::longitudinalAccelerationAvailable(),
               VehicleInfo::longitudinalAcceleration(), d.speedHighPrecision, (d.speedHighPrecisionValid ? "" : "in"),
               d.speedPrecisionError);

    WMBC_PRINT(WmbcDbgStg::e_INPUT,2, "[setVehicleData] dynamicCameraHeightChangeActive=%d, dynamicCameraHeightDelta=%.5f, "
               "dynamicCameraHeightChangeActiveValid= %d, dynamicCameraHeightDeltaValid=%d, dynamicCameraHeightDeltaAvailable=%d\n",
               d.dynamicCameraHeightChangeActive, d.dynamicCameraHeightDelta,
               d.dynamicCameraHeightChangeActiveValid, d.dynamicCameraHeightDeltaValid,
               d.dynamicCameraHeightDeltaAvailable);
  }

  void FOEFinder::validateFrameVehicle() {
    VehicleData &vd = _data.vehicle;

#ifndef EYEQ_HW_IMPL
    static bool useAdHocFrames = Debug::Args::instance().existsParameter("-sWmbc-adHocFrames");
    if (useAdHocFrames) {
      // TODO: adHoc still needs to check availability of EM and RM -
      // need to separate algo conditions from vehicle conditions
      if (validateFrameAdHoc()) {
        _data.algo.pauseReason = 0; // TODO: rename pauseReason -> invalidFrameReason
        vd.validFrame = true;
        vd.validFrameNum += 1;
        vd.validDistance += vd.ds;
        vd.validTime += vd.dt;
      } else {
        _data.algo.pauseReason = VEHICLE_INVALID;
        vd.validFrame = false;
      }
      WmbcOutputIF::instance().toItrkFrameValidation(&_data);
      return;
    }
#endif

    ValidParams &vp = _data.validParams;
    int &m = _data.algo.pauseReason;
    m = 0;

    // enforce taking the first measurement (for wmFoeReset acceleration)
    _data.algo.enforcingFirstValid = false;
    static bool enforceFirstValid = !Debug::Args::instance().existsParameter("-sWmbc-dontEnforceFirstValid");
    bool noDataYet = (_variable[e_YAW].validFrameNum() * _variable[e_HORIZON].validFrameNum() == 0);
    bool noSuspension = (!vp.useDynamicSuspension || !vd.dynamicCameraHeightChangeActive);

    if (enforceFirstValid && noDataYet && _data.em.valid && noSuspension) {
      _data.algo.enforcingFirstValid = true;
      // vd.validFrame = true;
      // vd.validFrameNum = 1;
      // vd.validDistance = vd.ds;
      // vd.validTime = vd.dt;

      // WmbcOutputIF::instance().toItrkFrameValidation(&_data);
      WMBC_PRINT(WmbcDbgStg::e_VALIDATE_FRAME, WmbcDbgClr::e_BROWN, "[validateFrame] enforcing 1st emValid frame", 0);
      // return;
    }

    // validate vehicle frame
    if (vd.speed < vp.minSpeed) {
      m |= e_SPEED_TOO_LOW;
      }
    if (vd.speed > vp.maxSpeed) {
      m |= e_SPEED_TOO_HIGH;
    }
    if (vd.radius < vp.minRadius) {
      m |= e_RADIUS_TOO_SMALL;
    }
    if (me_fabs(vd.accel) > vp.maxAccel) {
      m |= e_ACCELERATION_TOO_HIGH;
    }
    if (me_fabs(vd.yawRate) > vp.maxYawRate) {
      m |= e_YAWRATE_TOO_HIGH;
    }
    // if (vd.reverseGear || vd.vehicleKneeling) {
    //   m |= e_REVERSE_IS_ON;
    // }
    if (vp.useDynamicSuspension && vd.dynamicCameraHeightChangeActive) {
      m |= e_DYNAMIC_SUSPENSION_ACTIVE;
    }

    if (vp.hystEnabled) {
      if ((m & e_RADIUS_TOO_SMALL) == 0) {
        vp.minRadius = std::max(vp.hystMinRadiusRange[0], vp.minRadius - vp.hystMinRadiusRange[2]);
      } else {
        vp.minRadius = std::min(vp.hystMinRadiusRange[1], vp.minRadius + vp.hystMinRadiusRange[2]);
      }
    }

    vd.validFrame = ((m & VEHICLE_INVALID) == 0);
    if (vd.validFrame) {
      vd.validFrameNum += 1;
      vd.validDistance += vd.ds;
      vd.validTime += vd.dt;
    }
  }

  void FOEFinder::validateFrameAlgo() {
    // if (_data.algo.enforcingFirstValid) {
    //   return;
    // }

    ValidParams &vp = _data.validParams;
    int &m = _data.algo.pauseReason;

    if (!_data.em.valid) {
      m |= e_EM_INVALID;
    }
    if (!_data.plane.valid) {
      m |= e_RM_INVALID;
    }

    // if useAdHocFrames and validateFrameAdHoc() -> return

    if (_data.em.valid && !isStraightDrive()) {
      m |= e_EM_STRAIGHT;
    }
    bool isPitchDiff = (vp.maxPitchDiff > 0) && (_data.rm.pitchDiff >= vp.maxPitchDiff); // TODO: disable condition by default
    if (isPitchDiff && _data.rm.valid) {
      m |= e_EM_RM_PITCHDIFF;
    }
    bool isCrown = (vp.maxCrownAngDiff > 0) && (_data.rm.crownAngDiff >= vp.maxCrownAngDiff);
    if (isCrown) {
      m |= e_ROAD_CROWN;
    }
    bool useSpeedHighPrecision = (vp.useSpeedHighPrecision && _data.metaParams.runMode != e_SPC);
    if (useSpeedHighPrecision && !_data.vehicle.speedHighPrecisionValid) {
      m |= e_SPEED_HP_INVALID;
    }
    // if (_data.em.epiErr > _data.validParams.maxEpiErr ||
    //     _data.em.rmsPtNum[0] < _data.validParams.minEpiSize) {
    //   m |= e_REVERSE_IS_ON; // TODO: change name
    // }
    static const float maxEpiDiff = Debug::Args::instance().getStickyValue("-sWmbc-maxEpiDiff", -1.f)*DEG2RAD;
    bool largeEpiCov = (_data.em.epiCov[0] > vp.maxEpiCov || _data.em.epiCov[1] > vp.maxEpiCov);
    bool largeEpiDiffX = (me_fabs(_data.em.yaw - _data.em.epiFoeAngle[0]) > maxEpiDiff);
    bool largeEpiDiffY = (me_fabs(_data.em.pitch - _data.em.epiFoeAngle[1]) > maxEpiDiff);
    bool largeEpiDiff = (maxEpiDiff > 0 && (largeEpiDiffX || largeEpiDiffY));
    if (largeEpiCov || largeEpiDiff) {
      m |= e_REVERSE_IS_ON; // TODO: change name
    }


    if (_data.metaParams.runMode == e_SLC) {
      return;
    }

    if ((m & EM_STR_INVALID) == 0) {
      _data.em.validStraightFrameNum += 1;
    }
    if ((m & EM_QUAL_INVALID) == 0) {
      _data.em.epiQualityFrameNum += 1;
    }

    if ((m & ROLL_INVALID) == 0) {
      if (_data.rm.crownAngDiff < _data.rm.crownAngDiffMin) {
        _data.rm.crownAngDiffMin = _data.rm.crownAngDiff;
      }
      if (_data.rm.crownAngDiff > _data.rm.crownAngDiffMax) {
        _data.rm.crownAngDiffMax = _data.rm.crownAngDiff;
      }
      int n      = _variable[e_ROLL].validFrameNum(); // this is before the counter is incremented so actually its n-1
      float x    = _data.rm.crownAngDiff;
      float mfm1 = _data.rm.crownAngDiffMean;
      float vfm1 = _data.rm.crownAngDiffVar;
      float mf   = (x + n*mfm1)/(n+1);
      float vf   = (x*x + n*vfm1 + n*mfm1*mfm1)/(n+1) - mf*mf;
      _data.rm.crownAngDiffMean = mf;
      _data.rm.crownAngDiffVar  = vf;
      _data.em.rmsPtNumMean = (_data.em.rmsPtNum[0] + n*_data.em.rmsPtNumMean)/(n+1);
    }

    WmbcOutputIF::instance().toItrkFrameValidation(&_data);
    WMBC_PRINT(WmbcDbgStg::e_VALIDATE_FRAME, 3, "[validateFrame] speed: %d, driving: %d, radius: %d, yawrate: %d, accel: %d, validVeh: %d, #validVeh: %d\n"
               "EM: %d, RM: %d, EM-str: %d, pDiff: %d, speedHP: %d",
               (m & (e_SPEED_TOO_LOW | e_SPEED_TOO_HIGH)) == 0,
               (m & e_REVERSE_IS_ON)                      == 0,
               (m & e_RADIUS_TOO_SMALL)                   == 0,
               (m & e_YAWRATE_TOO_HIGH)                   == 0,
               (m & e_ACCELERATION_TOO_HIGH)              == 0,
               _data.vehicle.validFrame,
               _data.vehicle.validFrameNum,
               (m & e_EM_INVALID)                         == 0,
               (m & e_RM_INVALID)                         == 0,
               (m & e_EM_STRAIGHT)                        == 0,
               (m & e_EM_RM_PITCHDIFF)                    == 0,
               (m & e_SPEED_HP_INVALID)                   == 0);
  }

#ifndef EYEQ_HW_IMPL
  bool FOEFinder::validateFrameAdHoc() {
    FrameRangeData frd;
    bool ok = frd.loadFile("");//adHocFramesFile);
    if (!ok) {
      WMBC_PRINT(WmbcDbgStg::e_VALIDATE_FRAME, WmbcDbgClr::e_BROWN, "[validateFrame] framesFile loading failed", 0);
      return false;
    }

    int frameNum = frd.getFrameNum();
    int currFrame = *PrepSys_API::getGrabIndex(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
    bool okFrame = false;
    for (int i = 0; i < frameNum; ++i) {
      okFrame = okFrame || ((currFrame >= frd.getStartFrame(i)) && (currFrame <= frd.getEndFrame(i)));
    }

    if (!okFrame) {
      WMBC_PRINT(WmbcDbgStg::e_VALIDATE_FRAME, WmbcDbgClr::e_BROWN, "[validateFrame] frame outside valid range", 0);
      return false;
    }

    return true;
  }
#endif

  bool FOEFinder::isStraightDrive() {
    const EmData &em = _data.em;
    ValidParams &vp = _data.validParams;

    bool isStraightDrive = _data.em.valid;
    for (int i = 0; i < 3; ++i) {
      bool yprOk = (me_abs(em.ypr[i]) < vp.rotThreshold[i]); // || (ypr[i] == INVALID_VAL);
      isStraightDrive = isStraightDrive && yprOk;
      if (vp.hystEnabled && em.validFrames > 1) {
        if (yprOk) {
          vp.rotThreshold[i] = std::min(vp.hystRotThMax[i], vp.rotThreshold[i] + vp.hystRotThInc);
        } else {
          vp.rotThreshold[i] = std::max(vp.hystRotThMin, vp.rotThreshold[i] - vp.hystRotThInc);
        }
      }
    }
    return isStraightDrive;
  }

  // TODO: change to CalibComponent and move to separate file
  CalibVariable::CalibVariable() : _id(e_YAW), _dataGlobal(nullptr), _sf(0.f), _result(0.f), _pauseReason(0),
                                                     _invalidFrameMask(0), _validFrame(false), 
                                                     _validFrameNum(0), _validDistance(0.f), _validTime(0.f),
                                                     _conv(false), _progress(0), _quality(0),
				                     _stableMedianCount(0), _nonConvReason(0), _convNum(0),
                                                     _convLastFrame(-1), _convMovingSampleNumThreshold(0),
                                                     _inRange(false), _emLastValidFrameNum(0),
                                                     _emLastEpiQualityFrameNum(0), _crownQuality(0.f), 
                                                     _status(e_CALIB_UNDEFINED),
                                                     _coreError(e_CALIB_GENERAL), _coreStatus(e_CORE_INIT)
    {
      _hist.parent(this);
      reset();
    }

  void CalibVariable::reset() {
    _hist.reset();
    _convStat.reset();
    _sf = 0.f;
    _result = 0.f;
    _pauseReason = 0;
    _validFrame = false;
    _validFrameNum = 0;
    _validDistance = 0.f;
    _validTime = 0.f;
    _conv = false;
    _progress = 0;
    _quality = 0;
    for (int i = 0; i < CONF_BIT_NUM; ++i) {
      _confidence[i] = 0.f;
    }
    _stableMedianCount = 0;
    _nonConvReason = 0;
    _convMovingSampleNumThreshold = 0;
    _inRange = false;
    _crownQuality = 0.f;
    _status = e_CALIB_UNDEFINED;
    _coreError = e_CALIB_GENERAL;
    _coreStatus = e_CORE_INIT;
  }

  void CalibVariable::run(const WmbcData *dataGlobal) {
    _dataGlobal = dataGlobal;
    if (_dataGlobal->metaParams.runMode != e_Autofix && _conv) {
      return;
    }
    validateFrame();
    if (_validFrame) {
      setSingleFrame(); // TODO: should i update sf even when invalid for ease of itrk processing?
      _hist.update(_sf);
      updateConvergence();
      updateConfidence();
    }

    // When using wmbc internal limits the OOR is determined here
    // otherwise it is overriden outside because the comparison is in pixels where there is
    // coupling between yaw and pitch
    _inRange = true;
    if (_dataGlobal->metaParams.useSpecialLimits) {
    _inRange = (_hist.median() >= _dataGlobal->metaParams.foeRange[_id][0] &&
                _hist.median() <= _dataGlobal->metaParams.foeRange[_id][1]);
    }

    updateStatus();
    updateCoreStatus();
    WmbcOutputIF::instance().toItrkCalibVariable(this, _dataGlobal);
    WmbcOutputIF::instance().toItrkConvergence(_id); // TODO: remove
    if (_conv) {
      _convLastFrame = globalFrameIndex;
    }

  }

  void CalibVariable::setSingleFrame() {
    switch (_id) {
      case e_YAW:
        _sf = _dataGlobal->em.yaw;
        break;
      case e_HORIZON:
        _sf = _dataGlobal->em.pitch;
        break;
      case e_ROLL:
        _sf = _dataGlobal->plane.roll;
        break;
      case e_CAM_HEIGHT:
        _sf = _dataGlobal->plane.camh;
        break;
      default:
        _sf = 0.f;
        break;
    }
  }

  void CalibVariable::validateFrame() {
    if ((_id == e_YAW || _id == e_HORIZON) && _dataGlobal->algo.enforcingFirstValid) {
       _validFrame = (me_fabs(_dataGlobal->em.yaw) < MAX_IN_FOE_ANG &&
                      me_fabs(_dataGlobal->em.pitch) < MAX_IN_FOE_ANG);
    } else {
      int pauseReason = _dataGlobal->algo.pauseReason;
      _validFrame = ((pauseReason & _invalidFrameMask) == 0);
    }
    if (_validFrame) {
      _validFrameNum += 1;
      _validDistance += _dataGlobal->vehicle.ds;
      _validTime += _dataGlobal->vehicle.dt;
    }
    WMBC_PRINT(WmbcDbgStg::e_VALIDATE_FRAME, WmbcDbgClr::e_WHITE,
               "[validateFrame@CV] id=%s, validFrame=%d, #validFrame=%d, validDist=%.1f, validTime=%.1f",
                WMBC_DOF_ID_STR(_id), _validFrame, _validFrameNum, _validDistance, _validTime);
  }

  void CalibVariable::updateConvergence() {
    if (_conv) {
      _progress = 100;
      return;
    }

    const int NON_CONV_MASK = (e_HIGH_VAR | // TODO: move to wmbcTypes.h
                               e_SMALL_SAMPLE |
                               e_NON_STABLE |
                               e_LOW_QUALITY // todo: change to e_LOW_CONF, should confidence affect convergence?
                              );

    //ConvParams &p = _dataGlobal->convParams;
    const ConvParams &p = _dataGlobal->convParams;
    HistogramOneDim &h = _hist;

    static bool useRegularVar = Debug::Args::instance().existsParameter("-sWmbcUseRegularVar");
    static bool useIQR = Debug::Args::instance().existsParameter("-sWmbc-useIQR");

    float var = h.varDecay();
    if (useIQR) {
      float iqr = h.iqr();
      var = iqr*iqr;
    }
    if (useRegularVar) {
      var = h.var();
    }

    // ad-hoc quality function, TODO: use variance or something real
    //float x = _dataGlobal->vehicle.distance * QUALITY_FACTOR[_id];
    float x = _dataGlobal->vehicle.trajLength * QUALITY_FACTOR[_id];
    float quality_f = std::max(0, std::min(1, QUALITY_SHIFT[_id] + (x/(1 + x))));
    _quality = me_roundf(100*quality_f);

    // int emValidFrameNum = _dataGlobal->em.validStraightFrameNum - _emLastValidFrameNum;
    // int emEpiQualityFrameNum = _dataGlobal->em.epiQualityFrameNum - _emLastEpiQualityFrameNum;
    // float epiQuality = 0.f;
    // if (emValidFrameNum > 0) {
    //   epiQuality = 1.f*emEpiQualityFrameNum/emValidFrameNum;
    // }

    // quality induced by crown and iqr
    //float crownQuality = 1.f;
    float minCrownQuality = p.minCrownQuality;
    static const float crownQw0 = Debug::Args::instance().getStickyValue("-sWmbc-crownQW0", 0.2f);
    static const float crownQw1 = Debug::Args::instance().getStickyValue("-sWmbc-crownQW1", 0.2f);
    if (_id == e_ROLL) {

      // float q0 = sigmoidActivation(_dataGlobal->rm.crownAngDiffMean, 0.03f, 0.f, 5.f);
      // float q1 = sigmoidActivation(_dataGlobal->rm.crownAngDiffMax, 0.035f, 0.f, 5.f);

      float q0 = sigmoidActivation(_dataGlobal->rm.crownAngDiffMean*RAD2DEG, 1.7f, 0.f, 5.f);
      float q1 = linearActivation(_dataGlobal->em.rmsPtNumMean, 30.f, 75.f);
      float q2 = linearActivation(h.iqr()*RAD2DEG, 0.9f, 0.2f);
      float w[2] = {crownQw0, crownQw1};
      //crownQuality = w[0]*q0 + w[1]*q1 + (1-w[0]-w[1])*q2;
      _crownQuality = w[0]*q0 + w[1]*q1 + (1-w[0]-w[1])*q2;

      static const float NIGHT_FAC = Debug::Args::instance().getStickyValue("-sWmbc-crownQNightFac", 0.5f);
      if (_dataGlobal->camera.night) {
        minCrownQuality *= NIGHT_FAC;
      }
      WMBC_PRINT(WmbcDbgStg::e_CONVERGENCE, WmbcDbgClr::e_BROWN,
                 "[updateConvergence@CV] id=%s, crADiffMean=%.4f, crADiffMax=%.4f, iqr=%.4f\n"
                 "[updateConvergence@CV] q = %.2f*%.4f + %.2f*%.4f + %.2f*%.4f = %.4f",
                 WMBC_DOF_ID_STR(_id), _dataGlobal->rm.crownAngDiffMean, _dataGlobal->rm.crownAngDiffMax,
                 h.iqr()*RAD2DEG, w[0], q0, w[1], q1, (1-w[0]-w[1]), q2, _crownQuality);
    } else {
      _crownQuality = 1.f;
    }

    setSampleThreshold();

    if (me_fabs(h.median() - h.prevMedian()) < p.convStableMaxDiff[_id]) {
      _stableMedianCount++;
    } else {
      _stableMedianCount = 0;
    }

    int &m = _nonConvReason;
    m = 0;
    if (var > p.convMaxVar[_id]) {
      m |= e_HIGH_VAR;
    }
    if (h.sampleNum() < _convMovingSampleNumThreshold) {
      m |= e_SMALL_SAMPLE;
    }
    if (p.convStableMaxDiff[_id] > 0 && (_stableMedianCount < p.convStableNumThreshold)) {
      m |= e_NON_STABLE;
    }
    // if (quality_f < p.minQuality) {
    //   m |= e_LOW_QUALITY;
    // }
    // if (_dataGlobal->em.epiQuality < p.minEmEpiQuality) {
    //   m |= e_LOW_QUALITY;
    // }
    // if (epiQuality < p.minEmEpiQuality) {
    //   m |= e_LOW_QUALITY;
    // }
    // if (_crownQuality < minCrownQuality) {
    //   m |= e_LOW_QUALITY;
    // }

    _conv = ((m & NON_CONV_MASK) == 0);
    if (_conv) {
      _convNum += 1;
    }

    int sessionSampleNumThreshold = _convMovingSampleNumThreshold + SAMPLE_NUM_EXTRA_DEFAULT;
    float progress_f = 100*_validFrameNum / sessionSampleNumThreshold;
    int progress = _conv ? 100 : std::min(99, me_roundf(progress_f));
    // TODO: enforce upper bound 99 to the roundf?
    _progress = std::max(_progress, progress);

    bool timeout = !_conv && (_validFrameNum > sessionSampleNumThreshold);
    if (_conv || timeout) {
      _emLastValidFrameNum = _dataGlobal->em.validStraightFrameNum;
      _emLastEpiQualityFrameNum = _dataGlobal->em.epiQualityFrameNum;
    }

    WMBC_PRINT(WmbcDbgStg::e_CONVERGENCE, WmbcDbgClr::e_BROWN, "[updateConvergence@CV] id=%s, #conv=%d, "
               "convLastFrame=%d, conv=%d, prog=%d (%.1f), frames: %d/%d",
               WMBC_DOF_ID_STR(_id), _convNum, _convLastFrame, _conv,
               _progress, progress_f, h.sampleNum(), _convMovingSampleNumThreshold);
    WMBC_PRINT(WmbcDbgStg::e_CONVERGENCE, WmbcDbgClr::e_BROWN,
               "[updateConvergence@CV] id=%s, nonConvReason: %s",
               WMBC_DOF_ID_STR(_id), WMBC_ON_STR(m, e_HIGH_VAR, e_SMALL_SAMPLE, e_NON_STABLE, e_LOW_QUALITY)); 
  }

  void CalibVariable::setSampleThreshold() {
    if (_dataGlobal->camera.snowNightMode){
            _convMovingSampleNumThreshold = _dataGlobal->convParams.convSampleNumThresholdNightSnow;
            return;
          }

    if (_dataGlobal->metaParams.runMode != e_Autofix 
        || !_dataGlobal->convParams.useConvMoving
        || _id == e_CAM_HEIGHT
        ) {
      _convMovingSampleNumThreshold = _dataGlobal->convParams.convSampleNumThreshold;
      return;
    }

    float x, xm[4];
    int y, ym[4];
    setSampleThresholdMarks(x, xm, y, ym); // TODO: better naming than marks (ticks?)

    y = ym[3];

    if (x < xm[0]) {
      y = ym[0];
    }

    for (int i = 1; i < 3; ++i) {
      if (x < xm[i] && (xm[i]-xm[i-1] > 0.001*DEG2RAD) && ym[i-1]-ym[i] > 1) {
        y = me_lround((ym[i]*(x-xm[i-1]) - ym[i-1]*(x-xm[i]))/(xm[i]-xm[i-1]));
        break;
      }
      if ((xm[3]-xm[2] > 0.001*DEG2RAD) && ym[2]-ym[3] > 1) {
        y = me_lround((ym[3]*(x-xm[2]) - ym[2]*(x-xm[3]))/(xm[3]-xm[2]));
      }
    }

    _convMovingSampleNumThreshold = std::max(ym[3], std::min(ym[0], y));
  }

  void CalibVariable::setSampleThresholdMarks(float& x, float xm[], int& y, int ym[]) {
    const ResultBook &r = _dataGlobal->results;
    const ConvParams &p = _dataGlobal->convParams;
    VehicleToCam delta = r.curr - r.prevConv;

    switch (_id) {
      case e_YAW:
      case e_HORIZON:
        x = std::max(me_fabs(delta.angles[0]), me_fabs(delta.angles[1]));
        for (int i = 0; i < 4; ++i) {
          xm[i] = p.convMovingMarksFoe[i]; // TODO: change to double array [_id][i]?
          ym[i] = p.convMovingSamples[i];
        }
        break;
      case e_ROLL:
        x = me_fabs(delta.angles[2]);
        for (int i = 0; i < 4; ++i) {
          xm[i] = p.convMovingMarksRoll[i];
          ym[i] = p.convMovingSamples[i];
        } 
        break;
      default:
        x = 0.f;
        for (int i = 0; i < 4; ++i) {
          xm[i] = 0.f;
          ym[i] = 0.f;
        }
        break;
    }
  }

  void CalibVariable::updateConfidence() {
    if (!_conv) {
      return;
    }

    float invCount = 1.f/_convNum;

    // gather statistics
    ConvergenceDataStatistics &s = _convStat;
    float curr = _hist.median();
    s.distSqr = (curr - s.mean)*(curr - s.mean);
    s.sum += curr;
    s.distSqrSum += s.distSqr;
    s.mean = s.sum * invCount;
    s.var = s.distSqrSum * invCount;
    s.varSf = _hist.var();

    for (int j = 0; j < CONF_BIT_NUM; ++j) {
      _confidence[j] = calcConfidenceFunction(j);
    }
  }

  float CalibVariable::calcConfidenceFunction(int idx) {
    const ConfidenceParams &p = _dataGlobal->confParams[idx];
    const ConvergenceDataStatistics &s = _convStat;

    float invWgtDelta = 1.f/(p.wgtNumMax - p.wgtNumMin); //todo: get rid of unnecessayr invs
    float confVarDelta = (p.varMax - p.varMin);
    ASSERT(confVarDelta > EPS && p.varMin > EPS);

    // confidence based on sf-variance within session
    float confVarSf  = std::max(0, std::min(1, (p.varMax - s.varSf)/confVarDelta));

    // confidence based on current value "variance" with absolute limits
    float confDistAbs  = std::max(0, std::min(1, (p.varMax - s.distSqr)/confVarDelta)); // TODO: separate params for distAbs and varSF

    // confidence based on current value "variance" with relative limits
    float confDistRel = confDistAbs; // TODO: initialize with 1
    confVarDelta = (p.varFactorMax - p.varFactorMin)*s.var;
    if (confVarDelta >= p.varMin) {
      confDistRel  = std::max(0, std::min(1, (p.varFactorMax*s.var - s.distSqr)/confVarDelta));
    }

    // weights: confDistXXX are more important when there are more convergences and variance is more meaningful
    float wFac  = std::max(0, std::min(1, (_dataGlobal->algo.total.convNum - p.wgtNumMin)*invWgtDelta)); // TODO: change to _convNum after refactoring
    float w0 = wFac*p.wgtRel;
    float w1 = wFac*p.wgtAbs;

    float conf = w0*confDistRel + w1*confDistAbs + (1 - w0 - w1)*confVarSf;

    return conf;
  }

  void CalibVariable::updateStatus() {
    if (_conv) {
      _status = _inRange ? e_CALIB_OK : e_CALIB_ERROR_OUT_OF_RANGE;
      return;
    }

    int sessionSampleNumThreshold = _convMovingSampleNumThreshold + SAMPLE_NUM_EXTRA_DEFAULT;
    bool timeout = !_conv && (_validFrameNum > sessionSampleNumThreshold);

    if (timeout) {
      _status = e_CALIB_TIMEOUT;
      return;
    }

    if (!_validFrame) {
      _status = e_CALIB_PAUSED;
      return;
    }

    _status = (_dataGlobal->em.valid) ? e_CALIB_CAL : e_CALIB_RUN_ERROR;
  }

  void CalibVariable::updateCoreStatus() {
    switch (_status) {
      case e_CALIB_OK:
        _coreStatus = e_CORE_SUCCESS;
        _coreError = e_CALIB_OK;
        break;
      case e_CALIB_TIMEOUT:
      case e_CALIB_TIMEOUT_TIME:
      case e_CALIB_TIMEOUT_DISTANCE:
        _coreStatus = e_CORE_ERROR;
        _coreError = e_CALIB_TIMEOUT;
        break;
      case e_CALIB_ERROR_OUT_OF_RANGE:
        _coreStatus = e_CORE_ERROR;
        _coreError = e_CALIB_ERROR_OUT_OF_RANGE;
        break;
      case e_CALIB_UNDEFINED:
        _coreStatus = e_CORE_ERROR;
        _coreError = e_CALIB_GENERAL;
        break;
      default:
        _coreStatus = e_CORE_INIT;
        _coreError = e_CALIB_OK;
    }
  }

  void FOEFinder::setFlags() { // TODO: rename to something more appropriate
    ConvValidData &t = _data.algo.total;

    t.validFrame = false;
    t.validFrameNum = _variable[e_YAW].validFrameNum();
    t.validDistance = _variable[e_YAW].validDistance();
    t.validTime = _variable[e_YAW].validTime();
    t.quality = _variable[e_YAW].quality();
    t.nonConvReason = 0;
    for (int i = 0; i < 3; ++i) {
      t.confidence[i] = 1.f;
    }

    for (int i = e_YAW; i < e_CALIB_DOF_NUM; ++i) {
      ConvValidData &s       = _data.algo.single[i];
      const CalibVariable &v = _variable[i];

      s.validFrame           = v.validFrame();
      s.validFrameNum        = v.validFrameNum();
      s.progress             = v.progress();
      s.quality              = v.quality();
      s.validDistance        = v.validDistance();
      s.validTime            = v.validTime();
      s.nonConvReason        = v.nonConvReason();
      s.status               = v.status();
      s.coreStatus           = v.coreStatus();
      s.coreError            = v.coreError();

      t.validFrame = t.validFrame || (v.validFrame() && !v.conv());
      t.nonConvReason = t.nonConvReason | v.nonConvReason();
      if (t.validFrameNum >= v.validFrameNum()) {
        t.validFrameNum = v.validFrameNum();
      }
      if (t.validDistance >= v.validDistance()) {
        t.validDistance = v.validDistance();
      }
      if (t.validTime >= v.validTime()) {
        t.validTime = v.validTime();
      }
      if (t.quality >= v.quality()) {
        t.quality = v.quality();
      }

      for (int j = 0; j < 3; ++j) {
        s.confidence[j] = v.confidence(j);
        // While camera height is not used by WorldModel its confidence is ignored
        if (j < e_CAM_HEIGHT && t.confidence[j] > v.confidence(j)) {
          t.confidence[j] = v.confidence(j);
        }
      }
      s.updateTotalConf();
    }

    t.updateTotalConf();
  }

  void CalibVariable::conv(bool conv, bool skip) {
    if (skip && !conv) {
      return; // turn only false to true but not vice-versa
    }
    _conv          = conv;
    _convNum      += conv ? 1 : 0;
    _convLastFrame = conv ? globalFrameIndex : _convLastFrame;
  }

  void FOEFinder::updateConvergence_NoSeparation() {
    ConvValidData &t = _data.algo.total;
    // enforce yaw and horizon to converge together
    // xor is needed because conv() in order not increment convNum twice
    // if (_variable[e_YAW].conv() != _variable[e_HORIZON].conv()) {
    //   _variable[e_YAW].conv(_variable[e_HORIZON].conv(), true);
    //   _variable[e_HORIZON].conv(_variable[e_YAW].conv(), true);
    // }

    int convMask = 0;
    // int minConvNum = _variable[e_YAW].convNum();
    //int minConvNumArg = e_YAW;

    float progress = 0.f;
    float pRatio[e_CALIB_DOF_NUM] = {1.f, 1.f, 1.f, 1.f};
    int onNum = 0;
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      int mask = 1<<i;
      bool isOn = ((_data.metaParams.calcSwitchMask & mask) != 0);
      onNum += isOn ? 1 : 0;
      pRatio[i] = isOn ? 1.f : 0.f;
    }
    float pR = 1.f/onNum;

    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      ConvValidData &s = _data.algo.single[i];
      const CalibVariable &v = _variable[i];

      s.conv = v.conv();
      s.convNum = v.convNum();
      s.convLastFrame = v.convLastFrame();

      if (v.conv()) {
        int mask = 1<<i;
        convMask = (convMask | mask);
        //if (minConvNum > s.convNum) {
        //minConvNum = s.convNum;
        //minConvNumArg = i;
        //}
      }

      pRatio[i] *= pR;
      progress += pRatio[i]*v.progress();
    }

    // TODO: when convergences are separated this should be changed
    // need to set total conv when all single convNum are equal
    t.conv = ((convMask & _data.metaParams.calcSwitchMask) == _data.metaParams.calcSwitchMask);

    t.progress = t.conv ? 100 : std::min(99, me_roundf(progress));
    if (t.conv) {
      t.convNum +=1;
      t.convLastFrame = globalFrameIndex;
      _data.results.prevConv = _data.results.curr;
    }
  }

  void FOEFinder::updateConvergence() {
    ConvValidData &t = _data.algo.total;
    // enforce yaw and horizon to converge together
    // conv() xor is needed in order not increment convNum twice
    // if (_variable[e_YAW].conv() != _variable[e_HORIZON].conv()) {
    //   _variable[e_YAW].conv(_variable[e_HORIZON].conv(), true);
    //   _variable[e_HORIZON].conv(_variable[e_YAW].conv(), true);
    // }

    int convMask = 0;
    int progressSingle = 0;
    float progress = 0.f;

    float pRatio[e_CALIB_DOF_NUM] = {1.f, 1.f, 1.f, 1.f};
    int onNum = 0;
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      int mask = 1<<i;
      bool isOn = ((_data.metaParams.calcSwitchMask & mask) != 0);
      onNum += isOn ? 1 : 0;
      pRatio[i] = isOn ? 1.f : 0.f;
    }
    float pR = 1.f/onNum;

    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      ConvValidData &s = _data.algo.single[i];
      const CalibVariable &v = _variable[i];

      s.conv = v.conv();
      s.convNum = v.convNum();
      s.convLastFrame = v.convLastFrame();
      s.progress = v.progress();

      bool totConvCond = (_data.metaParams.runMode == e_Autofix) ? (s.convNum > t.convNum) : s.conv;
      if (totConvCond) {
        int mask = 1<<i;
        convMask = (convMask | mask);
        progressSingle = 100;
      } else {
        progressSingle = s.progress;
      }

      pRatio[i] *= pR;
      progress += pRatio[i]*progressSingle;

      if (s.conv) {
        if (i < e_CAM_HEIGHT) {
          _data.results.prevConv.angles[i] = _data.results.curr.angles[i];
          _data.results.prevConv.update(_data.results.prevConv.angles);
        } else {
          _data.results.prevConv.camh = _data.results.curr.camh;
        }
      }

      if (i == e_CAM_HEIGHT) {
	// check SDM for active suspension on camh convergence.
	bool sdm_value = validateSDM(s.conv,_data.convParams.camhSdmValue);
	_data.convParams.camhSdmValue = sdm_value;
      }
    }

    t.conv = ((convMask & _data.metaParams.calcSwitchMask) == _data.metaParams.calcSwitchMask);
    t.progress = t.conv ? 100 : std::min(99, me_roundf(progress));
    if (t.conv) {
      t.convNum +=1;
      t.convLastFrame = globalFrameIndex;
      //_data.results.prevConv = _data.results.curr;
    }
  }

  bool FOEFinder::validateSDM(bool conv, bool prev_value){
    bool sdm_value;
    for(int i = 0; i < CameraInfo::e_NUM_OF_CAM_INSTANCES; ++i) {
      WMBC_CAM_SKIP(i);

      if (conv) {
	CameraInfo::CameraInstance inst = CameraInfo::CameraInstance(i);
	float baselineCamH = CameraInfo::origCameraHeight(inst);
	float dynamicCamHDelta = _data.vehicle.dynamicCameraHeightDelta;
	float camHFormula = me_abs(baselineCamH + dynamicCamHDelta - _data.results.cams[i].camh)
	  / (baselineCamH + dynamicCamHDelta);

	//flag SDM21. NOTE: in previous versions (using an obsolete camHeightRangePlausible flag)
	//we made a fallback to the base camh this is not done anymore.
	sdm_value = camHFormula > _data.metaParams.camHeightRangeSevere;
      } else {
	sdm_value = prev_value;
      }

      SDM::setFailedDiagnosic(SDM::CAMERA_HEIGHT_LIMIT_EXCEEDED, sdm_value);
    }
    return sdm_value;
  }

  void ConvValidData::updateTotalConf() {
    totalConfidence = confidence[1];
    totalConfGrade = 0;
    for (int j = 0; j < CONF_BIT_NUM; ++j) {
      if (confidence[j] > CONF_TH) {
        totalConfGrade |= (1<<j);
      }
    }
  }

  bool ConvValidData::isSessionEnded() {
      bool isTimeout = (status == e_CALIB_TIMEOUT
                        || status == e_CALIB_TIMEOUT_TIME
                        || status == e_CALIB_TIMEOUT_DISTANCE);

      return (conv || isTimeout);
  }

  // todo: implement the quick mode (consider discarding the iheritant object for this)

  void FOEFinder::updateFoeResetFlag() {
    // reset WorldModel Camera calibration in one of these cases:
    // 1. one of the values converged and it has a substantial correction
    // 2. accumulated enough valid-vehicle-frames but egomotion is mostly invalid
    // 3. accumulated enough valid-vehicle-frames and egomotion output is present and indicates a large correction

    WM_ResetSignal &reset = _data.algo.wmFoeReset;

    reset = e_WMRESET_NONE;

    if (_data.algo.quickMode) {
      return;
    }
    static bool disableWmFoeReset = Debug::Args::instance().existsParameter("-sWmbc-skipUpdateWmCalib");
    if (disableWmFoeReset) {
     reset = e_WMRESET_SKIP;
      return;
    }

    // todo: make sure that:
    // 1. no wmbc reset takes place before cam height has chance to converge
    // 2. flag is turned back to off while waiting for cameraheight to converge

    // reset = _variable[e_YAW].conv() && _variable[e_HORIZON].conv() && _variable[e_ROLL].conv();
    // if (reset) {
    //   WMBC_PRINT(WmbcDbgStg::e_FOE_RESET, WmbcDbgClr::e_BROWN, "[updateFoeResetFlag] wmFoeReset ON (convergence)", 0);
    //   return;
    // }
    VehicleToCam& curr = _data.results.curr;
    VehicleToCam& prev = _data.results.prevWmReset;
    VehicleToCam& delta = _data.results.deltaWmReset;

    double deltaTh[3] = {0.3, 0.3, 0.5}; // [deg] TODO: create properties
    delta = curr - prev;
    for (int i = 0; i <= e_ROLL; ++i) {
      bool largeDelta = me_fabs(delta.angles[i]) > deltaTh[i]*DEG2RAD;
      if (_data.algo.single[i].conv && largeDelta) {
        reset = e_WMRESET_CONV;
        prev = curr; // TODO: probably need to change individually
        WMBC_PRINT(WmbcDbgStg::e_FOE_RESET, WmbcDbgClr::e_BROWN,
                   "[updateFoeResetFlag] wmFoeReset ON (%s convergence %.1fdeg)",
                   WMBC_DOF_ID_STR(i), delta.angles[i]*RAD2DEG);
        return;
      }
    }

    static int startResetFrame = Debug::Args::instance().getStickyValue("-sWmbc-startResetFrame", 20);
    static float minValidFrameRatio = Debug::Args::instance().getStickyValue("-sWmbc-validResetFrames", 0.5);
    static int minSample = Debug::Args::instance().getStickyValue("-sWmbc-minResetSample", 1);

    WMBC_PRINT(WmbcDbgStg::e_FOE_RESET, WmbcDbgClr::e_BROWN, "[updateFoeResetFlag] valid-vehicle-frames=%d %s %d=startResetFrame",
               _data.vehicle.validFrameNum,
               (_data.vehicle.validFrameNum < startResetFrame ? "<" : ">="),
               startResetFrame);

    if (_data.vehicle.validFrameNum < startResetFrame) {
      WMBC_PRINT(WmbcDbgStg::e_FOE_RESET, WmbcDbgClr::e_BROWN, "[updateFoeResetFlag] wmFoeReset OFF (vehicle frames)", 0);
      return;
    }

    WMBC_PRINT(WmbcDbgStg::e_FOE_RESET, WmbcDbgClr::e_BROWN, "[updateFoeResetFlag] yawSampleNum=%d %s %d, horSampleNum=%d %s %d (minSample)",
               _variable[e_YAW].hist().sampleNum(), WMBC_ISLESS(_variable[e_YAW].hist().sampleNum(), minSample), minSample,
               _variable[e_HORIZON].hist().sampleNum(), WMBC_ISLESS(_variable[e_HORIZON].hist().sampleNum(), minSample));

    if (_variable[e_YAW].hist().sampleNum() < minSample || _variable[e_HORIZON].hist().sampleNum() < minSample) {
      WMBC_PRINT(WmbcDbgStg::e_FOE_RESET, WmbcDbgClr::e_BROWN, "[updateFoeResetFlag] wmFoeReset OFF (min sample)", 0);
      return;
    }

    bool isReset = (_data.em.validFrames < (int)(minValidFrameRatio*_data.vehicle.validFrameNum));

    WMBC_PRINT(WmbcDbgStg::e_FOE_RESET, WmbcDbgClr::e_BROWN, "[updateFoeResetFlag] wmFoeReset %s, em-valid-frames=%d %s %d (%.2f = minValidFramesTh)",
               (isReset ? "ON (EM stagger)" : ""), _data.em.validFrames,
               WMBC_ISLESS(_data.em.validFrames, (int)(minValidFrameRatio*_data.vehicle.validFrameNum)),
               (int)(minValidFrameRatio*_data.vehicle.validFrameNum), minValidFrameRatio*_data.vehicle.validFrameNum);

    if (isReset) {
      reset = e_WMRESET_EM_STAGGER;
      return;
    }

    bool large[2] = {false, false};
    for (int i=0; i<2; ++i) {
      float dAngle = me_fabsf(delta.angles[i]);
      large[i] = (dAngle > _data.metaParams.foeDeltaThresholdFS[i]);
      large[i] = large[i] && (_variable[i].stableMedianCount() > 10);
      large[i] = large[i] && (_data.algo.single[i].convNum == 0);
    }

    isReset = (large[e_YAW] || large[e_HORIZON]);

    if (isReset) {
      reset = e_WMRESET_LARGE;
      prev = curr;
    }

    WMBC_PRINT(WmbcDbgStg::e_FOE_RESET, WmbcDbgClr::e_BROWN, "[updateFoeResetFlag] wmFoeReset %s, yawDeltaLm2=%.2f (%.2fdeg) %s %.2f (%.2fdeg) = th, "
               "horDeltaLm2=%.2f (%.2fdeg) %s %.2f (%.2fdeg) = th",
               (isReset ? "ON (large delta)" : "OFF"),
               me_fabsf(delta.angles[e_YAW])*_data.camera.focalLm2, me_fabsf(delta.angles[e_YAW])*RAD2DEG, (large[e_YAW] ? ">" : "<="),
              _data.metaParams.foeDeltaThresholdFS[e_YAW]*_data.camera.focalLm2, _data.metaParams.foeDeltaThresholdFS[e_YAW]*RAD2DEG,
              me_fabsf(delta.angles[e_HORIZON])*_data.camera.focalLm2, me_fabsf(delta.angles[e_HORIZON])*RAD2DEG, (large[e_HORIZON] ? ">" : "<="),
              _data.metaParams.foeDeltaThresholdFS[e_HORIZON]*_data.camera.focalLm2, _data.metaParams.foeDeltaThresholdFS[e_HORIZON]*RAD2DEG);
  }

  void FOEFinder::updateStatus() {
    ConvValidData &t = _data.algo.total;
    if (t.conv) {
      t.status = t.inRange ? e_CALIB_OK : e_CALIB_ERROR_OUT_OF_RANGE;
      return;
    }

    bool timeoutYaw = (_variable[e_YAW].status() == e_CALIB_TIMEOUT);
    bool timeoutRoll = (_variable[e_ROLL].status() == e_CALIB_TIMEOUT);
    bool timeout = (timeoutYaw && timeoutRoll);

    if (timeout) {
      t.status = e_CALIB_TIMEOUT;
      return;
    }

    if (!t.validFrame) {
      t.status = e_CALIB_PAUSED;
      return;
    }

    t.status = (_data.em.valid) ? e_CALIB_CAL : e_CALIB_RUN_ERROR;
  }

  void FOEFinder_SPC::updateStatus() {
    ConvValidData &t = _data.algo.total;
    if (t.conv) {
      t.status = t.inRange ? e_CALIB_OK : e_CALIB_ERROR_OUT_OF_RANGE;
      return;
    }

    bool timeoutDist = (_data.metaParams.maxDist > 0 && _data.vehicle.trajLength > _data.metaParams.maxDist);
    bool timeoutTime = (_data.metaParams.maxTime > 0 && _data.vehicle.drivingTime > _data.metaParams.maxTime);
    if (timeoutDist) {
      t.status = e_CALIB_TIMEOUT_DISTANCE;
      updateVariablesTimeout(e_CALIB_TIMEOUT_DISTANCE);
      return;
    }

    if (timeoutTime) {
      t.status = e_CALIB_TIMEOUT_TIME;
      updateVariablesTimeout(e_CALIB_TIMEOUT_TIME);
      return;
    }

    bool noConv =  (_data.algo.total.progress == 99);
    bool runForever = (_data.metaParams.maxAttempts == 0);
    bool exceededAttempts = (_data.algo.sessionNum >= _data.metaParams.maxAttempts);
    if (noConv && !runForever && exceededAttempts) {
      t.status = e_CALIB_TIMEOUT;
      return;
    }

    if (!t.validFrame) {
      t.status = e_CALIB_PAUSED;
      return;
    }

    t.status = (_data.em.valid) ? e_CALIB_CAL : e_CALIB_RUN_ERROR;
  }

  void FOEFinder::updateVariablesTimeout(CalibStatus status) {
    for (int i = e_YAW; i < e_CALIB_DOF_NUM; ++i) {
      _variable[i].status(status);
      _variable[i].coreStatus(e_CORE_ERROR);
      _variable[i].coreError(e_CALIB_TIMEOUT);
      _data.algo.single[i].status = status;
      _data.algo.single[i].coreStatus = e_CORE_ERROR;
      _data.algo.single[i].coreError = e_CALIB_TIMEOUT;
    }
  }



  bool FOEFinder::setIsCalibInRange(int idx, float val) {
    MetaParams &mp   = _data.metaParams;
    ConvValidData &s = _data.algo.single[idx];
    CalibVariable &v = _variable[idx];

    bool inRange = (val >= mp.foeRange[idx][0] && val <= mp.foeRange[idx][1]);
    v.inRange(inRange);

    if (v.conv()) {
      CalibStatus stat = inRange ? e_CALIB_OK : e_CALIB_ERROR_OUT_OF_RANGE;
      CalibCoreStatus coreStat = inRange ? e_CORE_SUCCESS : e_CORE_ERROR;
      v.status(stat);
      v.coreStatus(coreStat);
      v.coreError(stat);
      s.status = stat;
      s.coreStatus = coreStat;
      s.coreError = stat;
    }

    return inRange;
  }

  void FOEFinder::setIsCalibInRange() {
    // When using internal wmbc limits each signal can be compared separately to its limit
    // and this is done inside each _variable.
    // When using system limits (cameraInfo::min/max) the comparison is done here post _variable
    // execution because the translation from angles to pixels is coupling yaw and pitch.
    // I could in principle translate the limits from pixels to angles, but I want to avoid
    // the potential slight limit value change due to the distortion process.
    ConvValidData &t = _data.algo.total;

    if (_data.metaParams.useSpecialLimits) {
      t.inRange = true;
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        int mask = 1<<i;
        bool isOn = ((_data.metaParams.calcSwitchMask & mask) != 0);
        if (isOn) {
          t.inRange = t.inRange && _variable[i].inRange();
        }
      }
      return;
    }

    PixelLm2_f foeLm2 = _data.results.curr.foeLm2;
    foeLm2 += _data.camera.origin[CameraInfo::e_FORWARD].distDeltaLm2;
    int yawLm2 = (int)me_roundf(foeLm2.X());
    int horLm2 = (int)me_roundf(foeLm2.Y());
    horLm2 = (horLm2>>1)<<1;
    float roll = _data.results.curr.angles[e_ROLL];

    int mask = 1<<e_YAW;
    bool isOn = ((_data.metaParams.calcSwitchMask & mask) != 0);
    if (isOn) {
      t.inRange = setIsCalibInRange(e_YAW, yawLm2);
    }
    mask = 1<<e_HORIZON;
    isOn = ((_data.metaParams.calcSwitchMask & mask) != 0);
    if (isOn) {
      t.inRange = t.inRange && setIsCalibInRange(e_HORIZON, horLm2);
    }
    mask = 1<<e_ROLL;
    isOn = ((_data.metaParams.calcSwitchMask & mask) != 0);
    if (isOn) {
      t.inRange = t.inRange && setIsCalibInRange(e_ROLL, roll);
    }
  }

  void FOEFinder::setResults_NoSeparation() {
    if (_data.vehicle.validFrameNum == 0) { // we are at start or after conv with no measurements yet
      setInitialResults_NoSeparation();
    }

    // if (globalFrameIndex == 0) {
    //   _data.results.prevConv = _data.results.curr;
    // }

    // TODO: adjust roll for vehicleRoll
    double angles[3] = {0.0, 0.0, 0.0};
    angles[e_YAW] = _variable[e_YAW].resultInterim();
    angles[e_HORIZON] = _variable[e_HORIZON].resultInterim();
    angles[e_ROLL] = _variable[e_ROLL].resultInterim();
    double camh = _variable[e_CAM_HEIGHT].resultInterim();
    _data.results.curr.update(angles, camh);

    if (false) { //(_rotateFoe) { // TODO: check once and for all if this is relevant at all?
      _data.results.curr.rotate();
    }

    setAllCamerasResults();
  }

  void FOEFinder::setInitialResults_NoSeparation() {
    // If we converged already init with last conv results.
    // Else set values to etc values (which is the origin of the image, yaani 0,0 on distorted)
    if (_data.algo.total.convNum > 0) {
      _data.results.curr = _data.results.prevConv;
      return;
    }

    //PixelLm2_f foe(0.f, 0.f);
    PixelLm2_f foe;
    foe.X() = foe.Y() = 0.f;
    float roll = CameraInfo::rollAngle(CameraInfo::e_FORWARD);
    float camh = CameraInfo::cameraHeight(CameraInfo::e_FORWARD);
    _data.results.curr.update(foe, roll, camh);
    _data.results.prevConv = _data.results.curr;
  }

//  void FOEFinder::setResults() {
//    // Set results with current median unless we've started a new session:
//    // If we converged already init with last conv results.
//    // Else set values to etc values (for yaw/horizon it's the origin of the image, yaani 0,0 on distorted)
//    const AlgoData& d = _data.algo;
//    VehicleToCam& p = _data.results.prevConv;
//    bool noConv[e_CALIB_DOF_NUM] = {false, false, false, false}; // Do we need ad hoc to init prevConv with currConv
//
//    double val[e_CALIB_DOF_NUM] = {0.0, 0.0, 0.0, 0.0};
//    double pval[e_CALIB_DOF_NUM] = {0.0, 0.0, 0.0, 0.0};
//    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
//      val[i]  = _variable[i].resultInterim();
//      pval[i] = (i <= e_ROLL) ? p.angles[i] : p.camh;
//    }
//
//    double etc[e_CALIB_DOF_NUM] = {0.f, 0.f,
//                                   CameraInfo::rollAngle(CameraInfo::e_FORWARD),
//                                   CameraInfo::cameraHeight(CameraInfo::e_FORWARD)};
//
//    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
//      if (d.single[i].validFrameNum == 0) {
//        noConv[i] = (d.single[i].convNum == 0);
//        val[i]    = noConv[i] ? etc[i] : pval[i];
//        pval[i]   = noConv[i] ? val[i] : pval[i];
//      }
//    }
//
//    if (noConv[e_YAW]) {
//      Float::MEgeo::Point2D foe(0.f, 0.f);
//      p.update(foe, pval[e_ROLL], pval[e_CAM_HEIGHT]);
//    } else if (noConv[e_ROLL] || noConv[e_CAM_HEIGHT]) {
//      p.update(pval, pval[e_CAM_HEIGHT]);
//    }
//
//    if (d.single[e_YAW].validFrameNum == 0 && noConv[e_YAW]) { // horizon converges together with yaw
//      Float::MEgeo::Point2D foe(0.f, 0.f);
//      _data.results.curr.update(foe, val[e_ROLL], val[e_CAM_HEIGHT]);
//      setAllCamerasResults();
//      return;
//    }
//    _data.results.curr.update(val, val[e_CAM_HEIGHT]);
//    setAllCamerasResults();
//  }
  void FOEFinder::resetResultsToEtc() {
    //PixelLm2_f foe(0.f, 0.f);
    PixelLm2_f foe;
    foe.X() = foe.Y() = 0.f;
    double roll = CameraInfo::rollAngle(CameraInfo::e_FORWARD);
    double camh = CameraInfo::cameraHeight(CameraInfo::e_FORWARD);
    _data.results.curr.update(foe, roll, camh);
    _data.results.prevConv = _data.results.curr;
    _data.results.prevWmReset = _data.results.curr;
    setAllCamerasResults();
  }

  void FOEFinder::setResults() {
    // Set results with current median unless we've started a new session:
    // If we converged already init with last conv results.
    // Else set values to etc values (for yaw/horizon it's the origin of the image, yaani 0,0 on distorted)
    const AlgoData& d = _data.algo;
    VehicleToCam& p = _data.results.prevConv;

    double val[e_CALIB_DOF_NUM] = {0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      if (d.single[i].validFrameNum > 0) {
        val[i]  = _variable[i].resultInterim();
      } else {
        val[i] = (i <= e_ROLL) ? p.angles[i] : p.camh;
      }
    }

   _data.results.curr.update(val, val[e_CAM_HEIGHT]);
    setAllCamerasResults();
    setIsCalibInRange();
  }

  void FOEFinder::setAllCamerasResults() {
    VehicleToCam &src = _data.results.curr;

    for (int i=0 ; i < CameraInfo::e_NUM_OF_CAM_INSTANCES ; i++) {
      WMBC_CAM_SKIP(i)
      CameraInfo::CameraInstance inst = (CameraInfo::CameraInstance)i;

      if (inst == CameraInfo::e_FORWARD) {
        _data.results.cams[i] = _data.results.curr;
        continue;
      }

      VehicleToCam &tgt = _data.results.cams[i];
      transformVehicleToCam(inst, src, tgt);
    }
  }

  // TODO: reconsider the necessity of this container and function
  void FOEFinder::setDebugShowData() {
    VehicleData &vd = _data.vehicle;
    ConvValidData &t = _data.algo.total;

    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      ConvValidData &s = _data.algo.single[i];
      _debugShowData.validFramesNumIndividual[i] = s.validFrameNum; //_validFramesNumIndividual[i];
      //_debugShowData.validFramesNumIndividualPercentage[i] = _validFramesForWMNum > 0 ? me_roundf(100*_validFramesNumIndividual[i]/_validFramesForWMNum) : 0;
      _debugShowData.validFramesNumIndividualPercentage[i] = vd.validFrameNum > 0 ? me_roundf(100*s.validFrameNum/vd.validFrameNum) : 0;
      _debugShowData.validFrameIndividual[i] = s.validFrame; //_validFrameIndividual[i];
      _debugShowData.singleConverged[i] = s.conv;
      _debugShowData.singleStat[i] = _data.algo.single[i];
    }

    _debugShowData.currDistanceToTarget = 0;
    _debugShowData.trajLength = vd.trajLength;
    _debugShowData.isEndTraj = false;
    _debugShowData.validFramesVehicle = vd.validFrameNum; //_validFramesForWMNum;
    if (!_data.algo.total.conv) {
      //_debugShowData.validFramesVehiclePercentage = me_roundf(100.f*_validFramesForWMNum / (globalFrameIndex + 1 - _data.algo.single[].totalConvLastFrame);
      _debugShowData.validFramesVehiclePercentage = me_roundf(100.f*vd.validFrameNum / (globalFrameIndex + 1 - t.convLastFrame));
    }
    
    _debugShowData.progress = t.progress; //_conv.totalProgress;

    _debugShowData.sessionNumber = -1;
    _debugShowData.maxAttempts = -1;

    if (_data.algo.quickMode) {
      return;
    }

    // WMBC_PRINT(WmbcDbgClr::e_BROWN, "[setDebugShowData] validFramesVehiclePercentage = r(100*%d/(%d - %d)) = r(%d/%d) = r(%.2f) = %d",
    //            _validFramesForWMNum, globalFrameIndex + 1, _conv.totalConvLastFrame,
    //            100*_validFramesForWMNum, globalFrameIndex + 1 - _conv.totalConvLastFrame,
    //            (100.f*_validFramesForWMNum/(globalFrameIndex + 1 - _data.algo.single[].totalConvLastFrame),
    //            _debugShowData.validFramesVehiclePercentage);


    WmbcOutputIF::instance().toItrkDebug(&_data);
  }

  void FOEFinder_AutofixFailsafe::setDebugShowData() {
    FOEFinder::setDebugShowData();

    //WmbcOutputIF::instance().toItrkDebug(&_data);
  }

  void FOEFinder_SPC::setDebugShowData() {
    FOEFinder::setDebugShowData();
    _debugShowData.sessionNumber = _data.algo.sessionNum;
    _debugShowData.maxAttempts = _data.metaParams.maxAttempts;
  }

  void FOEFinder::fillModelIF() {
    fillWmbcIF();
  }

  void FOEFinder::fillWmbcIF() {
    WmbcIF *m = &_wmbcIF.editable();

    bool updateNonConv = (_data.metaParams.runMode == e_SPC && _data.algo.total.status == e_CALIB_TIMEOUT);

    for (int i=0 ; i < CameraInfo::e_NUM_OF_CAM_INSTANCES ; i++) {
      WMBC_CAM_SKIP(i)
      CameraInfo::CameraInstance inst = (CameraInfo::CameraInstance)i;
      VehicleToCam &v2c = _data.results.cams[i];

      m->interimYawAngle[i] = v2c.angles[0];
      m->interimPitchAngle[i] = v2c.angles[1];
      m->interimRoll[i] = v2c.angles[2];
      m->interimCameraHeight[i] = v2c.camh;

      float yawDistLm2 = v2c.foeLm2.X(); // TODO: copy to IF as Point2D
      float horDistLm2 = v2c.foeLm2.Y();

      int interimYawLm2 = (int)me_roundf(yawDistLm2 + _data.camera.origin[i].distDeltaLm2.X());
      int interimHorizonLm2 = (int)me_roundf(horDistLm2 + _data.camera.origin[i].distDeltaLm2.Y());
      int interimYawDeltaL0FromEtc = me_roundf(0.25*yawDistLm2); // TODO: consider round(yawDistLm2)>>2
      int interimHorizonDeltaL0FromEtc = me_roundf(0.25*horDistLm2);

      // TODO:
      //  1. only horizonFull need to be even
      //  2. decide which is the desirable rounding method
      //  3. make sure values on drawRpc are the same as in IF

      m->interimYawLm2[i] = (interimYawLm2>>1)<<1; // need to make sure that yawFull and horizonFull are even because there is no justice in the world
      m->interimHorizonLm2[i] = (interimHorizonLm2>>1)<<1;

      m->interimYawDeltaL0FromEtc[i] = interimYawDeltaL0FromEtc;
      m->interimHorizonDeltaL0FromEtc[i] = interimHorizonDeltaL0FromEtc;
      m->interimYawDeltaLm2FromEtc_f[i] = yawDistLm2;
      m->interimHorizonDeltaLm2FromEtc_f[i] = horDistLm2;
      m->interimYawDeltaL0FromLastConv[i] = ((m->interimYawLm2[i])>>2) - m->yawL0[i];
      m->interimHorizonDeltaL0FromLastConv[i] = ((m->interimHorizonLm2[i])>>2) - m->horizonL0[i];

      m->interimAutoFixOnlineLm2_yaw[i] = m->interimYawLm2[i] - CameraInfo::yawFull(inst);
      m->interimAutoFixOnlineLm2_horizon[i] = m->interimHorizonLm2[i] - CameraInfo::horizonFull(inst);

      m->interimVehicleToCam[i] = v2c;
      m->vehicleToCamWmReset = _data.results.prevWmReset;

      if (_data.algo.single[e_YAW].conv || updateNonConv) {
        m->yawL0[i] = (m->interimYawLm2[i])>>2;
        m->yawLm2[i] = m->interimYawLm2[i];
        m->yawAngle[i] = m->interimYawAngle[i];
        m->yawDeltaL0FromEtc[i] = m->interimYawDeltaL0FromEtc[i];
        m->yawDeltaLm2FromEtc_f[i] = m->interimYawDeltaLm2FromEtc_f[i];
        m->yawDeltaL0FromLastConv[i] = m->interimYawDeltaL0FromLastConv[i];
        m->autoFixOnlineLm2_yaw[i] = m->interimAutoFixOnlineLm2_yaw[i];
        m->vehicleToCam[i].update(m->yawAngle[i], e_YAW);
      }

      if (_data.algo.single[e_HORIZON].conv || updateNonConv) {
        m->horizonL0[i] = (m->interimHorizonLm2[i])>>2;
        m->horizonLm2[i] = m->interimHorizonLm2[i];
        m->pitchAngle[i] = m->interimPitchAngle[i];
        m->horizonDeltaL0FromEtc[i] = m->interimHorizonDeltaL0FromEtc[i];
        m->horizonDeltaLm2FromEtc_f[i] = m->interimHorizonDeltaLm2FromEtc_f[i];
        m->horizonDeltaL0FromLastConv[i] = m->interimHorizonDeltaL0FromLastConv[i];
        m->autoFixOnlineLm2_horizon[i] = m->interimAutoFixOnlineLm2_horizon[i];
        m->vehicleToCam[i].update(m->pitchAngle[i], e_HORIZON);
      }

      if (_data.algo.single[e_ROLL].conv || updateNonConv) {
        m->roll[i] = m->interimRoll[i];
        m->vehicleToCam[i].update(m->roll[i], e_ROLL);
      }

      if (_data.algo.single[e_CAM_HEIGHT].conv || updateNonConv) {
        m->cameraHeight[i] = m->interimCameraHeight[i];
        m->vehicleToCam[i].update(m->cameraHeight[i], e_CAM_HEIGHT);
      }

      if (globalFrameIndex ==0 ) {
        m->autoFix_yaw[i] = CameraInfo::initialAutoFix_yaw(inst);
        m->autoFix_horizon[i] = CameraInfo::initialAutoFix_horizon(inst);
        m->vehicleToCam[i] = v2c;
      }
      if (inst == CameraInfo::e_FORWARD) {
        WMBC_PRINT(WmbcDbgStg::e_MODEL, WmbcDbgClr::e_BLUE, "[fillWmbcIF] for protocol: af_yaw = autoFix_yaw+yawDeltaL0FromEtc = %d + %d = %d",
                   m->autoFix_yaw[i], m->yawDeltaL0FromEtc[i], m->autoFix_yaw[i]+m->yawDeltaL0FromEtc[i]);
        WMBC_PRINT(WmbcDbgStg::e_MODEL, WmbcDbgClr::e_BLUE, "[fillWmbcIF] for protocol: af_hor = autoFix_horizon+horizonDeltaL0FromEtc = %d + %d = %d",
                   m->autoFix_horizon[i], m->horizonDeltaL0FromEtc[i], m->autoFix_horizon[i]+m->horizonDeltaL0FromEtc[i]);
      }
    }

//    m->yawLm2Primary_f = _foe.yawLm2Primary_f;
//    m->horizonLm2Primary_f = _foe.horizonLm2Primary_f;
//    m->yawDeltaLm2FromeEtcPrimary_f = _foe.yawDeltaLm2FromEtcPrimary_f;
//    m->horizonDeltaLm2FromeEtcPrimary_f = _foe.horizonDeltaLm2FromEtcPrimary_f;
//    m->rollPrimary = _foe.rollPrimary;
//    m->camhPrimary = _foe.camhPrimary;

    m->status = _data.algo.total.status;
    m->pauseReason = _data.algo.pauseReason;
    m->progress = _data.algo.total.progress;

    for (int i = e_YAW; i < e_CALIB_DOF_NUM; ++i) { // TODO: iteration type
      ConvValidData &s = _data.algo.single[i];
      m->singleConverged[i] = s.conv; //_conv.conv[i];
      m->singleQuality[i] = s.quality; //_conv.quality[i]; // TODO: change to conf
      m->singleConfidence[i] = s.totalConfidence;
      m->singleProgress[i] = s.progress; //_conv.progress[i];
      m->singleConvFailReason[i] = s.nonConvReason; //_conv.condMask[i];
      m->singleStatus[i] = s.coreStatus;
      m->singleError[i] = s.coreError;
    }
    m->runMode = _data.metaParams.runMode; //_params.runMode;

    m->convergedNum = _data.algo.total.convNum; //_conv.totalConvNum;
    m->lastConvFrame = _data.algo.total.convLastFrame; //_conv.totalConvLastFrame;

    m->quality = _data.algo.total.quality; //_conv.totalQuality; // TODO: replace with conf
    m->confidence = _data.algo.total.totalConfidence;
    m->confidenceGrade = _data.algo.total.totalConfGrade;

    m->totalDistance = _data.vehicle.trajLength; //_vehicleData.trajLength;
    m->validDistance = _data.vehicle.validDistance; //_vehicleData.totalValidTrajLength;
    m->totalTime = _data.vehicle.trajTime; //_vehicleData.trajTime;
    m->validTime = _data.vehicle.validTime; //_vehicleData.totalValidTrajTime;
    m->validAlgoFrameNum = _data.algo.total.validFrameNum; //_validAlgoFrameNum;
    m->validVehicleFrameNum = _data.vehicle.validFrameNum; //_validFramesForWMNum;

    m->debugShowData = _debugShowData;

    m->wmFoeReset = _data.algo.wmFoeReset; //_wmFoeReset;

    _wmbcIF.update();

    if (_data.algo.quickMode) {
      return;
    }
    WmbcOutputIF::instance().toItrkFinal(m, &_data);

    WMBC_PRINT(WmbcDbgStg::e_MODEL, WmbcDbgClr::e_PURPLE, "[fillModelIF@WMBC] conv: %d, lastConvFrame: %d, progress: %d", 
               m->convergedNum, m->lastConvFrame, _data.algo.total.progress);
//     WMBC_PRINT(WmbcDbgStg::e_MODEL, 2, "[fillModelIF@WMBC] status: %s", 
//                (_data.algo.total.status == e_CALIB_OK ? "OK" : 
//                 (_data.algo.total.status == e_CALIB_CAL ? "CAL" : 
//                  (_data.algo.total.status == e_CALIB_PAUSED ? "PAUSED" : 
//                   (_data.algo.total.status == e_CALIB_RUN_ERROR ? "ERROR" : "TIMEOUT")))));
    WMBC_PRINT(WmbcDbgStg::e_MODEL, 2, "[fillModelIF@WMBC] status: %s", 
               WMBC_STATUS_STR(_wmbcIF.getObj().status));

    WMBC_PRINT(WmbcDbgStg::e_MODEL, WmbcDbgClr::e_BLUE, "[fillWmbcIF] yaw : val=%d, %s #c=%d #f=%d p=%d",
               _wmbcIF.getObj().interimYawLm2[0], WMBC_STATUS_STR(_data.algo.single[e_YAW].status),
               _data.algo.single[e_YAW].convNum,
               _data.algo.single[e_YAW].validFrameNum, _wmbcIF.getObj().singleProgress[e_YAW]);
    WMBC_PRINT(WmbcDbgStg::e_MODEL, WmbcDbgClr::e_BLUE, "[fillWmbcIF] hor : val=%d, %s #c=%d #f=%d p=%d",
               _wmbcIF.getObj().interimHorizonLm2[0], WMBC_STATUS_STR(_data.algo.single[e_HORIZON].status),
               _data.algo.single[e_HORIZON].convNum,
               _data.algo.single[e_HORIZON].validFrameNum, _wmbcIF.getObj().singleProgress[e_HORIZON]);
    WMBC_PRINT(WmbcDbgStg::e_MODEL, WmbcDbgClr::e_BLUE, "[fillWmbcIF] roll: val=%.4f %s #c=%d #f=%d p=%d",
               _wmbcIF.getObj().interimRoll[0], WMBC_STATUS_STR(_data.algo.single[e_ROLL].status),
               _data.algo.single[e_ROLL].convNum,
               _data.algo.single[e_ROLL].validFrameNum, _wmbcIF.getObj().singleProgress[e_ROLL]);
    WMBC_PRINT(WmbcDbgStg::e_MODEL, WmbcDbgClr::e_BLUE, "[fillWmbcIF] camh: val=%.2f %s #c=%d #f=%d p=%d",
               _wmbcIF.getObj().interimCameraHeight[0], WMBC_STATUS_STR(_data.algo.single[e_CAM_HEIGHT].status),
               _data.algo.single[e_CAM_HEIGHT].convNum,
               _data.algo.single[e_CAM_HEIGHT].validFrameNum, _wmbcIF.getObj().singleProgress[e_CAM_HEIGHT]);
    WMBC_PRINT(WmbcDbgStg::e_MODEL, WmbcDbgClr::e_BLUE, "[fillWmbcIF] core Status: %s %s %s %s",
               WMBC_CORE_STATUS_STR(_wmbcIF.getObj().singleStatus[e_YAW]),
               WMBC_CORE_STATUS_STR(_wmbcIF.getObj().singleStatus[e_HORIZON]),
               WMBC_CORE_STATUS_STR(_wmbcIF.getObj().singleStatus[e_ROLL]),
               WMBC_CORE_STATUS_STR(_wmbcIF.getObj().singleStatus[e_CAM_HEIGHT]));
    WMBC_PRINT(WmbcDbgStg::e_MODEL, WmbcDbgClr::e_BLUE, "[fillWmbcIF] core Error: %s %s %s %s",
               WMBC_STATUS_STR(_wmbcIF.getObj().singleError[e_YAW]),
               WMBC_STATUS_STR(_wmbcIF.getObj().singleError[e_HORIZON]),
               WMBC_STATUS_STR(_wmbcIF.getObj().singleError[e_ROLL]),
               WMBC_STATUS_STR(_wmbcIF.getObj().singleError[e_CAM_HEIGHT]));
  }

  void FOEFinder::fillAutofixFailSafe(bool &autoFixFailSafe, bool &autoFixFailSafeYaw, bool &autoFixFailSafeHorizon) {
    autoFixFailSafeYaw = false;
    autoFixFailSafeHorizon = false;
    autoFixFailSafe = false;
  }

  PixelLm2_f FOEFinder::foeAllocLm2(CameraInfo::CameraInstance inst, bool interim) {
      if (!CameraInfo::exists(inst)) {
        WMBC_PRINT(WmbcDbgStg::e_RESERVED1, WmbcDbgClr::e_BLUE, "[foeAllocLm2] inst %d does not exist", (int)inst);
        //return PixelLm2_f(0.f, 0.f);
        PixelLm2_f foeLm2;
        foeLm2.X() = foeLm2.Y() = 0.f;
        return foeLm2;
      }

    int i = (int)inst;
    const VehicleToCam &v2c = (interim) ? _wmbcIF.getObj().interimVehicleToCam[i] : _wmbcIF.getObj().vehicleToCam[i];
      // const VehicleToCam &v2c = _wmbcIF.getObj().vehicleToCam[i];
      PixelLm2_f foeLm2 = v2c.foeLm2 + _data.camera.origin[i].distLm2;
      WMBC_PRINT(WmbcDbgStg::e_RESERVED1, WmbcDbgClr::e_BLUE, "[foeAllocLm2] inst %d: foe = (%.3f, %.3f) + (%.3f, %.3f) = (%.3f, %.3f)",
                 (int)inst, v2c.foeLm2.X(), v2c.foeLm2.Y(), _data.camera.origin[i].distLm2.X(), _data.camera.origin[i].distLm2.Y(),
                 foeLm2.X(), foeLm2.Y());

      bool outOfBounds = (foeLm2.X() < IM_MARGIN || 
                          foeLm2.X() > _data.camera.origin[i].distLimitLm2X - IM_MARGIN ||
                          foeLm2.Y() < IM_MARGIN || 
                          foeLm2.Y() > _data.camera.origin[i].distLimitLm2Y - IM_MARGIN);

      if (outOfBounds) {
        foeLm2 = _data.camera.origin[i].distLm2;
        WMBC_PRINT(WmbcDbgStg::e_RESERVED1, WmbcDbgClr::e_BLUE, "[foeAllocLm2] outofBounds resetting", 0);
      }

      return foeLm2;
  }

  void FOEFinder::updateReset() {
    int &m = _data.algo.resetMask;
    static bool noReset = Debug::Args::instance().existsParameter("-sWmbc-noReset");
    if (noReset) {
      m = e_RESET_SKIP;
      return;
    }
    m = e_RESET_NONE;

    if (_data.validParams.useDynamicSuspension && _data.vehicle.dynamicCameraHeightChangeActive) {
      WMBC_PRINT(WmbcDbgStg::e_STATUS, WmbcDbgClr::e_BLUE, "[updateReset] reset - dynamic suspension", 0);
      m |= e_RESET_SUSPENSION;
      reset();
      return;
    }

    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      if (!_variable[i].histValid()) {
        WMBC_PRINT(WmbcDbgStg::e_STATUS, WmbcDbgClr::e_BLUE, "[updateReset] reset %s - histogram invalid",
                   WMBC_DOF_ID_STR(i));
        m |= (1<<i);
        reset(i);
      }
    }

    bool wmFoeReset = (_data.algo.wmFoeReset == e_WMRESET_EM_STAGGER
                       || _data.algo.wmFoeReset == e_WMRESET_LARGE);
    if (wmFoeReset) {
      WMBC_PRINT(WmbcDbgStg::e_STATUS, WmbcDbgClr::e_BLUE, "[updateReset] reset - wmFoeReset: %d",
                 _data.algo.wmFoeReset);
      m |= e_RESET_WMFOERESET;
      reset();
      return;
    }
  }

  void FOEFinder_SPC::updateReset() {
    FOEFinder::updateReset();
    bool isValidSession = (_data.metaParams.maxAttempts == 0 || _data.algo.sessionNum < _data.metaParams.maxAttempts);
    if (_data.algo.total.progress == 99 && isValidSession) {
      restartSession();
    }
  }

  void FOEFinder::printDebugConv(int convMask) const {
    /*
#if defined (MEwin) && defined (WMBC_DEBUG_PRINT)
    const char *dof_s[4] = {"yaw ", "horizon ", "roll ", "camh "};
    std::cout << "[updateConvergence] conv: (";
    for (int i = 0; i < 4; ++i) {
      std::cout << (_data.algo.single[i].conv ? dof_s[i] : "");
    }
    std::cout << "\b), convMask: " << convMask << "(0b";
    for (int i = 3; i >= 0; --i) {
      std::cout << ((convMask>>i)&1);
    }
    std::cout << "), inputMask: " << _params.calcSwitchMask << "(0b";
    for (int i = 3; i >= 0; --i) {
      std::cout << ((_data.metaParams.calcSwitchMask>>i)&1);
    }
    std::cout << "), totalConv: " << _data.algo.total.conv << std::endl;
#endif
*/
  }

  void FOEFinder::printDebugSingleConv(int calibInd) const {
  /*
#if defined (MEwin) && defined (WMBC_DEBUG_PRINT)
    std::cout << "[isSingleCalibConverged] calibInd: " << calibInd
              //<< ", condMask: " << _conv.condMask[calibInd] << " (0b";
              << ", condMask: " << _variable[calibInd].nonConvReason() << " (0b";
    for (int i = 4; i >= 0; --i) {
      //std::cout << ((_conv.condMask[calibInd]>>i) & 1);
      std::cout << ((_variable[calibInd].nonConvReason()>>i) & 1);
    }
    std::cout << ")\n";
#endif
*/
  }

  void FOEFinder::printDebugOrigins() const {
    /* 
#if defined (MEwin) && defined (WMBC_DEBUG_PRINT)
    for(int i = 0; i < CameraInfo::e_NUM_OF_CAM_INSTANCES; ++i) {
      CameraInfo::CameraInstance inst = CameraInfo::CameraInstance(i);
      if(!CameraInfo::exists(inst)) {
        continue;
      }

      // nominal thru getExposure
      int nominal[2] = {0, 0};
      const Prep::SafeImg12* imgexp = PrepSys_API::getExposure(PrepSys::exp_mask::T0, inst);
      if (imgexp) {
        nominal[0] = ((*imgexp)->effectiveWidth())/2;
        nominal[1] = ((*imgexp)->allocatedHeight())/2;
      }

      // etc thru getPyramid
      int pyramid[2] = {0, 0};
      const Prep::SafeImg* img = &PrepSys_API::getPyramid(PrepSys::exp_mask::T0, PrepSys::pyr_type::ltm, 0, inst)->sync(-2);
      if (img) {
        pyramid[0] = (*img)->origin().x;
        pyramid[1] = (*img)->origin().y;
      }

      // thru cam2cam
      float camcam[2] = {0, 0};
      CameraInfo::imageToBuffer(inst, camcam[0], camcam[1]);

      // fulls etc
      int full[2] = {0, 0};
      int af[2] = {0, 0};      
      if (CameraInfo::isYawFullValid(inst) && CameraInfo::isHorizonFullValid(inst)) {
        full[0] = CameraInfo::yawFull(inst);
        full[1] = CameraInfo::horizonFull(inst);
      }
      if (CameraInfo::autoFix_enable(inst)) {
        af[0] += CameraInfo::autoFix_yaw(inst)*4;
        af[1] += CameraInfo::autoFix_horizon(inst)*4;
      }

      WMBC_PRINT(WmbcDbgClr::e_CYAN, "[setOrigin] inst: %d\n----------------------\n"
                 "delta (pyramid-nominal) = (%d, %d) - (%d, %d) = (%d, %d)\n"
                 "delta (full+af)         = (%d, %d) + (%d, %d) = (%d, %d)\n"
                 "delta (cam2cam-nominal) = (%f, %f) - (%d, %d) = (%f, %f)\n"
                 "----------------------\n", i,
                 pyramid[0], pyramid[1], nominal[0], nominal[1], pyramid[0]-nominal[0], pyramid[1]-nominal[1],
                 full[0], full[1], af[0], af[1], full[0]+af[0], full[1]+af[1],
                 camcam[0], camcam[1], nominal[0], nominal[1], camcam[0]-nominal[0], camcam[1]-nominal[1]);
    }
#endif
*/
  }

  // ------------------------------------------- HistogramOneDim -----------------------------------------------------------

  HistogramOneDim::HistogramOneDim() : _lowerBound(0), _binSize(1), _decFac(DECAY_FACTOR), _startDecay(START_DECAY), _parent(nullptr) {
    _meanBuff.alloc(HIST_MEAN_BUFF_SIZE);
    reset();
  }

  HistogramOneDim::HistogramOneDim(float lowerBound, float binSize) : _lowerBound(lowerBound), 
                                                                      _binSize(binSize),
                                                                      _parent(nullptr)
  {
    _decFac = Debug::Args::instance().getStickyValue("-sWmbcDecFac", DECAY_FACTOR);
    _startDecay = Debug::Args::instance().getStickyValue("-sWmbcStartDecay", START_DECAY);
    _decFac = std::min(_decFac, _startDecay/(_startDecay+1));  // (1-decayFactor) > 1/(_startDecay+1)
    _meanBuff.alloc(HIST_MEAN_BUFF_SIZE);
    reset();
  }

  HistogramOneDim::HistogramOneDim(float lowerBound, float binSize, float decFac, int startDecay) : _lowerBound(lowerBound), 
                                                                                                    _binSize(binSize), 
                                                                                                    _decFac(decFac), 
                                                                                                    _startDecay(startDecay),
                                                                                                    _parent(nullptr)
  {
    _meanBuff.alloc(HIST_MEAN_BUFF_SIZE);
    reset();
  }

  void HistogramOneDim::reset() {
    for (int i = 0; i < HIST_BIN_NUM; ++i) {
      _hist[i] = 0;
    }
    for(int i=0; i < HIST_START_FRAME; ++i) {
      _firstValues[i] = 0;
    }
    _totalCount = 0;
    _lastVal = 0.f;
    _sum = 0.f;
    _sqrSum = 0.f;
    _mean = 0.f;
    _var = 0.f;
    _autocov = 0.f;
    _meanAutocovSize = 0.f;
    _median = 0.f;
    _prevMedian = 0.f;
    _interquartileRange = 0.f;
    _minVal = 0.f;
    _maxVal = 0.f;
    _minBin = 0; // TODO: const invalid
    _maxBin = 0;
    _lastBin = 0;
    _meanDecay = 0.f;
    _varDecay = 0.f;
    _valid = true;
    _meanBuff.clear();
  }

float HistogramOneDim::lastValHist() const {
  if (_totalCount <= HIST_START_FRAME) {
    return _lastVal;
  }
  return _lowerBound+_lastBin*_binSize;
}

void HistogramOneDim::update(float x) {
    // in the first few frames only mean is calculated and is reported also as median.
    // after a few frames the histogram boundaries are set with the mean at the center
    _lastVal = x;

    if (_totalCount == 0) {
      _minVal = _maxVal = _lastVal;
      _mean = _meanDecay = _lastVal;
      _median = _lastVal;
      _var = _varDecay = _autocov = 0.f;
      _sum = _lastVal;
      _sqrSum = _lastVal*_lastVal;
      _firstValues[0] = _lastVal;
      _meanBuff.push_back(_mean);
      _totalCount++;
      return;
    }

    if (_totalCount == HIST_START_FRAME) { // set histogram limits
      _lowerBound = _mean - (HIST_BIN_NUM/2)*_binSize; //according to mean of first HIST_START_FRAME values (not including _lastVal - using _mean from last iter)
      for (int i=0; i<HIST_START_FRAME; ++i) { //updating the histogram with all the first HIST_START_FRAME values 
        int bin = (int)std::floor(std::min(std::max((_firstValues[i] - _lowerBound)/_binSize, 0), HIST_BIN_NUM-1));
        _hist[bin]++;
      }
    }

    calcMoments();
    calcAutocov();

    if (_totalCount < HIST_START_FRAME) { // set median to mean and abort
      _maxVal = (_lastVal > _maxVal) ? _lastVal : _maxVal;
      _minVal = (_lastVal < _minVal) ? _lastVal : _minVal;
      _prevMedian = _median;
      _median = _mean;
      _firstValues[_totalCount] = _lastVal;
      _totalCount++;
      // std::cout<<"_totalCount<HIST_START_FRAME, inside if block, _median="<<_median<<std::endl;      
      return;
    }

    updateHistogram();
    calcMedian();
    _totalCount++;
    validate();
  }

  
  void HistogramOneDim::updateHistogram() {
    _lastBin = (int)std::floor(std::min(std::max((_lastVal - _lowerBound)/_binSize, 0), HIST_BIN_NUM-1));
    _hist[_lastBin]++;

    float val = lastValHist();
    if(val > _maxVal) {
      _maxVal = val;
      _maxBin = _lastBin;
    }

    if(val < _minVal) {
      _minVal = val;
      _minBin = _lastBin;
    }
  }

  void HistogramOneDim::calcMoments() {
    _sum += _lastVal;
    _sqrSum += _lastVal*_lastVal;

    float invTotalCount = 1.f / (_totalCount + 1);
    _mean = _sum * invTotalCount;
    _var = _sqrSum * invTotalCount - _mean * _mean;

    if (_totalCount > _startDecay) {
      float prevMeanDecay = _meanDecay;
      _meanDecay = (1 - _decFac)*_lastVal + _decFac*prevMeanDecay;
      _varDecay = (1 - _decFac)*_lastVal*_lastVal + _decFac*(_varDecay + prevMeanDecay*prevMeanDecay) - _meanDecay*_meanDecay;
    } else {
      _meanDecay = _mean;
      _varDecay = _var;
    }
  }

  void HistogramOneDim::calcAutocov() {
    // /homes/urilo/Documents/math/recursiveMoments/recursiveMoments.pdf
    int n = _totalCount;
    int a = HIST_MEAN_BUFF_SIZE - 1; // autocov win size

    if (n == a-1) {
      _meanAutocovSize = _mean; // mean at autocov-win-size minus one
    }

    if (n < a) {
      _autocov = 0.f;
      _meanBuff.push_back(_mean);
      return;
    }

    float Q0 = n*(n+1-a)*_meanBuff[1-a]; // <x>_{n-a} one before the end of buffer
    float Q1 = 0.f;
    if (n > a) { // buffer is full
      Q1 = (n+1)*(n-a)*_meanBuff[-a]; // <x>_{n-1-a} at the end of buffer
    }
    float Q2 = a*(_mean + _meanBuff[0] - _meanAutocovSize);

    _autocov = n*_autocov/(n+1);
    _autocov += (_mean - _meanBuff[0])*(Q0 - Q1 - Q2)/(n+1);

    _meanBuff.push_back(_mean);
  }

  void HistogramOneDim::calcMedian() {
    int valuesNum = _totalCount + 1;
    int tmpSum = 0; //, lastTmpSum = 0;
    int quartCount = valuesNum/4;
    int halfCount = valuesNum/2; //2*quartCount;
    int threeQuartCount = 3*valuesNum/4; //3*quartCount;
    int iQuart = 0, iThirdQuart = 0;
    float iHalf = 0.f; //, iHalfPlus = 0;
    bool bHalf = false;
    for (int i = 0; i < HIST_BIN_NUM; ++i) {
      tmpSum += _hist[i];
      // quartiles
      if (tmpSum > quartCount && iQuart == 0) {
        iQuart = i;
      }
      if (tmpSum > threeQuartCount && iThirdQuart == 0) {
        iThirdQuart = i;
      }

      // median
      if (tmpSum > halfCount and !bHalf) {
        if (valuesNum%2 != 0 || iHalf < EPS) { // TODO: create macro is_zero for floats and positive floats
          iHalf = i;
        } else {
          iHalf = 0.5 * (i + iHalf);
        }
        bHalf = true;
      }
      if (tmpSum == halfCount && iHalf < EPS) {
        iHalf = i;
      }
    }

    _prevMedian = _median;
    _median = _lowerBound + iHalf *_binSize;
    _interquartileRange = (iThirdQuart - iQuart) * _binSize;
  }


  void HistogramOneDim::validate() {
    // avoid int overflow
    // TODO: change condition to maximal bin (not max value) > HIST_MAX_SIZE ?
    // todo: chekc totalCount, max/min, maxCount
    _valid = (_totalCount <= HIST_MAX_SIZE); 
    if (!_valid) {
      return;
    }

    // if the edge of the histogram is being filled it's a sign that the histogram boundaries are not set correctly
    // the histogram is being invalidated in this case.
    int edgeCount = _hist[0] + _hist[HIST_BIN_NUM-1];
    int threshold = std::max(3, _totalCount/100);
    _valid = (edgeCount < threshold);

    WMBC_PRINT(WmbcDbgStg::e_HISTOGRAM, WmbcDbgClr::e_BROWN, "[validate@HistogramOneDim] edgeCount = %d + %d = %d, _totalCount = %d, threshold: %d, status: %svalid",
              _hist[0], _hist[HIST_BIN_NUM-1], edgeCount, _totalCount, threshold, (_valid ? "" : "in"));
  }

  void HistogramOneDim::printHist(std::string msg) {
    /*
#if defined (MEwin) && defined (WMBC_DEBUG_PRINT)
    std::cout << "<" << *PrepSys_API::getGrabIndex(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD) << "> [printHist] " << msg << ", last val: " << _lastVal 
              << " (" << _lowerBound + _lastBin*_binSize << "), in bin: " << _lastBin 
              << ", median: " << _median << ", iqr: " << _interquartileRange << std::endl;
    std::cout << "<" << *PrepSys_API::getGrabIndex(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD) << "> [printHist] " << msg << ", Bins: [";
    for (int i = 0; i < HIST_BIN_NUM; ++i) {
      if (_hist[i] == 0) {
        continue;
      }
      for (int j = 0; j < _hist[i]; ++j) {
        std::cout << _lowerBound + i*_binSize << ", ";
      }
    }
    std::cout << "\b\b]" << std::endl;
#endif
*/
  }

  void HistogramOneDim::printMeanBuff() {
#ifdef MEwin
    if ((_debugPrintMask & WmbcDbgStg::e_HISTOGRAM) != WmbcDbgStg::e_HISTOGRAM) {
      return;
    }
    fprintf(stderr, "[calcAutocov@HistogramOneDim] %s: size=%d, buff: ",
            WMBC_DOF_ID_STR(_parent->id()), (int)_meanBuff.size());
    for (unsigned int i=0; i<_meanBuff.size(); ++i) {
      fprintf(stderr, "%.3f ", _meanBuff[-i]);
    }
    fprintf(stderr, "\n");
#endif
  }
  
  // ------------------------------------------- WorldModelData -----------------------------------------------------------
/*
  WorldModelData::WorldModelData() {
    focalLm2 = INVALID_VAL;
    invFocalLm2 = INVALID_VAL;
    for (int i = 0; i < 3; ++i) {
      t[i] = INVALID_VAL;
      ypr[i] = INVALID_VAL;
    }
    N[0] = N[2] = 0.f;
    N[1] = 1.f;
    foeRectLm2[0] = foeRectLm2[1] = INVALID_VAL;
    straightScore = 0.f;
    emValid = false;
    emConf = 0.f;
    emStatus = EM_OFF;
    emValidFrames = 0;
    d = INVALID_VAL;
    roll = INVALID_VAL;
    rmPitch = INVALID_VAL;
    rmValid = false;
    rmStatus = RM_MODEL_INVALID;
    rmNumOfInliers = 0;
    rmPlaneIdx = 0;
    rmMessage = WorldModel::VRM::INVALID_PLANE;
    crownZ = Debug::Args::instance().getStickyValue("-sWmbc-crownDist",  5.f);
    for (int i = 0; i < CROWN_SAMP_NUM; ++i) {
      crownX[i] = 0.f;
      crownAng[i] = 0.f;
      crownAngStd[i] = 0.f;
    }
    pitchDiff = 0.f;
  }

  // bool WorldModelData::isStraightDrive(const double rotTh[]) const {
  bool WorldModelData::isStraightDrive(ValidationParams *params) {
    bool isStraightDrive = emValid;
    for (int i = 0; i < 3; ++i) {
      bool yprOk = (me_abs(ypr[i]) < params->rotThreshold[i]); // || (ypr[i] == INVALID_VAL);
      isStraightDrive = isStraightDrive && yprOk;
      if (params->hystEnabled && emValidFrames > 1) {
        if (yprOk) {
          params->rotThreshold[i] = std::min(params->hystRotThMax[i], params->rotThreshold[i] + params->hystRotThInc);
        } else {
          params->rotThreshold[i] = std::max(params->hystRotThMin, params->rotThreshold[i] - params->hystRotThInc);
        }
      }
    }
    return isStraightDrive;
  }
*/
  // ------------------------------------------- VehicleToCam -----------------------------------------------------------
  void VehicleToCam::reset() {
   R = Float::MEmath::identity<3, double>();
   for (int i = 0; i<3; ++i) {
     angles[i] = 0.0;
   }
   camh = 0.0;
   //foeLm2.set(0.f, 0.f); // TODO: this creates a discrepancy between angles and foeLm2 - decide which to reset and change other accordingly
   foeLm2.X() = 0.f;
   foeLm2.Y() = 0.f;
  }

  void VehicleToCam::update(const double anglesIn[3]) {
    for (int i = 0; i<3; ++i) {
      angles[i] = anglesIn[i];
    }
    // R = WorldModel::composeRotationMatrixFromLogAngles(yaw, pitch, -roll); // TODO: what is the correct form of ypr2R?
    ypr2R(angles[0], angles[1], -angles[2], R);

    // Float::MEmath::Mat<3,3,float> Rroll = WorldModel::composeRotationMatrixFromEulerAngles(0.0, 0.0, -roll);
    // Float::MEmath::Mat<3,3,float> Rfoe =  WorldModel::composeRotationMatrixFromLogAngles(yaw, pitch, 0.0);
    // R = Rroll * Rfoe;

    float yawRectLm2 = angles[0] * focalLm2;
    float horRectLm2 = angles[1] * focalLm2;
    // TODO: add member inst and use accordingly
    bool ok = DistortionCorrectionAPI::isDistortionValid(CameraInfo::e_FORWARD);
    if (ok) {
      ok = DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, yawRectLm2, horRectLm2, foeLm2.X(), foeLm2.Y());
    }
  }

  void VehicleToCam::update(double valIn, int idx) {
    if (idx == e_CAM_HEIGHT) {
      camh = valIn;
    } else {
      angles[idx] = valIn;
      update(angles);
    }
  }

  void VehicleToCam::update(const double anglesIn[3], double camhIn) {
    update(anglesIn); 
    camh = camhIn;
  }

  void VehicleToCam::update(const Float::MEmath::Mat<3,3,double>& Rin, double camhIn) {
    R = Rin;

    R2ypr(R, angles[0], angles[1], angles[2]);
    // WorldModel::decomposeRotationMatrixIntoLogAngles(R, yawAngle, pitchAngle, rollAngle);
    angles[2] = -angles[2]; 
    camh = camhIn;
  }
    
  void VehicleToCam::update(const PixelLm2_f foeLm2In, const double rollIn, const double camhIn) {

    foeLm2 = foeLm2In;
    angles[2] = rollIn;
    camh = camhIn;

    float yawRectLm2 = foeLm2.X();
    float horRectLm2 = foeLm2.Y();
    // TODO: add member inst and use accordingly
    bool ok = DistortionCorrectionAPI::isDistortionValid(CameraInfo::e_FORWARD);
    if (ok) {
      //ok = DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, yawRectLm2, horRectLm2, yawDistLm2, horDistLm2);
      ok = DistortionCorrectionAPI::rectifySafe(CameraInfo::e_FORWARD, -2, foeLm2.X(), foeLm2.Y(), yawRectLm2, horRectLm2);
    }
    angles[0] = (double)(yawRectLm2/focalLm2);
    angles[1] = (double)(horRectLm2/focalLm2);
    ypr2R(angles[0], angles[1], -angles[2], R);
  }

  void VehicleToCam::rotate() {
    float cosr = me_cosf(-angles[e_ROLL]);
    float sinr = me_sinf(-angles[e_ROLL]);
    float y = cosr * angles[e_YAW] - sinr * angles[e_HORIZON];
    float h = sinr * angles[e_YAW] + cosr * angles[e_HORIZON];
    angles[e_YAW] = y;
    angles[e_HORIZON] = h;
    update(angles);
  }

  VehicleToCam::VehicleToCam(const VehicleToCam& v) : focalLm2(v.focalLm2), R(v.R), camh(v.camh), foeLm2(v.foeLm2) {
    for (int i = 0; i < 3; ++i) {
      angles[i] = v.angles[i];
    }
  }


  VehicleToCam& VehicleToCam::operator=(const VehicleToCam& rhs) {
    focalLm2 = rhs.focalLm2;
    R = rhs.R;
    for (int i = 0; i < 3; ++i) {
      angles[i] = rhs.angles[i];
    }
    camh = rhs.camh;
    foeLm2 = rhs.foeLm2;
    return *this;
  }

  VehicleToCam& VehicleToCam::operator+=(const VehicleToCam& that) {
    // TODO: do matrix mult and then decompose to angles (how camh cahnges?)
    for (int i=0; i<3; ++i) {
      angles[i] += that.angles[i];
    }
    camh += that.camh;
    update(angles);
    return *this; 
  }

  VehicleToCam& VehicleToCam::operator-=(const VehicleToCam& that) {
    // TODO: do matrix mult and then decompose to angles (how camh cahnges?)
    for (int i=0; i<3; ++i) {
      angles[i] -= that.angles[i];
    }
    camh -= that.camh;
    update(angles);
    return *this; 
  }
  
  VehicleToCam operator+(const VehicleToCam& v0, const VehicleToCam& v1) {
    VehicleToCam v = v0;
    v += v1;
    return v;
  }

  VehicleToCam operator-(const VehicleToCam& v0, const VehicleToCam& v1) {
    VehicleToCam v = v0;
    v -= v1;
    return v;
  } 

// ---------------- result book ----
  void ResultBook::focalLm2(float focalLm2) {
    curr.focalLm2 = focalLm2;
    prevConv.focalLm2 = focalLm2;
    prevWmReset.focalLm2 = focalLm2;
    for (int i = 0; i < CameraInfo::e_NUM_OF_CAM_INSTANCES; ++i) {
      // TODO: fix this!
      cams[i].focalLm2 = focalLm2;
    }
  }
  // ------------------------------------------- WmbcFoe -----------------------------------------------------------
/*
  void WmbcFoe::reset(float invFocalLm2, const OriginData *originData) {
    yawAnglePrimary = 0.f;
    pitchAnglePrimary = 0.f;

    if (DistortionCorrectionAPI::isDistortionValid()) {
      DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, 0, 0, yawAnglePrimary, pitchAnglePrimary);
      yawAnglePrimary *= invFocalLm2;
      pitchAnglePrimary *= invFocalLm2;

      yawLm2Primary_f = originData->distDeltaLm2_x;
      horizonLm2Primary_f = originData->distDeltaLm2_y;
    }

    yawDeltaLm2FromEtcPrimary_f = 0.f;
    horizonDeltaLm2FromEtcPrimary_f = 0.f;
        
    yawDeltaAngleFromLastConvPrimary = 0.f;
    pitchDeltaAngleFromLastConvPrimary = 0.f;

    // prevRollPrimary = 0.f;
    rollPrimary = 0.f;
    rollDeltaFromLastConvPrimay = 0.f;
    
    camhPrimary = 0.f;
  }
*/


// ------------------------------------------- PlaneSync ------------------------------------------------------------

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
    }

    bool PlaneSync::run(const WmbcData *data) {
      _data = data;
      if (_data->em.valid) {
        updatePlanes(); //update all the planes that are already in the buffer: move them from the last frame to the current frame using R and t from Ego Motion
      } else {
        _planes.clear(); //em is not valid so we can't keep moving the planes forward. we have to start over.
      }
      addPlaneToBuffer(); //add the road model data from the current frame to the plane buffer

      bool validPlane=calcUnderlyingPlane(); //calculate the underlying plane for the current frame
            
      WmbcOutputIF::instance().toItrkPlaneBuffer(&_planes);
      return validPlane;
    }

    void PlaneSync::updatePlanes() {
      Float::MEmath::Mat<3,3,float> R = _data->em.R;
      Float::MEmath::Vec<3,float> t = _data->em.t;
      for (MEtypes::FastCyclicVector<Plane>::iterator iter = _planes.begin(); iter!=_planes.end(); iter++) {
        iter->N = R*(iter->N);
        iter->d = iter->d+iter->N*(R.transpose()*t);
      }

    }

    void PlaneSync::addPlaneToBuffer() {
      if (_data->rm.valid) {
        Plane plane;
        plane.N = _data->rm.N;
        plane.d = -_data->rm.d;
        plane.location = _data->vehicle.trajLength + _data->rm.Zstart + futurePlaneFac*(_data->rm.Zend-_data->rm.Zstart)/2;
        plane.startFrame = globalFrameIndex;
        plane.endFrame = globalFrameIndex;
        plane.confData.fill(_data->em, _data->rm);

        _planes.push_back(plane);
      }
    }

    bool PlaneSync::calcUnderlyingPlane() {
      bool validPlane=false;
      int startFrame = 0;
      int endFrame = 0;
      float dSum=0;
      Float::MEmath::Vec<3,float> Nsum;
      for (int i=0; i<3; ++i) {
        Nsum[i]=0;
      }
      int count=0;
      float totalDistance = _data->vehicle.trajLength;

      _underlyingPlane.confData.reset();

      for (MEtypes::FastCyclicVector<Plane>::iterator iter = _planes.begin(); iter!=_planes.end(); iter++) {
        Plane currPlane = *iter;

        if (me_fabs(currPlane.location-totalDistance)<maxDistFromLocation) {
          endFrame = currPlane.endFrame;
          if (count == 0) {
            startFrame = currPlane.startFrame;
          }
          _underlyingPlane.confData += currPlane.confData;

          dSum+=currPlane.d;
          Nsum+=currPlane.N;
          count++;
        }

      }
      if (count>0) {
        _underlyingPlane.d          = dSum/count;
        _underlyingPlane.N          = Nsum/count;
        _underlyingPlane.location   = totalDistance;
        _underlyingPlane.startFrame = startFrame;
        _underlyingPlane.endFrame   = endFrame;
        _underlyingPlane.confData  /= count;
        _underlyingPlane.confData.pitchDiff = me_fabs(_underlyingPlane.pitch() - _data->em.pitch);

        validPlane=_underlyingPlane.valid();
      }

      return validPlane;

    }

} // namespace WMBC
