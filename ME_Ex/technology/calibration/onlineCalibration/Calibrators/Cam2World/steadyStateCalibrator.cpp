/**
 * \file SteadyStateCalibrator.h
 * \brief SteadyStateCalibrator implementation
 *
 * \author Uri London
 * \date Jul 11, 2019
 */

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/steadyStateCalibrator.h"
#include "basicTypes/transformationMatrices_API.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wOutputIF.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wUtils.h"
#include "basicTypes/transformationMatrices_API.h"
#include "technology/worldModel/egoMotion/Repository/EmData.h"

namespace OnlineCalibration {
  namespace Cam2World {

    SteadyStateCalibrator::SteadyStateCalibrator() {
      setParams();
      int confType = Debug::Args::instance().getStickyValue("-sOCC2W-confType", 4); //3);
      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        _signal[i].id((C2WSig)i);
        _signal[i].binSize(HIST_BIN_SIZE[i]);
        _signal[i].invalidFrameMask(INVALID_MASK[i]);
        _signal[i].confType(confType);
      }
      reset();
    }

    void SteadyStateCalibrator::reset() {
      _data.results.confidence = 0.f;
      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        _signal[i].reset();
      }
    }

    void SteadyStateCalibrator::setParams() {
      ValidParams &vp = _data.validParams;
      // MetaParams &mp = _data.metaParams;
      SpcData &sd = _data.spc;

      // parameters controlling frame validation
      vp.minSpeed                   = 5.f;  // [m/sec]
      vp.maxSpeed                   = 70.f; // [m/sec]
      vp.maxAccel                   = 0.3f; // [m/sec^2]
      vp.maxYawRate                 = 1.5f*DEG2RAD; // [rad]
      vp.minRadius                  = 200; // [m]
      for (int i=0; i<3; ++i) {
        vp.maxEmRot[i]              = 0.004; // todo: consolidate to one, units?
      }
      vp.maxPitchDiff               = 0.6f*DEG2RAD; // [rad]
      vp.maxCrownAngDiff            = -1.f;

      // hysteresis mechanism: the threshold of minRadius can vary
      // between  hystMinRadiusRange[0] to hystMinRadiusRange[1]
      // in steps of hystMinRadiusRange[2]
      // the threshold is increased whenever this condition is invalid
      vp.hystEnabled                = true;
      vp.hystMinRadiusRange[0]      = vp.minRadius; // TODO: this is redundant - remove
      vp.hystMinRadiusRange[1]      = 1000; // [m]
      vp.hystMinRadiusRange[2]      = 50; // [m]

      // same as above the threshold for em_straight can vary
      // between hystRotThMin to hystRotThMax[i] in steps
      // of hystRotThInc. the threshold is decreased whenver this condition is invalid
      vp.hystRotThMin               = 0.001f;
      vp.hystRotThInc               = 0.00025f;
      for (int i                    = 0; i < 3; ++i) {
        vp.hystRotThMax[i]          = vp.maxEmRot[i]; // TODO: this is redundant - remove
      }

      // mp.calibRange[e_YAW][0]        = -5*DEG2RAD;
      // mp.calibRange[e_YAW][1]        = -mp.calibRange[e_YAW][0];
      // mp.calibRange[e_PITCH][0]      = -8*DEG2RAD;
      // mp.calibRange[e_PITCH][1]      = 5.2*DEG2RAD;
      // mp.calibRange[e_ROLL][0]       = -3*DEG2RAD;
      // mp.calibRange[e_ROLL][1]       = -mp.calibRange[e_ROLL][0];
      // mp.calibRange[e_CAM_HEIGHT][0] = 1.1;
      // mp.calibRange[e_CAM_HEIGHT][1] = 3.5;

      sd.maxAttempts = 5;   // why not iniitiate with this value?

      
      // parameters for confErr scaling. Ideally to be decided by client, will determine the location on ROC curve
      float confScale[e_C2W_SIG_NUM];
      confScale[e_YAW] = Debug::Args::instance().getStickyValue("-sOCC2W-confScaleY", 5.f);         // deg
      confScale[e_PITCH] = Debug::Args::instance().getStickyValue("-sOCC2W-confScaleP", 5.f);       // deg
      confScale[e_ROLL] = Debug::Args::instance().getStickyValue("-sOCC2W-confScaleR", 0.1f);       // deg
      confScale[e_CAM_HEIGHT] = Debug::Args::instance().getStickyValue("-sOCC2W-confScaleH", 0.01f); // m

      // todo: check that maxMedianChange>= binSize
      float maxMedianChangeDeg[2];
      float maxMedianChangeCh[2];
      maxMedianChangeDeg[0] = Debug::Args::instance().getStickyValue("-sOCC2W-maxMedianChangeDeg0", 0.028f);
      maxMedianChangeDeg[1] = Debug::Args::instance().getStickyValue("-sOCC2W-maxMedianChangeDeg1", 0.01f);
      maxMedianChangeCh[0] = Debug::Args::instance().getStickyValue("-sOCC2W-maxMedianChangeCh0", 0.001f);
      maxMedianChangeCh[1] = Debug::Args::instance().getStickyValue("-sOCC2W-maxMedianChangeCh1", 0.025f);
      // how many stable frames in a row is "actually" stable.
      int stableCountThresh = Debug::Args::instance().getStickyValue("-sOCC2W-stableCountThresh", 3); // 7);
      // setStableCountThresh(stableCountThresh);
      
      float minSteadyThreshDeg[2];
      float minSteadyThreshCh[2];
      minSteadyThreshDeg[0] = minSteadyThreshDeg[1] =
                    Debug::Args::instance().getStickyValue("-sOCC2W-minSteadyThreshDeg", 0.8f);
      minSteadyThreshCh[0] = minSteadyThreshCh[1] =
                    Debug::Args::instance().getStickyValue("-sOCC2W-minSteadyThreshCh", 0.05f);

      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        ConfParams &cp0 = _data.confParams[i][0];
        ConfParams &cp1 = _data.confParams[i][1];
        cp0.confScale = (i<e_CAM_HEIGHT) ? confScale[i]*DEG2RAD : confScale[i];
        cp0.maxMedianChange = (i<e_CAM_HEIGHT) ? maxMedianChangeDeg[0]*DEG2RAD : maxMedianChangeCh[0];
        cp0.minSteadyThresh = (i<e_CAM_HEIGHT) ? minSteadyThreshDeg[0]*DEG2RAD : minSteadyThreshDeg[0];
        cp0.varHmm.xlimits[0] = (i<e_CAM_HEIGHT) ? 0.025*DEG2RAD_SQR : 0.000625;
        cp0.varHmm.xlimits[1] = (i<e_CAM_HEIGHT) ? 0.0625*DEG2RAD_SQR : 0.0049;
        cp0.varHmm.zero = false;
        cp0.varHmm.up = false;
        cp0.stableHmm.xlimits[0] = 0.f;
        cp0.stableHmm.xlimits[1] = 20.f;
        cp0.stableHmm.zero = false;
        cp0.sampleHmm.xlimits[0] = 0.f;
        cp0.sampleHmm.xlimits[1] = 50.f; // todo: verify this is larger than stable limit
        cp0.sampleHmm.zero = false;
        cp0.stableCountThresh = stableCountThresh;
        cp1.confScale = (i<e_CAM_HEIGHT) ? confScale[i]*DEG2RAD : confScale[i];   // can this 2nd value be utilized better? as a stringent conf?
        cp1.maxMedianChange = (i<e_CAM_HEIGHT) ? maxMedianChangeDeg[1]*DEG2RAD : maxMedianChangeCh[1];
        cp1.minSteadyThresh = (i<e_CAM_HEIGHT) ? minSteadyThreshDeg[1]*DEG2RAD : minSteadyThreshDeg[1];
        cp1.varHmm.xlimits[0] = (i<e_CAM_HEIGHT) ? 0.01*DEG2RAD_SQR : 0.000225;
        cp1.varHmm.xlimits[1] = (i<e_CAM_HEIGHT) ? 0.04*DEG2RAD_SQR : 0.0009;
        cp1.varHmm.zero = false;
        cp1.varHmm.up = false;
        cp1.stableHmm.xlimits[0] = 0.f;
        cp1.stableHmm.xlimits[1] = Debug::Args::instance().getStickyValue("-sOCC2W-stMinStable", 50.f);
        cp1.stableHmm.zero = false;
        cp1.sampleHmm.xlimits[0] = 0.f;
        cp1.sampleHmm.xlimits[1] = Debug::Args::instance().getStickyValue("-sOCC2W-stMinSample", 150.f); // todo: verify this is larger than stable limit
        cp1.sampleHmm.zero = false;
        cp1.stableCountThresh = stableCountThresh;
      }
    }

    void SteadyStateCalibrator::update(const Cam2WorldSources& source) {
      if (_data.spc.spcMode && spcEnded()) {
        OC_C2W_PRINT(e_SPC, e_BO_CYAN, "[SSC::update] spc ended/stopped", 0);
        return;
      }
      updateReset();
      setInputData(source);
      propagatePlaneUnderCam();
      validateFrameVehicle();
      validateFrameAlgo();
      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        _signal[i].run(&_data);
      }
      if (_data.spc.spcMode) {
        spcUpdateStatus();
        OC_C2W_PRINT(e_SPC, e_BO_CYAN, "[SSC::update] spcStatus=%s, spcProg=%d, spcSFailed=%d, spcConv=%d", 
                     s_CStatus[_data.spc.status].c_str(), _data.spc.progress, _data.spc.sessionFailed,
                     _data.spc.conv);
      }
    }

    void SteadyStateCalibrator::updateReset() {
      int &m = _data.algo.resetMask;
      static bool noReset = Debug::Args::instance().existsParameter("-sOCC2W-noReset");
      if (noReset) {
        m = e_RESET_SKIP;
        return;
      }
      m = e_RESET_NONE;

      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        if (!_signal[i].histValid()) {
          OC_C2W_PRINT(e_STATE, e_BLUE, "[updateReset] reset %s - histogram invalid",
                     s_C2WSig[i].c_str());
          m |= (1<<i);
          _signal[i].reset();
        }
      }

      if (_data.spc.spcMode) {
        spcUpdateReset();
      }
    }

    void SteadyStateCalibrator::setInputData(const Cam2WorldSources& source) {
      setExtraParams(source); // TODO: can be merged into setParams after source init
      // setCameraData(source);
      setVehicleData(source);
      setWmEgomotionData(source);
      setWmRoadModelData(source);
      setWmConfData(source);
      Cam2WorldOutputIF::instance().toItrkInputData(&_data);
    }

    void SteadyStateCalibrator::setExtraParams(const Cam2WorldSources& source) {
      // TODO: ask sarap to add a prepareSource() func in init and get rid of this ugly workaround
      if (globalFrameIndex>0) {
        return;
      }

      const CameraRawData &s = source.cam;
      const ParamsRaw &p = source.params;
      MetaParams &mp = _data.metaParams;
      SpcData &sd = _data.spc;
      ValidParams &vp = _data.validParams;


      CalibUtils::PixelLm2_f minFoe(0, 0);
      CalibUtils::PixelLm2_f maxFoe(0, 0);
      CalibUtils::PixelLm2 minFoe_pp(0, 0);
      CalibUtils::PixelLm2 maxFoe_pp(0, 0);
      foeBoundariesFromImageCenter2PP(s.minFoe, s.maxFoe, minFoe_pp, maxFoe_pp);
      minFoe[0] = minFoe_pp[e_YAW];
      minFoe[1] = minFoe_pp[e_PITCH];
      maxFoe[0] = maxFoe_pp[e_YAW];
      maxFoe[1] = maxFoe_pp[e_PITCH];
      // old and obselete: 
      // minFoe[0] = s.minFoe[e_YAW];
      // minFoe[1] = s.minFoe[e_PITCH];
      // maxFoe[0] = s.maxFoe[e_YAW];
      // maxFoe[1] = s.maxFoe[e_PITCH];
      // CalibUtils::PixelLm2_f minFoe(s.minFoe[e_YAW], s.minFoe[e_PITCH]);
      // CalibUtils::PixelLm2_f maxFoe(s.maxFoe[e_YAW], s.maxFoe[e_PITCH]);

      pixelToAngles(minFoe, mp.calibRange[e_YAW][0], mp.calibRange[e_PITCH][0]);
      pixelToAngles(maxFoe, mp.calibRange[e_YAW][1], mp.calibRange[e_PITCH][1]);

      mp.calibRange[e_ROLL][0] = -s.maxRoll;
      mp.calibRange[e_ROLL][1] = s.maxRoll;

      //spc and slow spc - note that we require for slow mode
      //that both flags will be set (maybe we should forbid/assert setting the slow spc without setting the spc)
      //see also jiras: CPD-918, APTIVBMWMD-4400
      sd.spcMode = p.spcMode;
      sd.slowMode = (p.spcMode && p.slowMode);
      if (sd.slowMode) {
        vp.minSpeed = 1.f;  // [m/sec]
      }

      OC_C2W_PRINT(e_SPC, e_GREEN, "[setCameraData] spcMode: %d, slowMode: %d", sd.spcMode, sd.slowMode);
    }

    void SteadyStateCalibrator::setCameraData(const Cam2WorldSources& source) {
      CameraData &d = _data.cam;
      const CameraRawData &s = source.cam;

      d.focalLm2 = s.focalLm2;
      d.invFocalLm2 = 1.f/d.focalLm2;
      d.origin = s.foeFull + s.foeAfix;


      OC_C2W_PRINT(e_INPUT, e_GREEN, "[setCameraData] f=%.2f, yawLm2=%d+%d=%d, horLm2=%d+%d=%d",
                   d.focalLm2, s.foeFull[0], s.foeAfix[0], d.origin[0],
                   s.foeFull[1], s.foeAfix[1], d.origin[1]);
    }

    void SteadyStateCalibrator::setVehicleData(const Cam2WorldSources& source) {
      VehicleData &d = _data.vehicle;
      const VehicleRawData &s = source.vh;

      float invFocalLm2 = _data.cam.invFocalLm2;
      float dyaw = 0.f;
      float ds = 0.f;

      if (s.isYawRateAvailable) {
        float yawRateInPixels = s.yawRateInPixels;
        dyaw = -yawRateInPixels * invFocalLm2 * L0_TO_LM2_SCALE;
        if (s.dt > EPS) {
          d.yawRate =  dyaw/s.dt;
        }
      }

      if (s.speedAvailable) {
        ds = s.speed * s.dt;
        float dspeed = s.speed - d.speed;
        if (d.dt > EPS) {
          d.accel = dspeed/s.dt;
        }
        if (s.isYawRateAvailable && me_fabs(dyaw) > EPS) {
          // if yawRate is so small put radius above threshold
          d.radius = (me_fabs(dyaw) > EPS) ? me_fabs(ds/dyaw) : 2*_data.validParams.minRadius;
        }
        d.speed = s.speed;
      }

      d.isYawRateAvailable = s.isYawRateAvailable;
      d.speedAvailable = s.speedAvailable;
      d.dt = s.dt;
      d.ds = ds;
      d.totalDistance += ds;
      d.totalTime += s.dt;

      OC_C2W_PRINT(e_INPUT, e_GREEN, "[setVehicleData] yawRateAvailable=%d, speedAvailable=%d, "
                   "yawRate=%.2fdeg/s, speed=%.2fm/s, radius=%.2fm, accel=%.2fm/s^2, "
                   "dt=%.2fs, ds=%.2fm, totalDistance=%.2fm, totalTime=%.2fs",
                   d.isYawRateAvailable, d.speedAvailable, d.yawRate*RAD2DEG, d.speed,
                   d.radius, d.accel, d.dt, d.ds, d.totalDistance, d.totalTime);
    }

    void SteadyStateCalibrator::setWmEgomotionData(const Cam2WorldSources& source) {
      EmData &d = _data.em;
      const EmRawData &s = source.em;

      d.valid = s.valid;

      if (!d.valid) {
        OC_C2W_PRINT(e_INPUT, e_RED, "[setWmEgomotionData] EM: off", 0);
        d.status = EM_INVALID;
        return;
      }

      d.confVec = s.emVisionMeasurementData->confidenceVec();
      d.confT = s.emVisionMeasurementData->confT();
      d.confR = s.emVisionMeasurementData->confR();

      Float::MEmath::Mat<4,4,double> emVisionMeasurement;
      Float::MEmath::Mat<3,3,double> R;
      Float::MEmath::Vec<3,double> t;

      Float::MEmath::Mat<4,4,double>::toMat(s.emVisionMeasurementData->transformation(), emVisionMeasurement);
      WorldModel::decomposeTransformationMatrixIntoRt(emVisionMeasurement, R, t);
      d.R=CalibUtils::matd2f(R);
      d.t=CalibUtils::vecd2f(t);

      double dyaw, dpitch, droll;
      double tx, ty, tz;
      WorldModel::decomposeTransformationMatrixIntoEulerAngles(emVisionMeasurement, dyaw, dpitch, droll, tx, ty, tz);

      // use log(EM) to get tangent to drive
      static bool useLogEgo = true; //Debug::Args::instance().existsParameter("-sWmbcUseLogEgo");
      if (useLogEgo) {
        double dummy;
        Float::MEmath::Mat<4,4,double> logMeasurement;
        log4(emVisionMeasurement, logMeasurement);
        WorldModel::decomposeTransformationMatrixIntoEulerAngles(logMeasurement, dummy, dummy, dummy, tx, ty, tz);
      }

      d.rot[0] = dyaw;
      d.rot[1] = dpitch;
      d.rot[2] = droll;

      d.valid = (-tz > EPS);
      if (d.valid) {
        d.yaw = me_atanf(tx/tz);
        d.pitch = me_atanf(ty/tz);
        d.status = EM_OK;
        d.validFrameNum++;
      } else {
        d.status = EM_STANDING;
      }

      OC_C2W_PRINT(e_INPUT, e_GREEN, "[setWmEgomotionData] EM: %s, yaw=%.2frad (%.4fdeg), pitch=%.2frad (%.4fdeg)",
                   (d.status == EM_OK ? "OK" : "STANDING"), d.yaw, d.yaw*RAD2DEG, d.pitch, d.pitch*RAD2DEG);
    }

    void SteadyStateCalibrator::setWmRoadModelData(const Cam2WorldSources& source) {
      RmData &d = _data.rm;
      const RmRawData &s = source.rm;

      d.valid = s.valid;
      if (!d.valid) {
        d.status = RM_MODEL_NOT_FOUND;
        OC_C2W_PRINT(e_INPUT, e_RED, "[setWmRoadModelData] roadmodel not found", 0);
        return;
      }

      // validSevere how strict to be on data validation.
      // false: liberal approach: valid if model.d < 0
      // true : more stringent validation - only if model say it is valid
      static bool validSevere = Debug::Args::instance().existsParameter("-sOCC2W-rmValidSevere"); 
      d.N = Float::MEmath::Vec<3, float>::toVec(s.model->N);

      // dbg extreme roll
      // float tempRoll = me_atanf (-d.N[0]/(d.N[1]+1e-6));
      // std::cout << "current frame: " << globalFrameIndex << std::endl;
      // std::cout << "input roll: " << tempRoll << " \n" << std::endl;


      float dist = s.model->d;
      d.numOfInliers = s.model->numOfInliers;
      d.Zstart = s.model->Zstart;
      d.Zend = s.model->Zend;
      d.planeIdx = s.model->segmentIndex;
      d.planeValidity = s.model->isValid;
      d.message = s.model->type;
      d.valid = validSevere ? (d.message == WorldModel::VRM::VALID_PLANE) : (dist < 0);

      CalibUtils::mat33f::toMat(s.model->AtA, d.AtA);
      CalibUtils::mat33f::toMat(s.model->normalizedAtA, d.nAtA);

      OC_C2W_PRINT(e_INPUT, e_YELLOW, "[setWmRoadModelData] validType: %s, N=(%.2f, %.2f, %.2f), d=%.2f, #inliers: %d, planeIdx: %d, Message: %s",
                   (validSevere ? "Severe" : "Relaxed"), d.N[0], d.N[1], d.N[2], dist, d.numOfInliers, d.planeIdx, s_planeType[d.message].c_str());

      Cam2WorldOutputIF::instance().tostdoutMatf(e_INPUT, e_YELLOW, "[setWmRoadModelData] AtA:", d.AtA);
      Cam2WorldOutputIF::instance().tostdoutMatf(e_INPUT, e_YELLOW, "[setWmRoadModelData] nAtA:", d.nAtA);

      Cam2WorldOutputIF::instance().tostdoutMatf(e_INPUT, e_YELLOW, "[setWmRoadModelData] AtA:", d.AtA);
      Cam2WorldOutputIF::instance().tostdoutMatf(e_INPUT, e_YELLOW, "[setWmRoadModelData] nAtA:", d.nAtA);

      if (!d.valid) {
        d.status = RM_CLOSE_PLANE_INVALID;
        OC_C2W_PRINT(e_INPUT, e_RED, "[setWmRoadModelData] close road plane invalid", 0);
        return;
      }

      d.valid = (d.N[1] > EPS);
      if (!d.valid) {
        d.status = RM_PLANE_OFFSIDE;
        OC_C2W_PRINT(e_INPUT, e_RED, "[setWmRoadModelData] Ny = 0", 0);
        return;
      }

      d.camh = -dist;
      d.roll = me_atanf(-d.N[0]/d.N[1]);
      d.pitch = me_atanf(-d.N[2]/d.N[1]);
      d.status = RM_OK;
      d.valid = true;

      d.pitchDiff = me_fabs(d.pitch - _data.em.pitch);

      OC_C2W_PRINT(e_INPUT, e_GREEN, "[setWmRoadModelData] camh=%.2f, "
                 "roll = %.4f [rad] (%.2f [deg]), pitch = %.4f [rad] (%.2f [deg]), #inliers=%d",
                 d.camh, d.roll, d.roll*RAD2DEG, d.pitch, d.pitch*RAD2DEG, d.numOfInliers);
    }

    void SteadyStateCalibrator::propagatePlaneUnderCam(){
      PlaneData &d = _data.plane;
      d.valid =_planeSync.run(&_data);
      if (d.valid) {
        float roll;
        float camh;
        float location;
        _planeSync.calcRollAndCamH(_data.vehicle.totalDistance,roll,camh,location);
        d.roll=roll;
        d.camh=camh;
        d.location=location;
      }
      Cam2WorldOutputIF::instance().toItrkPlane(&_data);
    }


    void SteadyStateCalibrator::validateFrameVehicle() {
      VehicleData &vd = _data.vehicle;
      ValidParams &vp = _data.validParams;
      int &m = _data.algo.pauseReason;
      m = 0;

      // enforce taking the first measurement as valid and bypassing the regular validation mechanism.
      // purpose: in case the init calibration is way off egomotion will work badly. In that case,
      // we'll take anything from it we can work with and feed it back to worldmodel (wmFoeReset).
      // Since the feedback mechanism is not implemented yet, this is disabled by default.
      _data.algo.enforcingFirstValid = false;
      // static bool enforceFirstValid = !Debug::Args::instance().existsParameter("-sOCC2W-dontEnforceFirstValid");
      static bool enforceFirstValid = Debug::Args::instance().existsParameter("-sOCC2W-enforceFirstValid");
      bool noDataYet = (_signal[e_YAW].validFrameNum() * _signal[e_PITCH].validFrameNum() == 0);

      if (enforceFirstValid && noDataYet && _data.em.valid) {
        _data.algo.enforcingFirstValid = true; // this is transmitted to the Signal class
        OC_C2W_PRINT(e_VALID, e_YELLOW, "[validateFrameVehicle] enforcing 1st emValid frame", 0);
      }


      // validate vehicle frame
      if (!vd.speedAvailable || !vd.isYawRateAvailable) {
        m |= e_SENSORS_UNAVAILABLE;
      }
      if (vd.speedAvailable && vd.speed < vp.minSpeed) {
        m |= e_SPEED_TOO_LOW;
      }
      if (vd.speedAvailable && vd.speed > vp.maxSpeed) {
        m |= e_SPEED_TOO_HIGH;
      }
      if (vd.radius < vp.minRadius) {
        m |= e_RADIUS_TOO_SMALL;
      }
      if (me_fabs(vd.accel) > vp.maxAccel) {
        m |= e_ACCELERATION_TOO_HIGH;
      }
      if (vd.isYawRateAvailable && me_fabs(vd.yawRate) > vp.maxYawRate) {
        m |= e_YAWRATE_TOO_HIGH;
      }

      // hysteresis mechanism: when radius is invalidated the threshold is raised for the
      // next frame (up to vp.hystMinRadiusRange[1]) otherwise it gradually returns
      // to vp.hystMinRadiusRange[0]
      // the purpose is to not validate too soon in curved roads.
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

    void SteadyStateCalibrator::validateFrameAlgo() {
      // ValidParams &vp = _data.validParams;
      int &m = _data.algo.pauseReason;

      if (!_data.em.valid) {
        m |= e_EM_INVALID;
      }
      if (!_data.plane.valid) {
        m |= e_PLANE_INVALID;
      }

      if (_data.em.valid && !isStraightDrive()) {
        m |= e_EM_STRAIGHT;
      }
      bool isPitchDiff = false; //(vp.maxPitchDiff > 0) && (_data.rm.pitchDiff >= vp.maxPitchDiff); // TODO: disable condition by default
      if (isPitchDiff && _data.rm.valid) {
        m |= e_EM_RM_PITCHDIFF;
      }
      bool isCrown = false; // (vp.maxCrownAngDiff > 0) && (_data.rm.crownAngDiff >= vp.maxCrownAngDiff);
      if (isCrown) {
        m |= e_ROAD_CROWN;
      }

      if (_data.spc.spcMode) {
        spcUpdateValidFrame();
      }

      Cam2WorldOutputIF::instance().toItrkFrameValidation(&_data);
      OC_C2W_PRINT(e_VALID, (_data.vehicle.validFrame ? e_GREEN : e_RED),
                   "[validateFrameAlgo] veh: %svalid (%d), algo-bits: %s %s %s %s %s %s %s %s %s",
                   (_data.vehicle.validFrame ? "" : "in"), _data.vehicle.validFrameNum,
                   ((m & (e_SENSORS_UNAVAILABLE))              == 0 ? "" : "sensors"),
                   ((m & (e_SPEED_TOO_LOW | e_SPEED_TOO_HIGH)) == 0 ? "" : "speed"),
                   ((m & e_YAWRATE_TOO_HIGH)                   == 0 ? "" : "yawrate"),
                   ((m & e_RADIUS_TOO_SMALL)                   == 0 ? "" : "radius"),
                   ((m & e_ACCELERATION_TOO_HIGH)              == 0 ? "" : "accel"),
                   ((m & e_EM_INVALID)                         == 0 ? "" : "em invalid"),
                   ((m & e_PLANE_INVALID)                         == 0 ? "" : "rm invalid"),
                   ((m & e_EM_STRAIGHT)                        == 0 ? "" : "em straight"),
                   ((m & e_EM_RM_PITCHDIFF)                    == 0 ? "" : "pdiff"),
                   ((m & e_ROAD_CROWN)                         == 0 ? "" : "crown"));
    }

    bool SteadyStateCalibrator::isStraightDrive() {
      const EmData &em = _data.em;
      ValidParams &vp = _data.validParams;

      bool isStraightDrive = _data.em.valid;
      for (int i = 0; i < 3; ++i) {
        bool rotOk = (me_abs(em.rot[i]) < vp.maxEmRot[i]);
        isStraightDrive = isStraightDrive && rotOk;

        // hysteresis mechanism: when rotation is invalidated the threshold is decreased for the
        // next frame (down to vp.hystRotThMin) otherwise it gradually returns
        // to vp.maxEmRot[i]
        if (vp.hystEnabled && em.validFrameNum > 1) {
          if (rotOk) {
            vp.maxEmRot[i] = std::min(vp.hystRotThMax[i], vp.maxEmRot[i] + vp.hystRotThInc);
          } else {
            vp.maxEmRot[i] = std::max(vp.hystRotThMin, vp.maxEmRot[i] - vp.hystRotThInc);
          }
        }
      }
      return isStraightDrive;
    }

    void SteadyStateCalibrator::computeInit(const Cam2WorldSources& source) {
      CalibUtils::PixelLm2_f foe(0.f, 0.f);
      float yaw = 0.f;
      float pitch = 0.f;
      pixelToAngles(foe, yaw, pitch);
      _data.results.yawAngle = yaw;
      _data.results.pitchAngle = pitch;
      _data.results.rollAngle = source.cam.roll;

      Float::MEmath::Mat<3, 3, double> R;
      Float::MEmath::ypr2R((double)yaw, (double)pitch, (double)source.cam.roll, R);
      _data.results.R = matd2f(R);

      _data.results.t = Float::MEmath::zeros<3, float>();
      _data.results.t[1] = -source.cam.camh;

      _data.results.R_init = _data.results.R;
      _data.results.t_init = _data.results.t;

      _data.results.foeDelta = CalibUtils::PixelLm2(0, 0);
      _data.results.foe = _data.cam.origin;

      OC_C2W_PRINT(e_INPUT, e_PURPLE, "[SSCtor::computeInit] yaw=%.2fdeg, pitch=%.2fdeg, roll=%.2fdeg, camh=%.2f\n"
                   "[SSCtor::compute] foe.x: %d, foe.y: %d",
                   _data.results.yawAngle*RAD2DEG, _data.results.pitchAngle*RAD2DEG,
                   _data.results.rollAngle*RAD2DEG, -_data.results.t[1],
                   _data.results.foe.X(), _data.results.foe.Y());
      Cam2WorldOutputIF::instance().tostdoutRT(e_INPUT, e_PURPLE, "[C2WCalibrator::computeInit] ",
                                               _data.results.R, _data.results.t);
    }

    void SteadyStateCalibrator::compute() {
      bool validFrame = false;
      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        validFrame = validFrame || _signal[i].validFrame();
      }

      if (validFrame) {
        computeRotationTranslation();
      }

      computeConfidence();
      Cam2WorldOutputIF::instance().toItrkResult(&_data);
      // Cam2WorldOutputIF::instance().toItrkSPC(&_data);
    }

    void SteadyStateCalibrator::computeRotationTranslation() {
      float camh = _signal[e_CAM_HEIGHT].result();
      _data.results.t = Float::MEmath::zeros<3, float>();
      _data.results.t[1] = -camh;

      // transform results from our internal representation to the Rogriguez representation
      // the function WorldModel::decomposeRotationMatrixIntoLogAngles inverts the sign
      // therefore I use R2ypr
      double yaw   = (double)_signal[e_YAW].result();
      double pitch = (double)_signal[e_PITCH].result();
      double roll  = (double)_signal[e_ROLL].result();
      Float::MEmath::Mat<3, 3, double> R = composeRotationMatrixFromSensorAngles(yaw, pitch, roll);
      Float::MEmath::R2ypr(R, yaw, pitch, roll);

      _data.results.yawAngle   = (float)yaw;
      _data.results.pitchAngle = (float)pitch;
      _data.results.rollAngle  = (float)roll;
      _data.results.R          = matd2f(R);
      
      CalibUtils::PixelLm2_f foef(0.f, 0.f);
      anglesToPixel(yaw, pitch, foef); // foe relative to image origin
      // horizonFull must be even
      _data.results.foeDelta.X() = (int)me_roundf(foef.X());
      _data.results.foeDelta.Y() = (int)(me_roundf(foef.Y()*0.5f)*2.f);
      _data.results.foe = _data.results.foeDelta + _data.cam.origin;

      if (_data.spc.spcMode) {
        // spcUpdateFoe();
        _data.spc.foe = _data.results.foe;
      }

      OC_C2W_PRINT(e_RESULT, e_PURPLE, "[SSCtor::compute] yaw=%.2fdeg, pitch=%.2fdeg, roll=%.2fdeg, camh=%.2f\n"
                   "[SSCtor::compute] foe.x: r(%.2f)+%d=%d, foe.y: r(%.2f)+%d=%d",
                   yaw*RAD2DEG, pitch*RAD2DEG, roll*RAD2DEG, camh,
                   foef.X(), _data.results.foeDelta.X(), _data.results.foe.X(),
                   foef.Y(), _data.results.foeDelta.Y(), _data.results.foe.Y());
    }

    void SteadyStateCalibrator::computeConfidence() {
      float conf[2] = {0.f, 0.f};
      float w[e_C2W_SIG_NUM] = {0.25f, 0.25f, 0.25f, 0.25f}; // why was it: {0.f, 0.4f, 0.3f, 0.3f} ?
      _data.algo.degradeCause = (int)StateDegradeCause::NO_DEGRADE;
      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        conf[0] += w[i] * _signal[i].confidence(0);
        conf[1] += w[i] * _signal[i].confidence(1);
        _data.algo.degradeCause |= _signal[i].degradeCause();
      }

      _data.results.confidence = (unsigned int)me_roundf(100.f*conf[0]);
      _data.results.stringentConfidence = (unsigned int)me_roundf(100.f*conf[1]);

      for (int i=0; i<2; ++i) {
      OC_C2W_PRINT(e_CONF, e_PURPLE, "[SSCtor::computeConfidence] %s conf=mean(%.2f, %.2f, %.2f, %.2f)=%.2f",
                   (i==0 ? "" : "(stringent): "),
                   _signal[e_YAW].confidence(i),
                   _signal[e_PITCH].confidence(i),
                   _signal[e_ROLL].confidence(i),
                   _signal[e_CAM_HEIGHT].confidence(i),
                   conf[i]);
      }
    }

    bool SteadyStateCalibrator::isCalibInRange() const {
      bool inRange = true;
      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        inRange = inRange && _signal[i].inRange();
      }

      return inRange;
    }

    bool SteadyStateCalibrator::isSignalStable() const{
      // determine of signals are stable enough. This will be used in order to determine that state later on.
      bool stableSignal[2] = {true, true}; 
      for (int id=0; id<e_C2W_SIG_NUM; ++id) {
        for (int j=0; j<2; j++){
          bool instableDerivative = _signal[id].unstableMedianCount(j) > _data.confParams[id][j].stableCountThresh; // does sharp derivative indicate on instability?
          bool instableSteadyState = _signal[id].unsteadySigCount(j) > _data.confParams[id][j].stableCountThresh; // does difference from steadystae (difference between windows) indocate on instability?
          bool instable = instableDerivative || instableSteadyState;

          stableSignal[j] = stableSignal[j] && !instable;
        }
      }
      return stableSignal[0];  // NOTICE!! this takes into consideration only the first stability, with the first set of parameters.
    }

  } // Cam2World
} // OnlineCalibration
