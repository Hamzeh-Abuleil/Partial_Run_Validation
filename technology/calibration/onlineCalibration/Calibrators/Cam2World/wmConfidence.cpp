#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/steadyStateCalibrator.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wOutputIF.h"
// check which of these is necessary:
// #include "technology/worldModel/egoMotion_API.h"
#include "technology/worldModel/common/worldModelUtils.h"
#include "technology/worldModel/egoMotion/egoMotionExternalDefs.h"
// #include "technology/worldModel/roadModel_API.h"
#include "technology/worldModel/egoMotion/EgomotionPrivate_API.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/binaryHmm.h" // TODO: remove once moved to utils
#include "technology/calibration/onlineCalibration/CalibUtils/calibMath.h"

namespace OnlineCalibration {
  namespace Cam2World {

  void SteadyStateCalibrator::setWmRoadModelCov(const WorldModel::VRM::SegmentModel* model,
                                                const WorldModel::EgoMotion::RoadModelStorage *storage)
  {
    // setWmRoadModelCov reads data from RM and calculated cov mat, later to be
    // used calculate conf os r & ch.
    // Calculation of RM-Cov: /homes/urilo/Documents/math/other/rmStd.pdf
    // according to the document above: 0-pitch ; 1-roll ; 2-ch
    RmData &r = _data.rm;

    CalibUtils::mat33 nAtA_double = CalibUtils::matf2d(r.nAtA);
    CalibUtils::mat33 invAtA_double = CalibUtils::pinv(nAtA_double);
    CalibUtils::mat33f invAtA = CalibUtils::matd2f(invAtA_double);

    CalibUtils::vec3f N = CalibUtils::vec3f::toVec(model->N);
    float d = model->d;
    float f = _data.cam.focalLm2 * (storage->level == -1 ? 0.5f : 1.f);

    float Nxy = N[1]*N[1] + N[0]*N[0];
    float Nyz = N[1]*N[1] + N[2]*N[2];
    r.validCov = (Nxy > EPS && Nyz > EPS );
    if (!r.validCov) {
      OC_C2W_PRINT(e_INPUT, e_PURPLE, "[setWmRoadModelCov] rm-cov invalid Ny=%g", N[1]);
      r.cov = Float::MEmath::identity<3, float>();
      return;
    }

    float iNxy = 1.f/me_sqrtf(Nxy);
    float iNyz = 1.f/me_sqrtf(Nyz);

    CalibUtils::mat33f J;
    J(0,0) = 0.f;
    J(0,1) =  d*f*N[2]*iNyz;
    J(0,2) = -d * N[1]*iNyz;
    J(1,0) = -d*f*N[1]*iNxy;
    J(1,1) =  d*f*N[0]*iNxy;
    J(1,2) = 0.f;
    J(2,0) = -d*d*f*N[0];
    J(2,1) = -d*d*f*N[1];
    J(2,2) = -d*d * N[2];

    CalibUtils::mat33f Jt = J.transpose();
    r.cov = J * (invAtA * Jt);
  }

  void SteadyStateCalibrator::setWmConfData(const Cam2WorldSources& source) {
    EmData &d = _data.em;
    const WorldModel::VRM::SegmentModel *model = source.rm.model;
    const WorldModel::EgoMotion::RoadModelStorage *storage = source.rm.storage;
    // ErrPropagConfData       epConfData;
    TrackingConfidenceData  cdata;

    if (!storage || !model) {
      _data.rm.resetResConf();
      printDebugWmCsv(model, storage, cdata, -1);
      OC_C2W_PRINT(e_INPUT, e_RED, "[setWmConf] storage/model is null", 0);
      return;
    }

    ASSERT(storage->currDist.size() == storage->prevDist.size());
    ASSERT(storage->currDist.size() == storage->currUndistRS.size());
    ASSERT(storage->currDist.size() == storage->zInv.size());
    unsigned int fullsize = std::min(storage->currDist.size(), storage->prevDist.size()); //number of samples?
    fullsize = std::min(fullsize, (unsigned int)storage->currUndistRS.size());
    fullsize = std::min(fullsize, (unsigned int)storage->zInv.size());

    setWmRoadModelCov(model, storage);

    d.rmsValid = storage->valid;
    d.rmsPtNum[0] = fullsize;
    d.rmsPtNum[1] = storage->currDistOutliers.size();
    _data.rm.resetResConf();

    bool valid = (d.rmsValid && _data.rm.valid);
    if (fullsize == 0 || !valid) {
      OC_C2W_PRINT(e_INPUT, e_RED,
                 "[setWmEgomotionTracking] valid=%d, size=%d", valid, fullsize);
      printDebugWmCsv(model, storage, cdata, -1);
      return;
    }

    OC_C2W_PRINT(e_INPUT, e_GREEN, "[setWmConf] valid=%d, size=%d, "
               "#inliers=%d, level=%d",
               storage->valid, fullsize, d.rmsPtNum[1], storage->level);

    cdata.reset();
    cdata.ZstartInv = 1.f/model->Zstart;
    cdata.ZendInv = 1.f/model->Zend;
    cdata.fInv = _data.cam.invFocalLm2 * (storage->level == -1 ? 2.f : 1.f);

    for (unsigned int i=0; i<fullsize; ++i) {
      cdata.update(model, storage, i);
      printDebugWmCsv(model, storage, cdata, i);
    }

    cdata.scale();

    if (cdata.ptNum < 2) {
      OC_C2W_PRINT(e_INPUT,e_GREEN, "[setWmConf] only valid %d pts", cdata.ptNum);
    }
    setVarOfSignals();
  }


  void SteadyStateCalibrator::setVarOfSignals(){
    // setErrPropagationConf will take data givem from RM and EM and insdure it's given in units of the field.

    for (unsigned int sig_i=e_YAW; sig_i<e_C2W_SIG_NUM; ++sig_i) {
      float input_i;
      switch(sig_i) { // change input into activation function according to signal
        case e_YAW:
          input_i = 1/_data.em.confVec[3];
          break;
        case e_PITCH:
          input_i = 1/_data.em.confVec[4];
          break;
        case e_ROLL:
          input_i = _data.rm.cov(1,1);
          break;
        case e_CAM_HEIGHT:
          input_i = _data.rm.cov(2,2);
          break;
        default:
          input_i = 1/_data.em.confT;
          break;
      }
      _data.em.sigVar[sig_i] = input_i;
    }
    return;
  }

  void TrackingConfidenceData::reset() {
    ptNum = 0;
    AtA = Float::MEmath::diag(CalibUtils::zVec2f());
    Atb = CalibUtils::zVec2f();
    btb = 0.f;

    invL[e_YAW]         = 0.f;
    invL[e_PITCH]     = 1e6;   // L_pitch=1e-6
    invL[e_ROLL]        = 1.67f; // L_roll=0.6
    invL[e_CAM_HEIGHT]  = 5.f;   // L_dist=0.2
    invL2[e_YAW]        = 0.f;
    invL2[e_PITCH]    = 2e6;   // L_pitch=5e-7
    invL2[e_ROLL]       = 10.f;  // L_roll=0.1
    invL2[e_CAM_HEIGHT] = 20.f;  // L_dist=0.05
    for (int i=0; i<e_C2W_SIG_NUM; ++i) {
      distCurr[i] = 0.f;
      distMean[i] = 0.f;
      distVar[i] = 0.f;
      distMin[i] = 100.f;
      distMax[i] = 0.f;
      mRC[i]    = 0.f;
      mRC2[i]   = 0.f;
      sizeRC[i] = 0;
    }
    for (int i=0; i<15; ++i) {
      debugVals[i] = 0.f;
    }
  }

  void TrackingConfidenceData::update(const WorldModel::VRM::SegmentModel *model,
                                      const WorldModel::EgoMotion::RoadModelStorage *storage, unsigned int i) {
    float xdc = storage->currDist[i].x();
    float ydc = storage->currDist[i].y();
    float xdp = storage->prevDist[i].x();
    float ydp = storage->prevDist[i].y();
    float xu  = storage->currUndistRS[i].x();
    float yu  = storage->currUndistRS[i].y();
    float Zinv = storage->zInv[i];
    float conf = storage->confidence[i];

    // foe
    float xc = xdc, yc = ydc, xp = xdp, yp = ydp;
    if (DistortionCorrectionAPI::isDistortionValid(CameraInfo::e_FORWARD)) {
      DistortionCorrectionAPI::rectifySafe(CameraInfo::e_FORWARD, storage->level, xdc, ydc, xc, yc);
      DistortionCorrectionAPI::rectifySafe(CameraInfo::e_FORWARD, storage->level, xdp, ydp, xp, yp);
      updateFoeMeans(xc, yc, xp, yp);
    }

    debugVals[4] = xu;
    debugVals[5] = yu;
    debugVals[6] = xc;
    debugVals[7] = yc;
    debugVals[8] = xp;
    debugVals[9] = yp;
    debugVals[10] = xdc;
    debugVals[11] = ydc;
    debugVals[12] = xdp;
    debugVals[13] = ydp;
    debugVals[14] = conf;

    // RM residual
    if (Zinv < EPS || Zinv < ZendInv || Zinv > ZstartInv) {
      return;
    }

    float Z = 1.f/Zinv;
    float P[3] = {fInv*xu*Z, fInv*yu*Z, Z}; // TODO: convert to vec
    CalibUtils::vec3f N = CalibUtils::vec3f::toVec(model->N);
    for (unsigned int j=e_PITCH; j<e_C2W_SIG_NUM; ++j) {
      if (roadModelAngleDist(P, N, model->d, j)) {  // is this point in a valid location (distance relative to RM plane) in order to proceed computation?
        roadModelAngleUpdate(j);
        roadModelAngleUpdate(j, true);
        sizeRC[j] += 1;
      }
    }

    debugVals[0] = Zinv;
    debugVals[1] = P[0];
    debugVals[2] = P[1];
    debugVals[3] = P[2];
  }

  bool TrackingConfidenceData::roadModelAngleDist(float P[3], CalibUtils::vec3f N, float d,
                                                  int sig) {
    // /homes/nerib/work/projects/dynamicError/estimators_2.lyx
    if (sig == e_CAM_HEIGHT) {
      float dist = N[0]*P[0] + N[1]*P[1] + N[2]*P[2];
      distCurr[sig] = me_fabs(d - dist);
      return true;
    }

    float a = (sig == e_ROLL) ? N[0] : N[2];
    float b = N[1];
    float c = (sig == e_ROLL) ? N[2] : N[0];
    float X0 = (sig == e_ROLL) ? P[0] : P[2];
    float Y0 = P[1];
    float Z0 = (sig == e_ROLL) ? P[2] : P[0];
    if (me_fabs(b) < EPS) {
      distCurr[sig] = -1.f;
      return false;
    }

    float A = -a/b;
    float B = (d - c*Z0)/b;
    float A2 = A*A;
    float B2 = B*B;
    float r2 = X0*X0+Y0*Y0;
    float disc = A2*B2 - (1+A2)*(B2-r2);
    if (disc < -EPS) {
      distCurr[sig] = PI - 0.01f;
      //return true;
      return false;
    }
    disc = std::max(disc, 0);

    float X1 = -A*B + me_sqrtf(disc);
    X1 /= (1+A2);
    float Y1 = A*X1 + B;
    float dAng1 = me_atan2f(Y1*X0-Y0*X1, X1*X0+Y1*Y0);
    dAng1 = me_fabs(dAng1);

    float X2 = -A*B - me_sqrtf(disc);
    X2 /= (1+A2);
    float Y2 = A*X2 + B;
    float dAng2 = me_atan2f(Y2*X0-Y0*X2, X2*X0+Y2*Y0);
    dAng2 = me_fabs(dAng2);

    distCurr[sig] = std::min(dAng1, dAng2);
    return true;
  }

  //void TrackingConfidenceData::roadModelAngleUpdate(float dist, int sig, float invL, float& sum) {
  void TrackingConfidenceData::roadModelAngleUpdate(unsigned int sig, bool stringent) {
    // simple moments
    if (!stringent) {
      distMean[sig] += distCurr[sig];
      distVar[sig] += distCurr[sig]*distCurr[sig];
      if (distCurr[sig] > distMax[sig]) {
        distMax[sig] = distCurr[sig];
      }
      if (distCurr[sig] < distMin[sig]) {
        distMin[sig] = distCurr[sig];
      }
    }

    // confidence function
    static bool square = !Debug::Args::instance().existsParameter("-sWMBC-rmNotDistSquare");
    float iL = stringent ? invL2[sig] : invL[sig];
    float &sum  = stringent ? mRC2[sig] : mRC[sig];
    float dist = distCurr[sig];

    float maxRange = PI;
    if (square) {
      dist *= dist;
      maxRange *= maxRange;
    }

    if (sig == e_CAM_HEIGHT) {
      sum += me_expf(-iL*dist);
      return;
    }

    sum += me_powf(1.f - (dist/maxRange), iL);
  }

  void TrackingConfidenceData::updateFoeMeans(float xc, float yc, float xp, float yp) {
    float dx = xc - xp;
    float dy = yc - yp;
    float r = sqrt(dx*dx+dy*dy);

    if (r < EPS) {
      return;
    }
    ptNum += 1;
    float invR = 1.f/r;
    float ct = dx*invR;
    float st = dy*invR;
    float d  = ct*yc - st*xc;

    AtA(0,0) += st*st;
    AtA(0,1) += -ct*st;
    AtA(1,0) += -ct*st;
    AtA(1,1) += ct*ct;
    Atb[0]   += -st*d;
    Atb[1]   += ct*d;
    btb      += d*d;
  }

  void TrackingConfidenceData::scale() {
    for (unsigned int i=e_PITCH; i<e_C2W_SIG_NUM; ++i) {
      if (sizeRC[i] == 0) {
        continue;
      }
      float sizeInv = 1.f/sizeRC[i];
      distMean[i] *= sizeInv;
      distVar[i] = distVar[i]*sizeInv - distMean[i]*distMean[i];
      mRC[i]  *= sizeInv;
      mRC2[i] *= sizeInv;
    }
  }

  void SteadyStateCalibrator::printDebugWmCsv(const WorldModel::VRM::SegmentModel *model,
                                const WorldModel::EgoMotion::RoadModelStorage *storage,
                                TrackingConfidenceData &cdata, int idx)
  {
    // CSV header:
    // globalFrameIndex grabIdx frameId idx level focal Nx Ny Nz d Zstart Zend Zinv X Y Z xu yu xc yc xp yp xdc ydc xdp ydp conf distPitch distRoll distCamh
#ifdef MEwin
    unsigned int grabIdx = *PrepSys_API::getGrabIndex(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
    unsigned int frameId = *PrepSys_API::getFrameIDs();
    if (globalFrameIndex == 0) {
      OC_C2W_PRINT(e_INPUT, e_WHITE, "[setWmConf-CSV] globalFrameIndex grabIdx frameId idx "
                   "level focal Nx Ny Nz d Zstart Zend Zinv X Y Z xu yu xc yc xp yp xdc ydc "
                   "xdp ydp conf distPitch distRoll distCamh", 0);
    }
    if (idx >= 0) {
      OC_C2W_PRINT(e_INPUT, e_WHITE, "[setWmConf-CSV] "
                 "%d %u %u %d %d "
                 "%.6f %.6f %.6f %.6f %.6f "
                 "%.6f %.6f %.6f %.6f %.6f %.6f "
                 "%.6f %.6f %.6f %.6f %.6f "
                 "%.6f %.6f %.6f %.6f %.6f %.6f "
                 "%.6f %.6f %.6f ",
                 globalFrameIndex, grabIdx, frameId, idx, storage->level,
                 1.f/cdata.fInv, model->N[0], model->N[1], model->N[2], model->d,
                 model->Zstart, model->Zend, cdata.debugVals[0], cdata.debugVals[1], cdata.debugVals[2], cdata.debugVals[3],
                 cdata.debugVals[4], cdata.debugVals[5], cdata.debugVals[6], cdata.debugVals[7], cdata.debugVals[8],
                 cdata.debugVals[9], cdata.debugVals[10], cdata.debugVals[11], cdata.debugVals[12], cdata.debugVals[13], cdata.debugVals[14],
                 cdata.distCurr[e_PITCH], cdata.distCurr[e_ROLL], cdata.distCurr[e_CAM_HEIGHT]);
    } else {
      OC_C2W_PRINT(e_INPUT, e_WHITE, "[setWmConf-CSV] "
                 "%d %u %u nan nan "
                 "nan nan nan nan nan "
                 "nan nan nan nan nan nan "
                 "nan nan nan nan nan "
                 "nan nan nan nan nan nan "
                 "nan nan nan ",
                 globalFrameIndex, grabIdx, frameId);
    }
#endif
  }

  } // namespace Cam2World
} // namespace OnlineCalibration
