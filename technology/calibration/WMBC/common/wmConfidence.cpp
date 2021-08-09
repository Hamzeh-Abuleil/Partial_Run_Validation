#include "wmbc.h"
#include "technology/calibration/WMBC/common/wmbc_dbg.h"
// check which of these is necessary:
#include "technology/worldModel/common/worldModelUtils.h"
#include "technology/worldModel/egoMotion/egoMotionExternalDefs.h"
#include "technology/calibration/WMBC/wmbc_ServicesUser.h"
#include "technology/worldModel/egoMotion/EgomotionPrivate_API.h"
#include "technology/calibration/WMBC/common/wmbcUtils.h"

namespace WMBC {

  void FOEFinder::setWmRoadModelCov(const WorldModel::VRM::SegmentModel* model,
                                    const WorldModel::EgoMotion::RoadModelStorage *storage)
  {
    // Calculation of RM-Cov: /homes/urilo/Documents/math/other/rmStd.pdf
    RmData &r = _data.rm;

    // r.validCov = r.nAtA.inverse(invAtA); // TODO: find the problem with inverse function (float? bug?)
    float detAtA = *r.nAtA(0,0) * *r.nAtA(1,1) * *r.nAtA(2,2);
    detAtA      -= *r.nAtA(0,0) * *r.nAtA(1,2) * *r.nAtA(1,2);
    detAtA      -= *r.nAtA(1,1) * *r.nAtA(0,2) * *r.nAtA(0,2);
    detAtA      -= *r.nAtA(2,2) * *r.nAtA(0,1) * *r.nAtA(0,1);
    detAtA      += *r.nAtA(0,1) * *r.nAtA(0,2) * *r.nAtA(1,2) * 2;
    r.validCov = (me_fabs(detAtA) > EPS);
    if (!r.validCov) {
      WMBC_PRINT(WmbcDbgStg::e_INPUT, WmbcDbgClr::e_PURPLE, "[setWmRoadModelCov] rm-cov invalid detAtA=%g", detAtA);
      r.cov.zeroAll();
      return;
    }
    WorldModel::EMat3 invAtA;
    *invAtA(0,0) = *r.nAtA(1,1) * *r.nAtA(2,2) - *r.nAtA(1,2) * *r.nAtA(1,2);
    *invAtA(0,1) = *r.nAtA(0,2) * *r.nAtA(1,2) - *r.nAtA(2,2) * *r.nAtA(0,1);
    *invAtA(0,2) = *r.nAtA(0,1) * *r.nAtA(1,2) - *r.nAtA(1,1) * *r.nAtA(0,2);
    *invAtA(1,1) = *r.nAtA(0,0) * *r.nAtA(2,2) - *r.nAtA(0,2) * *r.nAtA(0,2);
    *invAtA(1,2) = *r.nAtA(0,1) * *r.nAtA(0,2) - *r.nAtA(0,0) * *r.nAtA(1,2);
    *invAtA(2,2) = *r.nAtA(0,0) * *r.nAtA(1,1) - *r.nAtA(0,1) * *r.nAtA(0,1);
    *invAtA(1,0) = *invAtA(0,1);
    *invAtA(2,0) = *invAtA(0,2);
    *invAtA(2,1) = *invAtA(1,2);
    invAtA /= detAtA;

    WorldModel::EVec3 N = model->N;
    float d = model->d;
    float f = _data.camera.focalLm2 * (storage->level == -1 ? 0.5f : 1.f);

    float Nxy = N[1]*N[1] + N[0]*N[0];
    float Nyz = N[1]*N[1] + N[2]*N[2];
    r.validCov = (Nxy > EPS && Nyz > EPS );
    if (!r.validCov) {
      WMBC_PRINT(WmbcDbgStg::e_INPUT, WmbcDbgClr::e_PURPLE, "[setWmRoadModelCov] rm-cov invalid Ny=%g", N[1]);
      r.cov.zeroAll();
      return;
    }

    float iNxy = 1.f/me_sqrtf(Nxy);
    float iNyz = 1.f/me_sqrtf(Nyz);

    float J01 =  d*f*N[2]*iNyz;
    float J02 = -d * N[1]*iNyz;
    float J10 = -d*f*N[1]*iNxy;
    float J11 =  d*f*N[0]*iNxy;
    float J20 = -d*d*f*N[0];
    float J21 = -d*d*f*N[1];
    float J22 = -d*d * N[2];
    WorldModel::EMat3 J({0.f, J01, J02, J10, J11, 0.f, J20, J21, J22});
    WorldModel::EMat3 Jt = J.transpose();

    r.cov = J * (invAtA * Jt);
  }

  void FOEFinder::setWmEgomotionTracking() {
    EmData &d = _data.em;
    auto * rmApi=WmbcServicesUser::instance().get<WorldModel::RM::RoadModelService_API>();
    assert(rmApi!=nullptr);
    const WorldModel::VRM::SegmentModel& model = rmApi->getClosestPlane();
    const WorldModel::EgoMotion::RoadModelStorage &storage = WorldModel::EgoMotion::getRoadModelStorage(MEtypes::RCI_FORWARD);
    TrackingConfidenceData cdata;

    ASSERT(storage.currDist.size() == storage.prevDist.size());
    ASSERT(storage.currDist.size() == storage.currUndistRS.size());
    ASSERT(storage.currDist.size() == storage.zInv.size());
    unsigned int fullsize = std::min(storage.currDist.size(), storage.prevDist.size());
    fullsize = std::min(fullsize, (unsigned int)storage.currUndistRS.size());
    fullsize = std::min(fullsize, (unsigned int)storage.zInv.size());

    setWmRoadModelCov(&model, &storage);

    d.rmsValid = storage.valid;
    d.rmsPtNum[0] = fullsize;
    d.rmsPtNum[1] = storage.currDistOutliers.size();
    d.epiValid = false;
    for (unsigned int i=e_HORIZON; i<e_CALIB_DOF_NUM; ++i) {
      _data.rm.resConfMean[i] = 0.f;
      _data.rm.resConfMeanStringent[i] = 0.f;
      _data.rm.residualPtNum[i] = 0;
      _data.rm.residualMean[i] = 0.f;
      _data.rm.residualVar[i] = 0.f;
      _data.rm.residualMin[i] = 0.f;
      _data.rm.residualMax[i] = 0.f;
    }

    bool valid = (d.rmsValid && _data.rm.valid);
    if (fullsize == 0 || !valid) {
      WMBC_PRINT(WmbcDbgStg::e_INPUT, WmbcDbgClr::e_RED, "[setWmEgomotionTracking] valid=%d, size=%d", valid, fullsize);
      printDebugCsv(model, storage, cdata, -1);
      return;
    }

    WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_GREEN, "[setWmEgomotionTracking] valid=%d, size=%d, "
               "#inliers=%d, level=%d",
               storage.valid, fullsize, d.rmsPtNum[1], storage.level);

    cdata.reset();
    cdata.ZstartInv = 1.f/model.Zstart;
    cdata.ZendInv = 1.f/model.Zend;
    cdata.fInv = _data.camera.invFocalLm2 * (storage.level == -1 ? 2.f : 1.f);

    for (unsigned int i=0; i<fullsize; ++i) {
      cdata.update(&model, &storage, i);
      printDebugCsv(model, storage, cdata, i);
    }

    d.epiInvalidPercent = 1.f*(fullsize-cdata.ptNum)/fullsize;
    cdata.scale();
    updateFoeConfidence(cdata, storage.level);

    if (cdata.ptNum < 2) {
      WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_GREEN, "[setWmEgomotionTracking] only valid %d pts", cdata.ptNum);
    }
  }

  void FOEFinder::updateFoeConfidence(TrackingConfidenceData &c, int level) {
    EmData &d = _data.em;

    float det = (c.mC2*c.mS2 - c.mCS*c.mCS);
    if (me_fabs(det) < EPS) { // TODO: what is the meaning of det=0
      WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_GREEN, "[setWmEgomotionTracking] zero det", 0);
      return;
    }

    float invDet = 1.f/det;
    float x = (c.mCS*c.mDC - c.mC2*c.mDS)*invDet;
    float y = (c.mS2*c.mDC - c.mCS*c.mDS)*invDet;
    //d.epiErr = 0.5f*c.mC2*y*y + 0.5f*c.mS2*x*x + 0.5f*c.mD2 - c.mCS*x*y - c.mDC*y + c.mDS*x;
    d.epiErr = c.mC2*y*y + c.mS2*x*x + c.mD2 - 2.f*c.mCS*x*y - 2.f*c.mDC*y + 2.f*c.mDS*x; // average residual (residual divided by ptNum)
    float levelScale = (level == -1 ? 2.f : 1.f);
    d.epiErrAng = (d.epiErr > EPS) ? me_sqrtf(d.epiErr)*levelScale*_data.camera.invFocalLm2 : -1.f;
    d.epiCov[0] = (c.ptNum > 0) ? c.mC2*invDet/c.ptNum : -1.f; // Cov_xx
    d.epiCov[1] = (c.ptNum > 0) ? c.mS2*invDet/c.ptNum : -1.f; // Cov_yy
    d.epiCov[2] = (c.ptNum > 0) ? c.mCS*invDet/c.ptNum : -1.f; // Cov_xy
    d.foeRectLm2.X() = x*levelScale;
    d.foeRectLm2.Y() = y*levelScale;
    d.epiFoeAngle[0] = d.foeRectLm2.X()*_data.camera.invFocalLm2;
    d.epiFoeAngle[1] = d.foeRectLm2.Y()*_data.camera.invFocalLm2;
    d.epiValid = true;

    // exp-moving-avs for quality test for convergence
    const float EMA_FAC = 0.99f;
    const float W[3] = {0.3f, 0.3f, 0.2f};
    float W3 = 1.f-W[0]-W[1]-W[2];
    float curr = 0.5f*(me_sqrtf(d.epiCov[0]) +  me_sqrtf(d.epiCov[1]));
    d.epiCovEma = EMA_FAC*d.epiCovEma + (1.f-EMA_FAC)*curr;
    curr = d.rmsPtNum[0];
    d.rmsPtNumEma = EMA_FAC*d.rmsPtNumEma + (1.f-EMA_FAC)*curr;
    curr = me_fabs(d.yaw - d.epiFoeAngle[0]);
    curr += me_fabs(d.pitch - d.epiFoeAngle[1]);
    curr *= 0.5f;
    d.epiDiffEma = EMA_FAC*d.epiDiffEma + (1.f-EMA_FAC)*curr;
    curr = d.epiErrAng;
    d.epiErrEma = EMA_FAC*d.epiErrEma + (1.f-EMA_FAC)*curr;

    d.epiQuality  = W[0]*linearActivation(d.epiCovEma, 0.35f, 0.15f);
    d.epiQuality += W[1]*linearActivation(d.rmsPtNumEma, 20.f, 70.f);
    d.epiQuality += W[2]*linearActivation(d.epiDiffEma*RAD2DEG, 1.f, 0.1f);
    d.epiQuality += W3  *linearActivation(d.epiErrEma*RAD2DEG, 0.6f, 0.1f);


    // fill plane fit residual values
    for (unsigned int i=e_HORIZON; i<e_CALIB_DOF_NUM; ++i) {
      _data.rm.resConfMean[i] = c.mRC[i];
      _data.rm.resConfMeanStringent[i] = c.mRC2[i];
      _data.rm.residualPtNum[i] = c.sizeRC[i];
      _data.rm.residualMean[i] = c.distMean[i];
      _data.rm.residualVar[i] = c.distVar[i];
      _data.rm.residualMin[i] = c.distMin[i];
      _data.rm.residualMax[i] = c.distMax[i];
    }

    WMBC_PRINT(WmbcDbgStg::e_INPUT,WmbcDbgClr::e_GREEN,
               "[setWmEgomotionTracking] my_epi_err=%.2f, foe=(%.2f, %.2f)",
               d.epiErr, x, y);
  }

  void TrackingConfidenceData::reset() {
    mC2 = 0.f;
    mS2 = 0.f;
    mCS = 0.f;
    mDC = 0.f;
    mDS = 0.f;
    mD2 = 0.f;
    ptNum = 0;

    invL[e_YAW]         = 0.f;
    invL[e_HORIZON]     = 1e6;   // L_pitch=1e-6
    invL[e_ROLL]        = 1.67f; // L_roll=0.6
    invL[e_CAM_HEIGHT]  = 5.f;   // L_dist=0.2
    invL2[e_YAW]        = 0.f;
    invL2[e_HORIZON]    = 2e6;   // L_pitch=5e-7
    invL2[e_ROLL]       = 10.f;  // L_roll=0.1
    invL2[e_CAM_HEIGHT] = 20.f;  // L_dist=0.05
    for (int i=0; i<e_CALIB_DOF_NUM; ++i) {
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
    float P[3] = {fInv*xu*Z, fInv*yu*Z, Z};
    //float dist = 0.f;
    for (unsigned int j=e_HORIZON; j<e_CALIB_DOF_NUM; ++j) {
      // if (roadModelAngleDist(P, model->N, model->d, j, dist)) {
      //   roadModelAngleUpdate(dist, j, invL[j], mRC[j]);
      //   roadModelAngleUpdate(dist, j, invL2[j], mRC2[j]);
      //   sizeRC[j] += 1;
      // }
      if (roadModelAngleDist(P, model->N, model->d, j)) {
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

  bool TrackingConfidenceData::roadModelAngleDist(float P[3], WorldModel::EVec3 N, float d,
                                                  int sig) {
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

    mC2 += ct*ct;
    mS2 += st*st;
    mCS += ct*st;
    mDC += ct*d;
    mDS += st*d;
    mD2 += d*d;
  }

  void TrackingConfidenceData::scale() {
    if (ptNum >= 2) {
      mC2 /= ptNum;
      mS2 /= ptNum;
      mCS /= ptNum;
      mDC /= ptNum;
      mDS /= ptNum;
      mD2 /= ptNum;
    }

    for (unsigned int i=e_HORIZON; i<e_CALIB_DOF_NUM; ++i) {
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

  void FOEFinder::printDebugCsv(const WorldModel::VRM::SegmentModel& model,
                                const WorldModel::EgoMotion::RoadModelStorage &storage,
                                TrackingConfidenceData &cdata, int idx)
  {
    // CSV header:
    // globalFrameIndex grabIdx frameId idx level focal Nx Ny Nz d Zstart Zend Zinv X Y Z xu yu xc yc xp yp xdc ydc xdp ydp conf distPitch distRoll distCamh
#ifdef MEwin
    unsigned int grabIdx = *PrepSys_API::getGrabIndex(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
    unsigned int frameId = *PrepSys_API::getFrameIDs();
    if (idx >= 0) {
      WMBC_PRINT(WmbcDbgStg::e_INPUT, WmbcDbgClr::e_WHITE, "[setWmEgomotionTracking-CSV] "
                 "%d %u %u %d %d "
                 "%.6f %.6f %.6f %.6f %.6f "
                 "%.6f %.6f %.6f %.6f %.6f %.6f "
                 "%.6f %.6f %.6f %.6f %.6f "
                 "%.6f %.6f %.6f %.6f %.6f %.6f "
                 "%.6f %.6f %.6f ",
                 globalFrameIndex, grabIdx, frameId, idx, storage.level,
                 1.f/cdata.fInv, model.N[0], model.N[1], model.N[2], model.d,
                 model.Zstart, model.Zend, cdata.debugVals[0], cdata.debugVals[1], cdata.debugVals[2], cdata.debugVals[3],
                 cdata.debugVals[4], cdata.debugVals[5], cdata.debugVals[6], cdata.debugVals[7], cdata.debugVals[8],
                 cdata.debugVals[9], cdata.debugVals[10], cdata.debugVals[11], cdata.debugVals[12], cdata.debugVals[13], cdata.debugVals[14],
                 cdata.distCurr[e_HORIZON], cdata.distCurr[e_ROLL], cdata.distCurr[e_CAM_HEIGHT]);
    } else {
      WMBC_PRINT(WmbcDbgStg::e_INPUT, WmbcDbgClr::e_WHITE, "[setWmEgomotionTracking-CSV] "
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

} // namespace WMBC
