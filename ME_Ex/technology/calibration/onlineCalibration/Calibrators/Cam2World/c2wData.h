/**
 * \file c2wData.h
 * \brief Container for all c2w data
 * 
 * \author Uri London
 * \date Jul 11, 2019
 */


#ifndef __OC_CAM2WORLD_DATA_H_
#define __OC_CAM2WORLD_DATA_H_

#include "technology/worldModel/roadModel/roadModelDefs.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wConsts.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/binaryHmm.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationDefs.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"

namespace WorldModel {
  namespace EgoMotion {
    struct RoadModelStorage;
  }
  namespace VRM {
    class SegmentModel;
  }
}

namespace OnlineCalibration {

  namespace Cam2World {

  enum WM_EM_status {
    EM_OK,
    EM_OFF,
    EM_INVALID,
    EM_STANDING
  };

  enum WM_RM_status {
    RM_OK,
    RM_MODEL_NOT_FOUND,
    RM_MODEL_INVALID,
    RM_CLOSE_PLANE_INVALID,
    RM_PLANE_OFFSIDE
  };

  enum ResetMask {
    e_RESET_SKIP         = -1,
    e_RESET_NONE         = 0,
    e_RESET_HIST_YAW     = 1,
    e_RESET_HIST_PITCH   = 2,
    e_RESET_HIST_ROLL    = 4,
    e_RESET_HIST_CAMH    = 8
  };

  struct ValidParams {
    ValidParams() : minSpeed(0.f), maxSpeed(0.f), maxAccel(0.f), maxYawRate(0.f), minRadius(0.f),
    maxPitchDiff(0.f), maxCrownAngDiff(0.f), hystEnabled(false),
    hystRotThMin(0.f), hystRotThInc(0.f) {
      for (int i = 0; i < 3; ++i) {
        maxEmRot[i] = 0.f;
        hystMinRadiusRange[i] = 0.f;
        hystRotThMax[i] = 0.f;
      }
    }

    // basic
    float minSpeed;
    float maxSpeed;
    float maxAccel;
    float maxYawRate;
    float minRadius;

    // needed?
    float maxEmRot[3];
    float maxPitchDiff;

    // crown
    float maxCrownAngDiff;

    // hysteresis: varying threhshold
    bool hystEnabled;
    float hystMinRadiusRange[3]; // hyst for min radius [0] min val, [1] max val, [2] increment step
    float hystRotThMin; // hyst for em_straight: min val
    float hystRotThInc; // hyst for em_straight: incrementation step
    float hystRotThMax[3]; // hyst for em_straight: max val for each dyaw, dpitch, droll
  };

  struct MetaParams {
    MetaParams() {
      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        calibRange[i][0] = calibRange[i][1] = 0.f;
      }
    }

    float calibRange[e_C2W_SIG_NUM][2];
  };

  struct ConfParams {
    ConfParams() : confScale(1.f), maxMedianChange(0.f), stableCountThresh(1), minSteadyThresh(0.f) {}

    float confScale;
    float maxMedianChange;
    int stableCountThresh;
    float minSteadyThresh;
    ActivationFuncParams varHmm;
    ActivationFuncParams stableHmm;
    ActivationFuncParams sampleHmm;
  };

    struct CameraData {
      CameraData() : focalLm2(1.f), invFocalLm2(1.f), origin(0, 0) {}

      float focalLm2;
      float invFocalLm2;
      CalibUtils::PixelLm2 origin; // image origin relative to nominal center
    };

    struct VehicleData {
      VehicleData() : isYawRateAvailable(false), speedAvailable(false), validFrame(false),
                      yawRate(0.f), speed(0.f), radius(0.f), dt(0.f), ds(0.f),
                      totalDistance(0.f), totalTime(0.f), 
                      accel(0.f), validDistance(0.f), validTime(0.f), validFrameNum(0) {}

      bool isYawRateAvailable;
      bool speedAvailable;
      bool validFrame;
      float yawRate;
      float speed;
      float radius;
      float dt;
      float ds;
      float totalDistance;
      float totalTime;
      float accel;

      float validDistance;
      float validTime;
      int validFrameNum;
    };

    struct EmData {
      EmData() : valid(false), status(EM_INVALID), confVec(0.f),
      confT(0.f), confR(0.f), R(CalibUtils::id3f()), t(CalibUtils::zVec3f()), yaw(0.f), pitch(0.f),
      validFrameNum(0), rmsValid(false),
      rmsPtNumEma(0.f)
      {
        rmsPtNum[0] = rmsPtNum[1] = 0;
        for (int i = 0; i < 3; ++i ) {
          rot[i] = 0.f;
        }
        for (int i=0; i<e_C2W_SIG_NUM; ++i) {
          sigVar[i] = 0.f;
        }
      }

      bool valid;
      WM_EM_status status;
      MEtypes::Vector<float,  6> confVec;
      float confT;  // mean of conf_foeX and conf_foeY, from EM
      float confR;
      Float::MEmath::Mat<3,3,float> R;
      Float::MEmath::Vec<3,float> t;
      float rot[3]; // dayw, dpitch, droll
      float yaw;
      float pitch;
      int validFrameNum;
      float sigVar[e_C2W_SIG_NUM];

      bool rmsValid;
      unsigned int rmsPtNum[2];
      float rmsPtNumEma;
    };

#ifdef MEwin
    const MEtl::string s_planeType[4] = {"INVALID_PLANE", "NOT_ENOUGH_POINTS", "FAR_FROM_CALIB", "VALID_PLANE"};
    const MEtl::string s_CStatus[4] = {"NOT_AVAILABLE", "RUNNING", "CONVERGED", "CalibrationStatus_ERROR"};
    const MEtl::string s_SPCError[4] = {"SPCError_NONE", "SPCError_TIMEOUT", "SPCError_OUT_OF_RANGE", "SPCError_INTERNAL"};
#endif
    // OC_C2W_ASTR(s_planeType, 4, "INVALID_PLANE", "NOT_ENOUGH_POINTS", "FAR_FROM_CALIB", "VALID_PLANE");

    struct RmData {
      RmData() : valid(false), status(RM_MODEL_INVALID), numOfInliers(0),
      planeIdx(0), planeValidity(false), message(WorldModel::VRM::VALID_PLANE), camh(0.f), roll(0.f),
      pitch(0.f), pitchDiff(0.f),
      AtA(CalibUtils::id3f()), nAtA(CalibUtils::id3f()), cov(CalibUtils::id3f()) {
        for (int i=0; i<3; ++i) {
          N[i]=0;
        }
        resetResConf();
      }
      void resetResConf() {
        for (int i=0; i<e_C2W_SIG_NUM; ++i) {
          resConfMean[i] = 0.f;
          resConfMeanStringent[i] = 0.f;
          residualPtNum[i] = 0; 
          residualMean[i] = 0.f;
          residualVar[i] = 0.f;
          residualMin[i] = 0.f;
          residualMax[i] = 0.f;
        }
      }

      bool valid;
      bool validCov;
      WM_RM_status status;
      int numOfInliers;
      int planeIdx;
      bool planeValidity;
      float Zstart;
      float Zend;
      WorldModel::VRM::PlaneType message;
      Float::MEmath::Vec<3,float> N;
      float camh;
      float roll;
      float pitch;
      float pitchDiff;
      float resConfMean[e_C2W_SIG_NUM]; // TODO: move to a separate struct
      float resConfMeanStringent[e_C2W_SIG_NUM];
      int   residualPtNum[e_C2W_SIG_NUM];
      float residualMean[e_C2W_SIG_NUM];
      float residualVar[e_C2W_SIG_NUM];
      float residualMin[e_C2W_SIG_NUM];
      float residualMax[e_C2W_SIG_NUM];
      CalibUtils::mat33f AtA;
      CalibUtils::mat33f nAtA;
      CalibUtils::mat33f cov;
    };


    //For now: I'm adding here a struct to hold roll and camh of the under-the-vehicle plane. In the long run I think this should be placed elsewhere
    // and preferably with a different name (I think the word "Data" is inapropriate here)
    struct PlaneData {
      PlaneData() : valid(false), roll(0), camh(0), location(0.f) {}

      bool valid;
      float roll;
      float camh;
      float location;
    };

    struct TrackingConfidenceData {
      TrackingConfidenceData() { reset();}
      void reset();
      void update(const WorldModel::VRM::SegmentModel *model,
                  const WorldModel::EgoMotion::RoadModelStorage *storage, unsigned int i);
      bool roadModelAngleDist(float P[3], CalibUtils::vec3f N, float d, int sig);
      void roadModelAngleUpdate(unsigned int sig, bool stringent=false);
      void updateFoeMeans(float xc, float yc, float xp, float yp);
      void scale();

      // foe conf
      unsigned int ptNum = 0;
      CalibUtils::mat22f AtA;
      CalibUtils::vec2f Atb;
      float btb;

      // RM residual conf
      float distCurr[e_C2W_SIG_NUM];
      float distMean[e_C2W_SIG_NUM];
      float distVar[e_C2W_SIG_NUM];
      float distMin[e_C2W_SIG_NUM];
      float distMax[e_C2W_SIG_NUM];

      float invL[e_C2W_SIG_NUM];
      float invL2[e_C2W_SIG_NUM];
      float mRC[e_C2W_SIG_NUM];
      float mRC2[e_C2W_SIG_NUM];
      float ZstartInv;
      float ZendInv;
      float fInv;
      int sizeRC[e_C2W_SIG_NUM];

      // Debug - writing to csv
      float debugVals[15]; // Zinv,Px,Py,Pz,xu,yu,xc,yc,xp,yp,xdc,ydc,xdp,ydp,conf
    };

  struct AlgoData {
    AlgoData() : pauseReason(0), degradeCause(0), resetMask(e_RESET_NONE), enforcingFirstValid(false) {}

    int pauseReason; // bit-mask encodeing reasons for frame invalidation
    int degradeCause; // bit-mask like pauseReason aligned with output enum def and changes by state
    int resetMask; // bit-mask encoding reasons for reset
    bool enforcingFirstValid; // enforce first frame to be valid regardless of other conditions
  };

  struct SpcData {
   SpcData() : maxAttempts(0), sessionNum(1), progress(0), conf_progress(0), frame_progress(0),
               invalidSignalMask(0), invalidFrameMask(0), status(NOT_AVAILABLE),
               error(SPCError_NONE), spcMode(false), slowMode(false), frame_conv(false), conv(false),
               sessionFailed(false), validFrame(false), stopped(false), foe(0, 0) {}
   SpcData(const SpcData& d) : maxAttempts(d.maxAttempts), sessionNum(d.sessionNum),
                         progress(d.progress), conf_progress(d.conf_progress), frame_progress(d.frame_progress),
                         invalidSignalMask(d.invalidSignalMask), invalidFrameMask(d.invalidFrameMask),
                         status(d.status), error(d.error), spcMode(d.spcMode), slowMode(d.slowMode),
                         frame_conv(d.frame_conv), conv(d.conv), sessionFailed(d.sessionFailed), validFrame(d.validFrame),
                         stopped(d.stopped), foe(d.foe) {}
   SpcData& operator=(const SpcData& d) { maxAttempts=d.maxAttempts; sessionNum=d.sessionNum;
                         progress=d.progress; conf_progress=d.conf_progress; frame_progress=d.frame_progress;
                         invalidSignalMask=d.invalidSignalMask;  invalidFrameMask=d.invalidFrameMask;
                         status=d.status; error=d.error; spcMode=d.spcMode; slowMode=d.slowMode;
                         frame_conv=d.frame_conv; conv=d.conv; sessionFailed=d.sessionFailed; validFrame=d.validFrame;
                         stopped=d.stopped; foe=d.foe; return *this;}
   void reset() { sessionNum=1; progress=0; conf_progress=0; frame_progress=0; invalidSignalMask=0; invalidFrameMask=0;
                  status=RUNNING; error=SPCError_NONE; frame_conv=false; conv=false; sessionFailed=false;
                  validFrame=false; stopped=false; foe.X()=0; foe.Y()=0;}

    unsigned int maxAttempts;
    unsigned int sessionNum;
    unsigned int progress;
    unsigned int conf_progress;
    unsigned int frame_progress;
    int invalidSignalMask;
    int invalidFrameMask;
    // SPC_Status status;
    CalibrationStatus status;
    SPCError error;
    bool spcMode;
    bool slowMode;
    bool frame_conv;
    bool conv;  // should I remove this?
    bool sessionFailed;
    bool validFrame;
    bool stopped;
    CalibUtils::PixelLm2 foe;
  };


    struct ResultBook {
      ResultBook() : confidence(0), stringentConfidence(0),
                     yawAngle(0.f), pitchAngle(0.f), rollAngle(0.f),
                     R(Float::MEmath::identity<3, float>()), t(Float::MEmath::zeros<3, float>()),
                     R_init(Float::MEmath::identity<3, float>()), t_init(Float::MEmath::zeros<3, float>()),
                     foe(0,0), foeDelta(0,0) {}
      ResultBook(const ResultBook& d) : confidence(d.confidence), stringentConfidence(d.stringentConfidence),
                     yawAngle(d.yawAngle), pitchAngle(d.pitchAngle), rollAngle(d.rollAngle),
                     R(d.R), t(d.t), R_init(d.R_init), t_init(d.t_init),
                     foe(d.foe), foeDelta(d.foeDelta) {}
      ResultBook& operator=(const ResultBook& d) { confidence=d.confidence; stringentConfidence=d.stringentConfidence;
                     yawAngle=d.yawAngle; pitchAngle=d.pitchAngle; rollAngle=d.rollAngle;
                     R=d.R; t=d.t; R_init=d.R_init; t_init=d.t_init;
                     foe=d.foe; foeDelta=d.foeDelta; return *this;}

      unsigned int confidence;
      unsigned int stringentConfidence;
      float yawAngle;
      float pitchAngle;
      float rollAngle;
      Float::MEmath::Mat<3, 3, float> R;
      Float::MEmath::Vec<3, float> t;
      Float::MEmath::Mat<3, 3, float> R_init;
      Float::MEmath::Vec<3, float> t_init;
      CalibUtils::PixelLm2 foe; // relative to nominal center (comparable to full+autofix)
      CalibUtils::PixelLm2 foeDelta; // relative to init MEorigin
    };

    struct Cam2WorldData {
      ValidParams validParams;
      MetaParams metaParams;
      ConfParams confParams[e_C2W_SIG_NUM][2];
      CameraData cam;
      VehicleData vehicle;
      EmData em;
      RmData rm;
      PlaneData plane;
      AlgoData algo;
      SpcData spc;
      ResultBook results;
    };

  } // namespace Cam2World
} // namespace OnlineCalibration

#endif // __OC_CAM2WORLD_DATA_H_
