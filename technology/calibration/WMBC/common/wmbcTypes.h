/*
 * wmbcTypes.h
 *
 *  Created on: Jun 08, 2016
 *      Author: urilo
 */

#ifndef WMBC_TYPES_H_
#define WMBC_TYPES_H_

#include "technology/mobilib/fix/common/MEXimage/typeImages.h"
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "utilities/saddlePoints/common/types.h"
#include "technology/worldModel/roadModel/roadModelDefs.h"
#include "basicTypes/geo/geo2D/Pixel.h"
#include "basicTypes/geo/GeometryDefinitions.h"
#include "functionality/calibration/cameraInformationProperTypes.h"

namespace WorldModel {
  namespace CameraModel {
    class IntrinsicCameraModel;
  }
  namespace EgoMotion {
    struct RoadModelStorage;
  }
  namespace VRM {
    class SegmentModel;
  }
}

// namespace Float {
//   namespace MEmath {
//     template <int ROWS, class T> class Vec;
//     template <int ROWS, int COLS, class T> class Mat;
//   }
// }


namespace WMBC {

  typedef Float::MEmath::Vec<3, float> Vec3;
  typedef Float::MEmath::Vec<3, double> Vec3d;
  typedef Float::MEmath::Mat<3, 3, float> Mat3;
  typedef Float::MEmath::Mat<3, 3, double> Mat3d;
  typedef Float::MEmath::Mat<4, 4, float> Mat4;
  typedef Float::MEmath::Mat<4, 4, double> Mat4d;

  typedef MEtypes::ptr_vector<SaddlePoints::Types::SPInfo> SPInfoVec;
  typedef MEtypes::ptr_vector<Float::MEmath::Vec<2, double> > SPVec;

  typedef MEtypes::DynPixel<float,MEtypes::DistortionModel::DM_ORIGINAL,MEtypes::PixLevel::PIXLEVEL_M2> PixelLm2_f;

  //struct ValidParams;

  static const float PI = 3.1415926535897931f;
  static const float RAD2DEG = 57.29577951308232f;
  static const float DEG2RAD = 0.017453292519943295f;
  static const float DEG2RAD_SQR = DEG2RAD*DEG2RAD;
  static const float MPS2KPH = 3.6f;
  static const float KPH2MPS = 0.28f;
  static const float EPS = 1e-6;
  static const float INVALID_VAL = 1905.f; // annus mirabilis
  static const int HIST_BIN_NUM = 500;
  // static const float HIST_LOW_BOUND[4] = {-47.f*4, -40.f*4, -10.f*PI/180.f, 1.f}; // yaw [px lm2], hor [px lm2], roll [rad], camH [m]
  // static const float HIST_BIN_SIZE[4] = {0.5f, 0.5f, 0.05f*DEG2RAD, 0.001f}; // yaw [px lm2], hor [px lm2], roll [rad], camH [m]
  static const float HIST_BIN_SIZE[4] = {0.05f*DEG2RAD, 0.05f*DEG2RAD, 0.05f*DEG2RAD, 0.001f}; // yaw [rad], hor [rad], roll [rad], camH [m]
  static const int HIST_MAX_SIZE = 1E5;
  static const int HIST_START_FRAME = 5;
  static const float DECAY_FACTOR = 0.97f;
  static const int START_DECAY = 33;
  static const unsigned int HIST_MEAN_BUFF_SIZE = 8; // need to be power of 2 and greater than HIST_START_FRAME

  static const int SAMPLE_NUM_EXTRA_DEFAULT = 20;
  static const int DELAY_VALID_TH_DEFAULT = -1; // disabled by default
  static const float QUALITY_FACTOR[4] = {0.015f, 0.015f, 0.015f, 0.015f}; // [1/m]
  static const float QUALITY_SHIFT[4] = {0.05f, 0.05f, 0.05f, 0.05f};

  static const int CONF_GRADE_NUM = 7; // 3 bits for REM confidence
  static const float CONF_TH = 0.75f;
  static const float CONF_GRADE_LIMITS[CONF_GRADE_NUM] = {0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f};
  static const int CONF_BIT_NUM = 3;

  static const float CONF_PARAMS0[8] = {0.117779f*DEG2RAD_SQR, 0.189041f*DEG2RAD_SQR, 1.159724f, 3.114018f, 0.01f, 0.922616f, 0.700734f, 0.260066f};
  static const float CONF_PARAMS1[8] = {0.027635f*DEG2RAD_SQR, 0.098897f*DEG2RAD_SQR, 1.138975f, 3.093269f, 0.01f, 0.336952f, 0.071959f, 0.888841f};
  static const float CONF_PARAMS2[8] = {0.011479f*DEG2RAD_SQR, 0.082741f*DEG2RAD_SQR, 0.226997f, 2.181291f, 1.007205f, 4.395534f, 0.128337f, 0.832463f}; 

  static const int CROWN_SAMP_NUM = 5;

  //plane sync related consts
  const int PLANE_BUFFER_CAPACITY = 8; //Must be a power of 2 due to usage of fastCyclicVector
  const float futurePlaneFac = 0.45;
  const float maxDistFromLocation = 3;
  
  const float MAX_IN_FOE_ANG = 10.f*DEG2RAD;
  const float IM_MARGIN = 40.f; // px level -2
  
  enum CalibMode {
    e_None,
    e_Autofix,
    e_SPC,  // Service Point Calibration
    e_SLC, // Stationless
    e_MODE_NUM
  };

  enum CALIB_DOF {
    e_YAW,
    e_HORIZON,
    e_ROLL,
    e_CAM_HEIGHT,
    e_CALIB_DOF_NUM
  };

  enum CalibStatus {
    e_CALIB_UNDEFINED,
    e_CALIB_OK,
    e_CALIB_CAL,
    e_CALIB_PAUSED,
    e_CALIB_RUN_ERROR,
    e_CALIB_TIMEOUT,
    e_CALIB_TIMEOUT_TIME,
    e_CALIB_TIMEOUT_DISTANCE,
    e_CALIB_ERROR_OUT_OF_RANGE,
    e_CALIB_FAILSAFE_YAW,
    e_CALIB_FAILSAFE_HORIZON,
    e_CALIB_FAILSAFE_ROLL,
    e_CALIB_GENERAL
  };

  enum CalibCoreStatus {
    e_CORE_INIT,
    e_CORE_SUCCESS,
    e_CORE_ERROR
  };

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

  enum NonValidBits {
    e_SPEED_TOO_LOW                = 1,
    e_SPEED_TOO_HIGH               = 2,
    e_YAWRATE_TOO_HIGH             = 4,
    e_RADIUS_TOO_SMALL             = 8,
    e_ACCELERATION_TOO_HIGH        = 16,
    e_REVERSE_IS_ON                = 32,
    e_DYNAMIC_SUSPENSION_ACTIVE    = 64, // related to kafas-bmw signal dynamicCameraHeightDelta
    e_EM_INVALID                   = 128,
    e_RM_INVALID                   = 256,
    e_EM_STRAIGHT                  = 512,
    e_EM_RM_PITCHDIFF              = 1024,
    e_ROAD_CROWN                   = 2048,
    e_SPEED_HP_INVALID             = 4096, // related to kafas-bmw signal speedHighPrecision
    e_LEFT_TARGET_NOT_FOUND        = 8192, // stationless specific
    e_RIGHT_TRAGET_NOT_FOUND       = 16384, // stationless specific
    e_ORIENTATION_INCORRECT        = 32768, // stationless specific
    e_FRAME_VALID_COND_NUM         = 16
  };

  enum NonConvBits {
    e_HIGH_VAR       = 1,
    e_SMALL_SAMPLE   = 2,
    e_NON_STABLE     = 4,
    e_LOW_QUALITY    = 8,
    e_CONV_COND_NUM  = 4
  };

  const int VEHICLE_INVALID = (e_SPEED_TOO_LOW |
                               e_SPEED_TOO_HIGH |
                               e_YAWRATE_TOO_HIGH |
                               e_RADIUS_TOO_SMALL |
                               e_ACCELERATION_TOO_HIGH |
                               e_DYNAMIC_SUSPENSION_ACTIVE
                              );

  const int EM_STR_INVALID = (e_YAWRATE_TOO_HIGH |
                              e_RADIUS_TOO_SMALL |
                              e_EM_INVALID
                             );

  const int EM_QUAL_INVALID = (EM_STR_INVALID |
                               e_REVERSE_IS_ON
                              );

  const int FOE_INVALID = (VEHICLE_INVALID |
                           e_EM_INVALID |
                           e_EM_STRAIGHT |
                           e_REVERSE_IS_ON // epiCov
                          );

  const int ROLL_INVALID = (FOE_INVALID |
                            e_RM_INVALID |
                            e_EM_RM_PITCHDIFF |
                            e_ROAD_CROWN
                           );

  const int CAMH_INVALID = (ROLL_INVALID |
                            e_SPEED_HP_INVALID
                           );

  const int INVALID_MASK[e_CALIB_DOF_NUM] = {FOE_INVALID, FOE_INVALID, ROLL_INVALID, CAMH_INVALID};

  enum ResetMask {
    e_RESET_SKIP         = -1,
    e_RESET_NONE         = 0,
    e_RESET_HIST_YAW     = 1,
    e_RESET_HIST_HORIZON = 2,
    e_RESET_HIST_ROLL    = 4,
    e_RESET_HIST_CAMH    = 8,
    e_RESET_SUSPENSION   = 16,
    e_RESET_WMFOERESET   = 32,
    e_RESET_DONE_YAW     = 64,
    e_RESET_DONE_HORIZON = 128,
    e_RESET_DONE_ROLL    = 256,
    e_RESET_DONE_CAMH    = 512
  };

  enum WM_ResetSignal {
    e_WMRESET_NONE = 0,
    e_WMRESET_SKIP = 1,
    e_WMRESET_CONV = 2,
    e_WMRESET_EM_STAGGER = 3,
    e_WMRESET_LARGE = 4,
  };

  // TODO: ctor for every struct!

  struct ValidParams {
    ValidParams() : minSpeed(0.f), maxSpeed(0.f), maxAccel(0.f), maxYawRate(0.f), minRadius(0.f),
    maxPitchDiff(0.f), maxCrownAngDiff(0.f), useSpeedHighPrecision(false), useDynamicSuspension(false),
    delayValidThreshold(0), hystEnabled(false), hystRotThMin(0.f), hystRotThInc(0.f),
    maxEpiErr(0.f), maxEpiCov(0.f), minEpiSize(0) {
      for (int i = 0; i < 3; ++i) {
        rotThreshold[i] = 0.f;
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
    float rotThreshold[3]; // todo: change to maxEmRotation;
    float maxPitchDiff;

    // crown
    float maxCrownAngDiff;

    // Kafas
    bool useSpeedHighPrecision;
    bool useDynamicSuspension;

    // hysteresis
    int delayValidThreshold;
    bool hystEnabled;
    float hystMinRadiusRange[3];
    float hystRotThMin;
    float hystRotThInc;
    float hystRotThMax[3];

    // other
    float maxEpiErr;
    float maxEpiCov;
    unsigned int minEpiSize;
  };

  struct ConvParams {
    ConvParams() : convSampleNumThreshold(0), convStableNumThreshold(0), useConvMoving(false), 
    convSampleNumThresholdFS(0), convStableNumThresholdFS(0), minQuality(0),
		   minEmEpiQuality(0.f), minCrownQuality(0.f), camhSdmValue(false) {
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        convMaxVar[i] = 0.f;
        convMovingSampleNumThreshold[i] = 0.f;
        convStableMaxDiff[i] = 0.f;
      }
      for (int i = 0; i < 4; ++i) {
        convMovingMarksFoe[i] = 0.f;
        convMovingMarksRoll[i] = 0.f;
        convMovingSamples[i] = 0.f;
      }
    }

    int convSampleNumThreshold; // TODO: change to minSampleNum;
    int convSampleNumThresholdNightSnow; // default min num of frames for snow+night conditions
    int convStableNumThreshold; // TODO: change to minStableNum;
    float convMaxVar[e_CALIB_DOF_NUM]; // variance threshold for convergence [rad^2]
    int convMovingSampleNumThreshold[e_CALIB_DOF_NUM]; // TODO: what to call this?
    float convStableMaxDiff[e_CALIB_DOF_NUM]; // TODO: change to minStableRange?
    bool useConvMoving;
    float convMovingMarksFoe[4]; // TODO: change to what?
    float convMovingMarksRoll[4];
    float convMovingSamples[4];
    int convSampleNumThresholdFS;
    int convStableNumThresholdFS;
    float minQuality;
    float minEmEpiQuality;
    float minCrownQuality;
    bool camhSdmValue;
  };

  struct MetaParams {
    MetaParams() : runMode(e_Autofix), maxAttempts(0), calcSwitchMask(0), 
    useSpecialLimits(false), camHeightRangeSevere(0.f),
    maxTime(0.f), maxDist(0.f), maxValidFrames(0), timeoutTimeMinSpeed(0.f), sensor8MP(false) {
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        foeDeltaThresholdFS[i] = 0.f;
        foeRange[i][0] = foeRange[i][1] = 0.f;
        if (i < e_CAM_HEIGHT) {
          afixClippedJump[i] = 0.f;
        }
      }
      foeDeltaThresholdFS[e_CAM_HEIGHT] = 0.f;
    }

    CalibMode runMode;
    int maxAttempts;
    int calcSwitchMask;

    // result clipping and failsafe
    float foeDeltaThresholdFS[e_CALIB_DOF_NUM];
    float afixClippedJump[e_CALIB_DOF_NUM-1]; // camh hasn't this feature

    // value accepted range
    bool useSpecialLimits;
    float foeRange[e_CALIB_DOF_NUM][2];

    // safety range for camH
    float camHeightRangeSevere;
    
    // timeout
    float maxTime;
    float maxDist;
    int maxValidFrames;
    float timeoutTimeMinSpeed;

    // adcam
    bool sensor8MP; // special origin setting for 8MP sensors
  };

  struct ConfidenceParams {
    float varMin;
    float varMax;
    float varFactorMin;
    float varFactorMax;
    float wgtNumMin;
    float wgtNumMax;
    float wgtRel;
    float wgtAbs;

    ConfidenceParams() : varMin(EPS), varMax(1.f), varFactorMin(EPS), varFactorMax(1.f),
    wgtNumMin(0.f), wgtNumMax(1.f), wgtRel(1.f), wgtAbs(0.f) {}
    ConfidenceParams(const float p[8]) : varMin(p[0]), varMax(p[1]), varFactorMin(p[2]), varFactorMax(p[3]),
    wgtNumMin(p[4]), wgtNumMax(p[5]), wgtRel(p[6]), wgtAbs(p[7]) {}
  };

  struct TrackingConfidenceData {
    // TrackingConfidenceData() : ptNum(0), mC2(0.f), mS2(0.f), mCS(0.f), mDC(0.f), mDS(0.f), mD2(0.f) {}
    TrackingConfidenceData() { reset();}
    void reset();
    void update(const WorldModel::VRM::SegmentModel *model,
                const WorldModel::EgoMotion::RoadModelStorage *storage, unsigned int i);
    bool roadModelAngleDist(float P[3], WorldModel::EVec3 N, float d, int sig);
    void roadModelAngleUpdate(unsigned int sig, bool stringent=false);
    void updateFoeMeans(float xc, float yc, float xp, float yp);
    void scale();

    // foe conf
    unsigned int ptNum = 0;
    float mC2;
    float mS2;
    float mCS;
    float mDC;
    float mDS;
    float mD2;

    // RM residual conf
    float distCurr[e_CALIB_DOF_NUM];
    float distMean[e_CALIB_DOF_NUM];
    float distVar[e_CALIB_DOF_NUM];
    float distMin[e_CALIB_DOF_NUM];
    float distMax[e_CALIB_DOF_NUM];

    float invL[e_CALIB_DOF_NUM];
    float invL2[e_CALIB_DOF_NUM];
    float mRC[e_CALIB_DOF_NUM];
    float mRC2[e_CALIB_DOF_NUM];
    float ZstartInv;
    float ZendInv;
    float fInv;
    int sizeRC[e_CALIB_DOF_NUM];

    // Debug
    float debugVals[15]; // Zinv,Px,Py,Pz,xu,yu,xc,yc,xp,yp,xdc,ydc,xdp,ydp,conf
  };

  struct OriginData {
    //OriginData() : valid(false), init(false), 
    //distNominalLm2(0.f, 0.f), distLm2(0.f, 0.f),
    //distDeltaLm2(0.f, 0.f), rectLm2(0.f, 0.f) {}
    OriginData() : valid(false), init(false),
                   distLimitLm2X(0.f), distLimitLm2Y(0.f)
    {
      distNominalLm2[0] = distNominalLm2[1] = 0.f;
      distLm2[0] = distLm2[1] = 0.f;
      distDeltaLm2[0] = distDeltaLm2[1] = 0.f;
      rectLm2[0] = rectLm2[1] = 0.f;
    } 

    bool valid;
    bool init;
    float distLimitLm2X;
    float distLimitLm2Y;
    PixelLm2_f distNominalLm2;
    PixelLm2_f distLm2;
    PixelLm2_f distDeltaLm2;
    PixelLm2_f rectLm2;
  };

  struct CameraData {
    // origin should not be array, but this whole struct should be instanciated as an array (each camera has its own focal)
    CameraData() : night(false), snow(false), snowNightMode(false), nonSnowNightCounter(1000), focalLm2(1.f), invFocalLm2(1.f) {}

    bool night;
    bool snow;
    bool snowNightMode;
    int nonSnowNightCounter;
    float focalLm2;
    float invFocalLm2;
    OriginData origin[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  };

  struct EmData {
    EmData() : valid(false), rmsValid(false), epiValid(false),
               status(EM_INVALID), conf(0.f), confT(0.f), confR(0.f), straightScore(0.f),
               yaw(0.f), pitch(0.f), epiErr(0.f), epiErrAng(0.f), epiInvalidPercent(0.f), validFrames(0), validStraightFrameNum(0),
               epiQualityFrameNum(0), epiQuality(0.f), rmsPtNumMean(0.f),
               epiCovEma(0.f), rmsPtNumEma(0.f), epiDiffEma(0.f), epiErrEma(0.f)
    { //, foeRectLm2(0.f, 0.f) {
      foeRectLm2.X() = foeRectLm2.Y() = 0.f;
      for (int i = 0; i < 3; ++i ) {
        t[i] = 0.f;
        ypr[i] = 0.f;
        epiCov[i] = 0.f;
      }
      rmsPtNum[0] = rmsPtNum[1] = 0;
      epiFoeAngle[0] = epiFoeAngle[1] = 0.f;
    }
    void reset() { validFrames=0; validStraightFrameNum=0; epiQualityFrameNum=0;}

    bool valid;
    bool rmsValid; // RoadModelStorage valid
    bool epiValid;
    WM_EM_status status;
    float conf;
    float confT;
    float confR;
    float straightScore;
    Float::MEmath::Mat<3,3,float> R;
    Float::MEmath::Vec<3,float> t;
    float ypr[3];
    float yaw;
    float pitch;
    float epiErr; // epipolar-err
    float epiErrAng;
    float epiCov[3];
    float epiInvalidPercent;
    int validFrames; // TODO: change to validFrameNum
    int validStraightFrameNum;
    int epiQualityFrameNum;
    float epiQuality;
    unsigned int rmsPtNum[2]; // #pts in RoadModelStorage, 0: all, 1: inliers
    float rmsPtNumMean;
    PixelLm2_f foeRectLm2; // foe via LS on RoadModelStorage points
    float epiFoeAngle[2];
    float epiCovEma;
    float rmsPtNumEma;
    float epiDiffEma;
    float epiErrEma;
  };

  struct RmData {
    RmData() : valid(false), validCov(false), status(RM_MODEL_INVALID), numOfInliers(0),
               planeIdx(0), message(WorldModel::VRM::VALID_PLANE), d(0.f), Zstart(0.f), Zend(0.f), roll(0.f),
               pitch(0.f), pitchDiff(0.f), crownZ(0.f), crownAngDiff(0.f), crownAngDiff1(0.f), crownDistDiff1(0.f),
               crownAngDiffMean(0.f), crownAngDiffVar(0.f), crownAngDiffMin(0.f), crownAngDiffMax(0.f),
               rmResidual(0.f), AtA(0.f), nAtA(0.f), cov(0.f)
    {
      N[0] =  N[1] = N[2] = 0.f;
      for (int i = 0; i < CROWN_SAMP_NUM; ++i) {
        crownX[i] = 0.f;
        crownAng[i] = 0.f;
        crownAngStd[i] = 0.f;
      }
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
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
    WorldModel::VRM::PlaneType message;
    float N[3];
    float d;
    float Zstart;
    float Zend;
    float roll;
    float pitch;
    float pitchDiff;
    float crownZ;
    float crownX[CROWN_SAMP_NUM];
    float crownAng[CROWN_SAMP_NUM];
    float crownAngStd[CROWN_SAMP_NUM];
    float crownAngDiff;
    float crownAngDiff1;
    float crownDistDiff1;
    float crownAngDiffMean;
    float crownAngDiffVar;
    float crownAngDiffMin;
    float crownAngDiffMax;
    float resConfMean[e_CALIB_DOF_NUM];
    float resConfMeanStringent[e_CALIB_DOF_NUM];
    int residualPtNum[e_CALIB_DOF_NUM];
    float residualMean[e_CALIB_DOF_NUM];
    float residualVar[e_CALIB_DOF_NUM];
    float residualMin[e_CALIB_DOF_NUM];
    float residualMax[e_CALIB_DOF_NUM];
    float rmResidual;
    WorldModel::EMat3 AtA; // MEtypes::EfficientMatrix<float, 3, 3>
    WorldModel::EMat3 nAtA;
    WorldModel::EMat3 cov;
  };

  struct PlaneConfData { // experimental TODO: delete if not helpful
    PlaneConfData() : rmValidCov(false), rmStorageSize(0), epiErrAng(0.f),
                      epiCovXX(0.f), epiCovYY(0.f),
                      pitchDiff(0.f), crownAngDiff(0.f), rmResidual(0.f), rmCov(0.f)
    {
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        residualPtNum[i] = 0;
        residualMean[i] = 0.f;
      }
    }
    void reset() {
      rmValidCov    = false;
      rmStorageSize = 0;  
      epiErrAng     = 0.f;  
      epiCovXX      = 0.f;  
      epiCovYY      = 0.f;  
      pitchDiff     = 0.f;  
      crownAngDiff  = 0.f;  
      rmResidual    = 0.f;  
      rmCov         = 0.f;  
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        residualPtNum[i] = 0;
        residualMean[i]  = 0.f;
      }
    }
    void fill(const EmData &em, const RmData &rm) {
      rmValidCov    = rm.validCov;
      rmStorageSize = em.rmsPtNum[0];
      epiErrAng     = em.epiErrAng;
      epiCovXX      = em.epiCov[0];
      epiCovYY      = em.epiCov[1];
      pitchDiff     = rm.pitchDiff;
      crownAngDiff  = rm.crownAngDiff;
      rmResidual    = rm.rmResidual;
      rmCov         = rm.cov;
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        residualPtNum[i] = rm.residualPtNum[i];
        residualMean[i]  = rm.residualMean[i];
      }
    }
    PlaneConfData& operator=(const PlaneConfData &that) {
      rmValidCov    = that.rmValidCov;
      rmStorageSize = that.rmStorageSize;
      epiErrAng     = that.epiErrAng;
      epiCovXX      = that.epiCovXX;
      epiCovYY      = that.epiCovYY;
      pitchDiff     = that.pitchDiff;
      crownAngDiff  = that.crownAngDiff;
      rmResidual    = that.rmResidual;
      rmCov         = that.rmCov;
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        residualPtNum[i] = that.residualPtNum[i];
        residualMean[i]  = that.residualMean[i];
      }
      return *this;
    }
    PlaneConfData& operator+=(const PlaneConfData &that) {
      rmValidCov     = rmValidCov || that.rmValidCov;
      rmStorageSize += that.rmStorageSize;
      epiErrAng     += that.epiErrAng;
      epiCovXX      += that.epiCovXX;
      epiCovYY      += that.epiCovYY;
      //pitchDiff   += that.pitchDiff;
      crownAngDiff  += that.crownAngDiff;
      rmResidual    += that.rmResidual;
      rmCov         += that.rmCov;
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        residualPtNum[i] += that.residualPtNum[i];
        residualMean[i]  += that.residualMean[i];
      }
      return *this;
    }
    PlaneConfData& operator/=(int n) {
      rmStorageSize /= n;
      epiErrAng     /= n;
      epiCovXX      /= n;
      epiCovYY      /= n;
      //pitchDiff   /= n;
      crownAngDiff  /= n;
      rmResidual    /= n;
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        residualPtNum[i] /= n;
        residualMean[i]  /= n;
      }
      return *this;
    }

    bool rmValidCov;
    int rmStorageSize;
    int residualPtNum[e_CALIB_DOF_NUM];
    float epiErrAng; // residual of LS epi-lines to foe
    float epiCovXX;  // cov of LS epi-lines to foe
    float epiCovYY;  // cov of LS epi-lines to foe
    float pitchDiff;
    float crownAngDiff;
    float rmResidual; // residual from RM internal LS
    float residualMean[e_CALIB_DOF_NUM]; // residual of plane components (neri's calc)
    WorldModel::EMat3 rmCov; // cov from RM internal LS
  };

  struct PlaneData {
    PlaneData() : valid(false), roll(0), camh(0), location(0),
                  startFrame(0), endFrame(0) {}

    bool valid;
    float roll;
    float camh;
    float location;
    int startFrame;
    int endFrame;
    PlaneConfData confData;
  };

  struct VehicleData {
    VehicleData() : radius(0.f), yawRate(0.f), dt(0.f), ds(0.f), trajLength(0.f), trajLengthAbs(0.f),
    trajTime(0.f), drivingTime(0.f), reverseGear(false), vehicleKneeling(false), accel(0.f), speed(0.f),
    validFrame(false), validFrameNum(0), validDistance(0.f), validTime(0.f), vehicleRoll(0.f),
    speedHighPrecisionValid(false), speedHighPrecision(0.f), speedPrecisionError(0.f),
    dynamicCameraHeightValid(false), dynamicCameraHeight(0.f), dynamicCameraHeightDeltaValid(false),
    dynamicCameraHeightDelta(0.f), dynamicCameraHeightChangeActiveValid(false), dynamicCameraHeightChangeActive(false),
    dynamicCameraHeightDeltaAvailable(false) {}
    void reset() { radius = 0.f; yawRate = 0.f; dt = 0.f; ds = 0.f; trajLength = 0.f; trajLengthAbs = 0.f;
      trajTime = 0.f; drivingTime = 0.f; reverseGear = false; vehicleKneeling = false; accel = 0.f; speed = 0.f;
      validFrame = false; validFrameNum = 0; validDistance = 0.f; validTime = 0.f; 
      speedHighPrecisionValid = false; speedHighPrecision = 0.f; speedPrecisionError = 0.f;
      dynamicCameraHeightValid = false; dynamicCameraHeight = 0.f; dynamicCameraHeightDeltaValid = false;
      dynamicCameraHeightDelta = 0.f; dynamicCameraHeightChangeActiveValid = false; dynamicCameraHeightChangeActive = false;
      dynamicCameraHeightDeltaAvailable = false;} 

    float radius;
    float yawRate;
    float dt;
    float ds;
    float trajLength; // TODO: change to totalDistance;
    float trajLengthAbs; // TODO: change to totalDistanceAbs;
    float trajTime; // TODO: change to totalTime;
    float drivingTime; 
    bool reverseGear; // TODO: remove
    bool vehicleKneeling; // TODO: remove
    float accel;
    float speed;

    bool validFrame;
    int validFrameNum;
    float validDistance;
    float validTime;

    // Zfas
    float vehicleRoll;

    // Kafas
    bool speedHighPrecisionValid;
    float speedHighPrecision;
    float speedPrecisionError;

    bool dynamicCameraHeightValid; // TODO: remove
    float dynamicCameraHeight; // TODO: remove

    bool dynamicCameraHeightDeltaValid;
    float dynamicCameraHeightDelta;
    bool dynamicCameraHeightChangeActiveValid;
    bool dynamicCameraHeightChangeActive;
    bool dynamicCameraHeightDeltaAvailable;
  };

  struct ConvValidData {
    ConvValidData() : validFrame(false), validFrameNum(0), conv(false), convNum(0), convLastFrame(-1),
		      nonConvReason(0), inRange(false), progress(0), quality(0),
		      totalConfidence(0.f), totalConfGrade(0), validDistance(0.f), validTime(0.f), status(e_CALIB_UNDEFINED),
		      coreStatus(e_CORE_INIT), coreError(e_CALIB_OK) {
      for (int i = 0; i < CONF_BIT_NUM; ++i) {
        confidence[i] = 0.f;
      }
    }
    ConvValidData(ConvValidData& d) : validFrame(d.validFrame), validFrameNum(d.validFrameNum), 
				      conv(d.conv), convNum(d.convNum), convLastFrame(d.convLastFrame), nonConvReason(d.nonConvReason), inRange(d.inRange),
    progress(d.progress), quality(d.quality), totalConfidence(d.totalConfidence), totalConfGrade(d.totalConfGrade), 
    validDistance(d.validDistance), validTime(d.validTime), status(d.status), coreStatus(d.coreStatus),
    coreError(d.coreError) {
      for (int i = 0; i < CONF_BIT_NUM; ++i) {
        confidence[i] = d.confidence[i];
      }
    }
    void reset() { validFrame = false; validFrameNum = 0; conv = false; nonConvReason = 0; inRange=false; progress = 0;
      quality = 0; totalConfidence = 0.f; totalConfGrade = 0; validDistance = 0.f; validTime = 0.f; status = e_CALIB_UNDEFINED; coreStatus = e_CORE_INIT; coreError = e_CALIB_OK;
      for (int i = 0; i < CONF_BIT_NUM; ++i) {
        confidence[i] = 0.f;
      }
    }
    ConvValidData& operator=(const ConvValidData& d)  { validFrame = d.validFrame; validFrameNum = d.validFrameNum; conv = d.conv;
      nonConvReason = d.nonConvReason; inRange=d.inRange; progress = d.progress; quality = d.quality; totalConfidence = d.totalConfidence;
      convNum = d.convNum; convLastFrame = d.convLastFrame;
      totalConfGrade = d.totalConfGrade; validDistance = d.validDistance; validTime = d.validTime; status = d.status;
      coreStatus = d.coreStatus; coreError = d.coreError;
      for (int i = 0; i < CONF_BIT_NUM; ++i) {
        confidence[i] = d.confidence[i];
      }
      return *this;
    }

    void updateTotalConf();
    bool isSessionEnded();

    bool validFrame;
    int validFrameNum;
    bool conv;
    int convNum;
    int convLastFrame;
    int nonConvReason;
    bool inRange;
    int progress;
    int quality;
    float confidence[CONF_BIT_NUM];
    float totalConfidence;
    int totalConfGrade;
    float validDistance;
    float validTime;
    CalibStatus status;
    CalibCoreStatus coreStatus;
    CalibStatus coreError;
  };


  struct AlgoData {
    AlgoData() : img(nullptr), pauseReason(0), sessionNum(1), enforcingFirstValid(false), quickMode(false), 
    wmFoeReset(e_WMRESET_NONE),resetMask(e_RESET_NONE) {}
    void reset() {
      pauseReason = 0; enforcingFirstValid = false; wmFoeReset = e_WMRESET_NONE; total.reset();
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        single[i].reset();
      }
    }

    ConvValidData single[e_CALIB_DOF_NUM]; // TODO: is it needed?
    ConvValidData total;
    const Prep::SafeImg* img;
    int pauseReason;
    int sessionNum;
    bool enforcingFirstValid;
    bool quickMode;
    WM_ResetSignal wmFoeReset;
    int resetMask;
  };

  struct TargetData {
    TargetData() : img(nullptr), level(-1) {}

    const Prep::SafeImg* img;
    int level;
  };

  struct VehicleToCam {
    // TODO: add ypr2R which corresponds to sensor angles
    VehicleToCam() : focalLm2(1.f), R(Float::MEmath::identity<3, double>()),
    camh(0.0) {
      angles[0] = angles[1] = angles[2] = 0.0;
      foeLm2.X() = foeLm2.Y() = 0.f;
    }

    VehicleToCam(const VehicleToCam& v);
    VehicleToCam& operator=(const VehicleToCam& rhs); 
    VehicleToCam& operator+=(const VehicleToCam& that);
    VehicleToCam& operator-=(const VehicleToCam& that);

    void reset();
    void update(const double angles[3]);
    void update(double val, int idx);
    void update(const double angles[3], const double camh);
    void update(const Float::MEmath::Mat<3,3,double>& Rin, double camhIn);
    void update(const PixelLm2_f foeLm2, const double roll, const double camh);
    void rotate();

    float focalLm2;
    Float::MEmath::Mat<3,3,double> R;
    double angles[3];
    double camh; // [m] conventional camh (relative to road)
    PixelLm2_f foeLm2; // foe pixel on distorted image relative to current image origin (delta from etc)
  };

  VehicleToCam operator+(const VehicleToCam& v0, const VehicleToCam& v1); 
  VehicleToCam operator-(const VehicleToCam& v0, const VehicleToCam& v1); 
  
  struct ResultBook {
    void focalLm2(float focalLm2);

    VehicleToCam curr;
    VehicleToCam prevConv;
    VehicleToCam prevWmReset;
    VehicleToCam deltaWmReset;
    VehicleToCam cams[CameraInfo::e_NUM_OF_CAM_INSTANCES];
  };

  struct WmbcData {
    void reset() { vehicle.reset(); em.reset(); algo.reset(); }

    ValidParams validParams;
    ConvParams convParams;
    MetaParams metaParams;
    ConfidenceParams confParams[CONF_BIT_NUM];
    CameraData camera;
    EmData em;
    RmData rm;
    PlaneData plane;
    VehicleData vehicle;
    AlgoData algo;
    TargetData target;
    ResultBook results;
  };

  struct ConvergenceDataStatistics {
    ConvergenceDataStatistics() : distSqr(0.f), sum(0.f), distSqrSum(0.f), sumSqr(0.f), mean(0.f), var(0.f), varSf(0.f) {}
    void reset() {distSqr = 0.f; sum = 0.f; distSqrSum = 0.f; sumSqr = 0.f; mean = 0.f; var = 0.f; varSf = 0.f;}

    float distSqr;
    float sum;
    float distSqrSum;
    float sumSqr; // TODO: remove
    float mean;
    float var;
    float varSf;
  };

  struct DebugShowData { // TODO: organize this shit!
    // DebugShowData () {}
    // virtual ~DebugShowData() {}

    // Stationless target data
    // bool validTargets;
    // unsigned int spNum; // number of saddle pts in a single target
    // int xTargets[4]; // order: bottom-left, top-left, bottom-right, top-right
    // int yTargets[4];
    // int xSearch[4];
    // int ySearch[2];
    Fix::MEimage::Rect targetSearchWindow[2];
    float startDistanceToTarget;
    float targetWidthWorld;
    float targetWidthImage;

    // general
    int validFramesNumIndividual[e_CALIB_DOF_NUM];
    int validFramesNumIndividualPercentage[e_CALIB_DOF_NUM];
    bool validFrameIndividual[e_CALIB_DOF_NUM];
    bool singleConverged[e_CALIB_DOF_NUM];
    float currDistanceToTarget;
    float trajLength;
    bool isEndTraj;
    int validFramesVehicle;
    int validFramesVehiclePercentage;
    int progress;

    ConvValidData singleStat[e_CALIB_DOF_NUM];

    // SPC
    int sessionNumber;
    int maxAttempts;

    DebugShowData() : startDistanceToTarget(0.f), targetWidthWorld(0.f), targetWidthImage(0.f),
                      currDistanceToTarget(0.f), trajLength(0.f), isEndTraj(false),
                      validFramesVehicle(0), validFramesVehiclePercentage(0), progress(0),
                      sessionNumber(0), maxAttempts(0) {
      for (int i = 0; i < 2; ++i) {
        targetSearchWindow[i] = Fix::MEimage::Rect();
      }
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        validFramesNumIndividual[i] = 0;
        validFramesNumIndividualPercentage[i] = 0;
        validFrameIndividual[i] = false;
        singleConverged[i] = false;
      }
    }

    DebugShowData(DebugShowData& d) :
      startDistanceToTarget(d.startDistanceToTarget),
      targetWidthWorld(d.targetWidthWorld),
      targetWidthImage(d.targetWidthImage),
      currDistanceToTarget(d.currDistanceToTarget),
      trajLength(d.trajLength),
      isEndTraj(d.isEndTraj),
      validFramesVehicle(d.validFramesVehicle),
      validFramesVehiclePercentage(d.validFramesVehiclePercentage),
      progress(d.progress),
      sessionNumber(d.sessionNumber),
      maxAttempts(d.maxAttempts) {
      for (int i = 0; i < 2; ++i) {
        targetSearchWindow[i] = d.targetSearchWindow[i];
      }
      for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        validFramesNumIndividual[i] = d.validFramesNumIndividual[i];
        validFramesNumIndividualPercentage[i] = d.validFramesNumIndividualPercentage[i];
        validFrameIndividual[i] = d.validFrameIndividual[i];
        singleConverged[i] = d.singleConverged[i];
      }
    }
  };

} // namespace WMBC

#endif // WMBC_TYPES_H_
