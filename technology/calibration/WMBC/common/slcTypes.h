/*
 * slc.h
 *
 *  Created on: Jul 03, 2016
 *      Author: urilo
 */

#ifndef _WMBC_SLC_TYPES_H_
#define _WMBC_SLC_TYPES_H_

// #include "utilities/saddlePoints/common/types.h"
#include "technology/calibration/WMBC/common/wmbcTypes.h"

namespace WMBC {

  // typedef MEtypes::ptr_vector<SaddlePoints::Types::SPInfo> SPInfoVec;
  // typedef MEtypes::ptr_vector<Float::MEmath::Vec<2, double> > SPVec;

  // SLC consts
  enum BottomLeftSquare { WHITE = 0, BLACK };
  const int TO_LEVEL_LM2[3] = {4, 2, 1};
  const float FROM_LEVEL_LM2[3] = {0.25f, 0.5f, 1.f};
  const int SLC_MAX_TARGET_NUM = 2;
  static const int SLC_RUN_LEVEL = -2;
  const unsigned int MAX_SP_CAND_NUM = 300;
  const unsigned int MAX_SP_NUM = 50; // used by egomotion
  const int SLC_MAX_IM_HEIGHT = 960;                                                                                                                                                          
  const int SLC_MAX_IM_WIDTH = 1280;
  const float IM_TARGET_WIDTH_TH = 100.f;

  const int MAX_FRAME_NUM = 70;                                                                                                                                                               
  const int SP_X_SEARCH_WIN = 70; // px, size of saddle pt search window in x
  const int SLC_SP_Y_SEARCH_WIN_DEFAULT = 90; // px, size of saddle pt search window in y
  const int CALC_FOE_FRAME = 60; // TODO: discard
  const int MAX_SP_NUM_IN_TARGET =  25;
  const int MAX_MISSING_SP_ALLOWED = 2;

  const float SLC_SP_X_SEARCH_WIN_DEFAULT = 50.f;
  const int SLC_MIN_VALID_FRAMES_DEFAULT = 6;
  const int SLC_MAX_RUN_FRAMES_DEFAULT = 70;
  const float SLC_END_DIST_FACTOR_DEFAULT = 2;
  const int SLC_HFRAMES_CAMH_DEFAULT = 10;
  const float SLC_MAX_HEIGHT_DIFF = 100.f;
  const float SLC_QUALITY_MAX_FRAMES[e_CALIB_DOF_NUM] = {35, 35, 10, 14};


  // stationlessTargetFinder consts
  const int MAX_POLE_CANDS = 10;
  const unsigned int MAX_CANDS_PTS = 300;
  //const int MAX_PATTERN_PTS = 30;
  const int TARGET_POOL_SIZE = 16;
  enum Side {LEFT, RIGHT, SIDE_NUM};
  enum PatternMode {PATTERN_MODE_SUM, PATTERN_MODE_MEAN};

  // distance estimator consts
  const float DE_PROC_NOISE = 0.01f;
  const float DE_MEAS_NOISE = 0.01f;

  // target img pos estimator consts
  const float TE_PROC_NOISE = 1.f;
  const float TE_MEAS_NOISE = 1.f;

  const int ROLL_INVALID_SLC = (e_LEFT_TARGET_NOT_FOUND |
                                e_RIGHT_TRAGET_NOT_FOUND
                               );

  struct SlcConvParams {
    int minValidFrames;
    int maxRunFrames;
    float endDistanceFactor;
    int validHorFramesForCamh;
    int minValidFramesCamh;
    float maxTravelDist;
  };

  struct TargetParams {
    float height; // [m] height of lowest SP from ground
    float width; // [m] distance between target centers
    float rect[2]; // [m] dimension of single rectangle in target width x height
    int dim[2]; // dimension of target rectangles: rows x columns
    int spNumInTarget; // number of saddle pts in a single target
    int bottomLeftSquare[SLC_MAX_TARGET_NUM]; // color of bottom left rectangle
    float patternMatchingThreshold; // threshold of saddle point pattern matching
    int shiftWinX;
    int searchWinRadX;
    int searchWinRadY;
    bool useCalibShiftWin;
    float camhMaxDistance; // max distance from targets where camera height calc is done
    float camhMinDistance;
    float maxDistance; // [m] max distance from targets (start distance)
    float minDistance; // [m] min distance from targets (stop distance)
    float maxHeightDiff; // [px] max difference between target (vertical) length in pixels
    float cameraHeightGuess; // [m] input height guess (for positioning search window)
    int maxSuccNonValidTgtFrmNum;
  };

  struct TFParams {                                                                                                                                                                         
    bool useDebugPrints;                                                                                                                                                                    
    int spNumInTarget;
    //BottomLeftSquare bottomLeftSquare;                                                                                                                                                    
    int bottomLeftSquare;                                                                                                                                                                   
    int targetsLevel; // level of _imgTargets
    int maxMissingSpAllowed;
                                                                                                                                                                                              
    TFParams() {                                                                                                                                                                            
      useDebugPrints = false;                                                                                                                                                               
      spNumInTarget = MAX_SP_NUM_IN_TARGET;
      bottomLeftSquare = WHITE; 
      targetsLevel = -1;
      maxMissingSpAllowed = MAX_MISSING_SP_ALLOWED;
    }                                                                                                                                                                                       
                                                                                                                                                                                              
    TFParams(bool useDebugPrints, int spNumInTarget, int bottomLeftSquare, int targetsLevel) {
      this->useDebugPrints = useDebugPrints;                                                                                                                                                
      this->spNumInTarget = spNumInTarget;
      this->bottomLeftSquare = bottomLeftSquare;                                                                                                                                            
      this->targetsLevel = targetsLevel;
      this->maxMissingSpAllowed = MAX_MISSING_SP_ALLOWED;
    }
  };

  struct TargetCandInfo { 
    int ptNum;
    float imSquareSize;
    int score;
    int idx[MAX_SP_NUM_IN_TARGET]; // idx[i] = n with i being logical order from lowest (zero based), n being index in spVecInfo container (_poles)

    TargetCandInfo() : ptNum(0), imSquareSize(0), score(0) {
      for (int i = 0; i < MAX_SP_NUM_IN_TARGET; ++i) {
        idx[i] = -1;
      }
    }                                                                                                                                                                                       
  };

  struct TargetDrawInfo { // transmitted to show and protocol
    bool valid;
    float distance;
    Float::MEgeo::Rect imCorners[2];

    TargetDrawInfo() : valid(false), distance(-1.f) {
      for (int i = 0; i < 2; ++i) {
        imCorners[i].clear();
      }
    }
    TargetDrawInfo(TargetDrawInfo& t) : valid(t.valid), distance(t.distance) {
      for (int i = 0; i < 2; ++i) {
        imCorners[i] = t.imCorners[i];
      }
    }
  };

  // struct SlcDebugShowData : DebugShowData {
  //   SPInfoVec *sp;

  //   SlcDebugShowData() : DebugShowData(), sp(NULL) {}
  //   // SlcDebugShowData(const DebugShowData *dsd) : DebugShowData(), sp(nullptr) {}
  //   // SlcDebugShowData(const DebugShowData &dsd) : DebugShowData(), sp(nullptr) {}
  //   SlcDebugShowData(const DebugShowData &dsd)  {}
  //   virtual ~SlcDebugShowData() { delete sp; sp = NULL;}
  // };

  struct CamhData { // data for calculation of cam height
    int frame;
    float foer[2]; // [px] lm2 on rectified im
    // float foed[2]; // [px] lm2 on distorted im
    float distance; // [m] estimated distance to target plane
    float distanceMeas; // [m] sf measurement distance to target plane
    float invImageTargetWidth; // [px^-1] lateral distance between targets on rectified image inveresed
    //float targetCenter[2]; // [px] center of two targets on rectified image
    float targetCornersY[4];
    float camh;
    float camhErr;
    bool valid;
    bool chosen; // used for camh calc

    CamhData() : frame(0), distance(0.f), distanceMeas(0.f), invImageTargetWidth(0.f), camh(0.f), camhErr(0.f), valid(false), chosen(false) {
      for (int i = 0; i < 2; ++i) {
        foer[i] = 0.f;
        //targetCenter[i] = 0.f;
        targetCornersY[i] = 0.f;
        targetCornersY[i+2] = 0.f;
      }
    }
  };

  typedef MEtypes::ptr_vector<CamhData> CamhDataVec;
  typedef MEtypes::ptr_vector<CamhData>::iterator CamhDataIter;    

  } // namespace WMBC

#endif // _WMBC_SLC_TYPES_H_

