/*
 * stationlessTargetFinder.cpp
#include "technology/mobilib/std/me_math.h"
 *
 *  Created on: Nov 19, 2015
 *  Author: urilo
 */

#include "stationlessTargetFinder.h"
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "SLC_utils.h"
#include <stdlib.h>  
#include "technology/mobilib/std/me_math.h"
#include "utilities/saddlePoints/saddlePoints_API.h"

#ifdef MEwin
#include "technology/calibration/WMBC/i386/stationlessTargetFinderDebugger.h"
#define DEBUGGER(foo) do { if (_debugger!=NULL) _debugger->foo; } while (0)
#define DEBUG_LOG_CLR(arg, c) if ((_useDebugPrints) && (c >=0) && (c < 8)) {std::cout << "\033[3" << c << "m" << arg << "\033[0m";}
#else
#define DEBUGGER(foo)
#define DEBUG_LOG_CLR(arg, c)
#endif

using namespace Float::MEmath;

namespace WMBC {

  // using namespace Types;  

  bool bestFitLineSymSp(SPInfoVec &sp, float *coeff);
  void ransacBestLineSp(SPInfoVec &sp, SPInfoVec &spBest, float threshold = 0.5);
  void sortSpByY(SPInfoVec &sp);
  bool matchPattern(float *arr, int *mask, int len, int *idx, int mode);
  int matchPatternCyclic(float *arr, int *mask, int len, int zeroLen, int mode);
  void rotateIntArray(int *arr, int len, int step);
  bool matchPatternSingle(float *arr, int *mask, int len, int matchLen, int mode);
  void debugPrintPatternMatch(bool isMatch, float sum, float *arr, int *mask, int len, int matchLen);

  //StationlessTargetFinder::StationlessTargetFinder(TFParams &params, OriginData *origins) :
  StationlessTargetFinder::StationlessTargetFinder() :
    _tmpSp(MAX_SP_CAND_NUM),
    _candsSp(MAX_SP_CAND_NUM),
    _imageTargetWidth(1.0f),
    //_params(params),
    _img(NULL),
    // _squareSize(8.f),
    //_origins(origins),
    _origins(NULL),
    _debugger(NULL)
  {    
    for (int i = LEFT; i < SIDE_NUM; ++i) {
      _poles[i].reserve(MAX_SP_CAND_NUM);
      _squareSize[i] = Debug::Args::instance().getStickyValue("-sSlc-iniSqSize", 8.f);
    }
    _useDebugPrints = Debug::Args::instance().existsParameter("-sSLCtargetDebugPrint"); //_params.useDebugPrints;
    _xBinRad = Debug::Args::instance().getStickyValue("-sSLCxBinRad", 6);
    _tfRansacTh = Debug::Args::instance().getStickyValue("-sSLCtfRansacTh", 2.f);
  }

  void StationlessTargetFinder::setDebugger() {
#ifdef MEwin
    _debugger = new StationlessTargetFinder_Debugger(this);
    DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_INIT));
#endif
  }

  // void StationlessTargetFinder::init(const Fix::MEimage::Image *img) {
  void StationlessTargetFinder::init(const Prep::SafeImg *img) {
    _img = img;
  }
    

  void StationlessTargetFinder::reset() {
    _tmpSp.clear();
    _candsSp.clear();
    for (int i = LEFT; i < SIDE_NUM; ++i) {
      _poles[i].clear();
      _polesPtNum[i] = 0;
    }
  }

  bool StationlessTargetFinder::run(SPInfoVec &rawSp) {
    DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_START));
    DEBUGGER(setInputSp(rawSp));
    
    reset();
    // TAC2_DTIMER(tfTimer);
    _tmpSp = rawSp;
    bool useLm2 = true;
    SaddlePoints::API::refine(_tmpSp, FROM_LEVEL_LM2[2], useLm2);
    // _distortion->undistort(_tmpSp);
    rectify(_tmpSp);
    DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_RAW_SP));

    // if ((int)rawSp.size() < 2*_params.spNumInTarget) {
    //   DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_END));
    //   return false;
    // }
    
    // TAC2_STIMER(tfTimer);
    bool ok = findPolesCands();
    // TAC2_ETIMER(tfTimer, "[TF::run::findPolesCands]");
    DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_POLES_CANDS));
    if (!ok) {
      DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_END));
      return ok;
    }

    // TAC2_STIMER(tfTimer);
    ok = findTwoPoles();
    // TAC2_ETIMER(tfTimer, "[TF::run::findTwoPoles]");
    DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_POLES_CHOSEN));
    if (!ok) {
      DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_END));
      return ok;
    }

    // TAC2_STIMER(tfTimer);
    for (int i = LEFT; i < SIDE_NUM; ++i) {
      // eliminateOutliersBySign(i);
      // eliminateOutliersByDistance(i);

      estimateSquareSize(i);
      DEBUG_LOG_CLR("[run] sqsz, side: " << i << ", sq: " << _squareSize[i] << std::endl, 5);
      bool ok = finalizeTarget(i);
      if (!ok) {
        DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_END));
        return ok;
      }
    }
    // TAC2_ETIMER(tfTimer, "[TF::run::eliminateOutliers]");

    rawSp.clear();
    for (int i = LEFT; i < SIDE_NUM; ++i) {
      int ptNum = _poles[i].size();
      int validPtNum = 0;
      
      for (int j = 0; j < ptNum; ++j) {
        if (me_abs(_poles[i][j].score) > 0.1 && validPtNum < _params.spNumInTarget) {
          rawSp.push_back(_poles[i][j]);
          validPtNum++;
        }
      }
    }

    // DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_POLES_CLEAR));
    DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_FINAL_TARGETS));
    DEBUGGER(updateStage(StationlessTargetFinder_Debugger::e_END));

    return ok;
  }

  bool StationlessTargetFinder::findPolesCands() {
    // make histogram in x with ovelapping bins
    const int MAX_XBIN_NUM = 1280;
    // _xBinRad = std::max(4, ceil(0.5*_squareSize));
    int xBinNum = 1280 / _xBinRad;

    unsigned int xbins[MAX_XBIN_NUM];
    int spnum = _tmpSp.size();
    // int origin_x = (*_img)->origin().x;
    // int origin_x = _distortion->imgRectOriginLm2[0]; // _distortion -> _origins
    int origin_x = (int)_origins->rectLm2.X();

    for (int i = 0; i < MAX_XBIN_NUM; ++i) {
      xbins[i] = 0;
    }

    for (int i = 0; i < spnum; ++i) {
      int x = _tmpSp[i].x + origin_x; 
      int j = (int) x / _xBinRad;
      // assert(j >= 0 && j < xBinNum);
      
      if (j >= 0 && j < xBinNum) {
        xbins[j] += 1;
        if (j > 0) {
          xbins[j-1] += 1;
        }
      }
    }

#ifdef MEwin
    printXbins(xbins, xBinNum, _xBinRad);
#endif
    
    
    // find which of the bins contains enough points on a straight lines

    const int MAX_PT_NUM = 100;
    // float ransacTh = 2;
    static int maxMissingSpAllowed = Debug::Args::instance().getStickyValue("-sSlc-maxMissing", MAX_MISSING_SP_ALLOWED);
    _params.maxMissingSpAllowed = maxMissingSpAllowed;
    const int unsigned MIN_PTS_BIN = _params.spNumInTarget - _params.maxMissingSpAllowed;
    const int MIN_PTS_RANSAC = MIN_PTS_BIN;

    for (int i = 0; i < xBinNum; ++i) {
      if (xbins[i] >= MIN_PTS_BIN) {
        SPInfoVec tmpPoleCand(MAX_PT_NUM), tmpPoleCandRansac(MAX_PT_NUM); // TODO: check if MAX_PT_NUM is necessary
        for (int j = 0; j < spnum; ++j) {
          if (!tmpPoleCand.isFull() && _tmpSp[j].x + origin_x >= _xBinRad * i && _tmpSp[j].x + origin_x < _xBinRad * (i+2)) {
            tmpPoleCand.push_back(_tmpSp[j]);
          }
        }

        // SaddlePoints::API::subpixelInterpolation(tmpPoleCand, FROM_LEVEL_LM2[1]);
        // _distortion->undistort(tmpPoleCand);
#ifdef MEwin
        debugPrintPtsShort(tmpPoleCand, "[targetFinder::findPolesCands] tmpPoleCand distorted", false, false);
        debugPrintPtsShort(tmpPoleCand, "[targetFinder::findPolesCands] tmpPoleCand", false, true);
#endif
        // ransacBestLineSp(tmpPoleCand, tmpPoleCandRansac, _tfRansacTh); 
        ransacBestLineSymSp(tmpPoleCand, tmpPoleCandRansac, _tfRansacTh); 
#ifdef MEwin
        debugPrintPtsShort(tmpPoleCandRansac, "[targetFinder::findPolesCands] tmpPoleCandRansac", false, true);
        DEBUG_LOG_CLR("-------------------------------------------------------------", 0);
#endif

        int ptNum = tmpPoleCandRansac.size();
        if (ptNum >= MIN_PTS_RANSAC) {
          for (int j = 0; j < ptNum && !_candsSp.isFull(); ++j) {
            _candsSp.push_back(tmpPoleCandRansac[j]); // TODO: make pole candidates instead of  aggregating all pts to a single container!!
          }
        }
      }
    }

    return true; //(_candsSp.size() >= 34); // TODO: const
  }

  void ransacBestLineSp(SPInfoVec &sp, SPInfoVec &spBest, float threshold) {
    int testNum = 1000; // TODO: make const or function parameter? use nchoosek to get seder godel?
    int inliersNum = -1;
    const int SAMPLE_NUM = 3;
    int ptNum = sp.size();
    SPInfoVec spTmp(ptNum);

    for (int i = 0; i < testNum; ++i) {
      int idx[SAMPLE_NUM];
      idx[0] = rand() % ptNum;
      idx[1] = idx[0];
      idx[2] = idx[0];
      while (idx[1] == idx[0]) {
        idx[1] = rand() % ptNum;
        while (idx[2] == idx[0] || idx[2] == idx[1]) {
          idx[2] = rand() % ptNum;
        }
      }

      float coeff[2];
      bool ok = bestFitLineSymSp(sp, coeff);
      if (!ok) {
        continue;
      }
      float res;
      int newInliersNum = 0;
      for (int j = 0; j < ptNum; ++j) {
        float x = (float) sp[j].x;
        float y = (float) sp[j].y;

        res = x*me_sinf(coeff[1]) - y*me_cosf(coeff[1]);
        res = fabsf(fabsf(res) - fabsf(coeff[0]));

        if (res < threshold) {
          spTmp.push_back(sp[j]);
          newInliersNum++;
        }
      }

      if (inliersNum < newInliersNum) {
        spBest = spTmp;
      }

      spTmp.clear();
    }
  }

  bool bestFitLineSymSp(SPInfoVec &sp, float *coeff) {
    int ptNum = sp.size();
    float xMean = 0.0f;
    float yMean = 0.0f;
    // float xShift = 

    for (int i = 0; i < ptNum; ++i) {
      xMean += sp[i].x;
      yMean += sp[i].y;
    }
    xMean /= (ptNum > 0) ? ptNum : 1;
    yMean /= (ptNum > 0) ? ptNum : 1;

    float a = 0;
    float b = 0;

    for (int i = 0; i < ptNum; ++i) {
      float x = sp[i].x - xMean;
      float y = sp[i].y - yMean;
      a += 2*x*y;
      b += x*x - y*y;
    }

    coeff[1] = 0.5 * me_atan2f(a, b); // need std::atan2 or #include <math.h> or both
    coeff[0] = -xMean * me_sinf(coeff[1]) + yMean * me_cosf(coeff[1]);

    return true;
  }

  bool StationlessTargetFinder::findTwoPoles() {
    // find average
    int ptNum = _candsSp.size();
    float xMean = 0;
    for (int i = 0; i < ptNum; ++i) {
      xMean += _candsSp[i].x;
    }
    xMean /= (ptNum > 0) ? ptNum : 1;

    // divide for left and right
    float xLeftMean = 0.0f;
    float xRightMean = 0.0f;
    int nLeft = 0;
    int nRight = 0;

    for (int i = 0; i < ptNum; ++i) {
      if (_candsSp[i].xSub <= xMean) {
        _poles[LEFT].push_back(_candsSp[i]);
        xLeftMean += _candsSp[i].xSub;
        nLeft++;
      } else {
        _poles[RIGHT].push_back(_candsSp[i]);
        xRightMean += _candsSp[i].xSub;
        nRight++;
      }
    }

    xRightMean /= (nRight > 0) ? nRight : 1; // TODO: do it here or somewhere else?
    xLeftMean /= (nLeft > 0) ? nLeft : 1;
    _imageTargetWidth = (xRightMean - xLeftMean);

    // sort 
    sortSpByY(_poles[LEFT]);
    sortSpByY(_poles[RIGHT]);

    for (int i = LEFT; i < SIDE_NUM; ++i) {
      _polesPtNum[i] = _poles[i].size();
      int polePtNumM1 = _polesPtNum[i] - 1;
      
      for (int j = 0; j < polePtNumM1; ++j) {
        if (me_abs(_poles[i][j].y - _poles[i][j+1].y) < 1e-6) {
          _poles[i][j].score = 0.0;
          _polesPtNum[i] -= 1;
        }
      }
    }

    DEBUG_LOG_CLR("[targetFinder::findTwoPoles] duplicant cleaned poles: " << _polesPtNum[0]
                  << " " << _polesPtNum[1] << std::endl, 7);
#ifdef MEwin    
    debugPrintPtsShort(_poles[LEFT], "left pole", true, true);
    debugPrintPtsShort(_poles[RIGHT], "right pole", true, true);
#endif

    return true;
  }

  void sortSpByY(SPInfoVec &sp) {
    // bubble sort
    int ptNum = sp.size();
    for (int i = ptNum; i > 0; --i) {
      for (int j = 0; j < i - 1; ++j) {
        if (sp[j].y > sp[j+1].y) {
          SaddlePoints::Types:: SPInfo tmp = sp[j];
          sp[j] = sp[j+1];
          sp[j+1] = tmp;
        }
      }
    }
  }

  bool StationlessTargetFinder::eliminateOutliersBySign(int side) {
    //const unsigned int MAX_PTS = 100; // todo: use already existing global consts
    //unsigned int iterNum = std::min(_poles[side].size(), MAX_PTS);

    const int MAX_PTS = 100;
    int iterNum = std::min((int)_poles[side].size(), MAX_PTS);
    float spSign[MAX_PTS];
    int mask[MAX_PTS];
    int minus1 = (_params.bottomLeftSquare == WHITE) ? -1 : 1;
    int idx[2];
    int len = 0;
    int validSpMap[MAX_PTS];
    
    int j = 0;
    //for (unsigned int i = 0; i < iterNum; ++i) {
    for (int i = 0; i < iterNum; ++i) {
      if (me_abs(_poles[side][i].score) > 0.1) {
        spSign[j] = (_poles[side][i].score > 0) ? 1 : -1;
        validSpMap[j] = i;
        mask[j] = minus1;
        minus1 *= -1;
        j++;
        len += 1;
      }
    }

    DEBUG_LOG_CLR("[eliminateOutliersBySign] " << ((side == LEFT) ? "LEFT" : "RIGHT") << std::endl, 5);

    bool ok = matchPattern(spSign, mask, len, idx, PATTERN_MODE_SUM);
    if (!ok) {
      return ok;
    }

    for (int j = 0; j < len; ++j) {
      if (j < idx[0] || j >= idx[1]) {
        _poles[side][validSpMap[j]].score = 0.0; // invalidate pt
      }
    }    

    return ok;
  }

  bool matchPattern(float *arr, int *mask, int len, int *idx, int mode) {
    if (matchPatternSingle(arr, mask, len, 0, mode)) {
      idx[0] = 0;
      idx[1] = len;
      return true;
    }

    // for (int i = 1; i < len - 6; ++i) { // TODO: change magic 6 to const or drop altogether
    for (int i = 1; i < len; ++i) { // TODO: change magic 6 to const or drop altogether
      int startIdx = matchPatternCyclic(arr, mask, len, i, mode);
      if (startIdx > -1) {
        idx[0] = startIdx;
        idx[1] = startIdx + len - i;
        return true;
      }
    }
    
    return false;
  }

  int matchPatternCyclic(float *arr, int *basemask, int len, int zeroLen, int mode) {
    int mask[MAX_CANDS_PTS];
    for (int i = 0; i < len; ++i) {
      if (i < len - zeroLen) {
        mask[i] = basemask[i];
      } else {
        mask[i] = 0;
      }
    }
    // for (int i = len - zeroLen; i < len; ++i) {
    //   mask[i] = 0;
    // }
    
    if (matchPatternSingle(arr, mask, len, zeroLen, mode)) {
      return 0;
    }

    for (int i = 1; i < zeroLen + 1; ++i) { 
      rotateIntArray(mask, len, 1); // cyclic rotation of mask to the right by a single "bit"
      if (matchPatternSingle(arr, mask, len, zeroLen, mode)) {
        return i;
      }
    }

    return -1;
  }

  void rotateIntArray(int *arr, int len, int step) {
    int stepAbs = me_abs(step);

    for (int i = 0;  i < stepAbs; ++i) {
      if (step > 0) { // rotate right
        int temp = arr[len-1];
        for (int j = len - 1; j > 0; --j) {
          arr[j] = arr[j-1];
        }
        arr[0] = temp;
      } else { // rotate left
        int temp = arr[0];
        for(int j = 0; j < len - 1; ++j) {
          arr[j] = arr[j+1];
        }
        arr[len-1] = temp;
      }
    }
  }


  bool matchPatternSingle(float *arr, int *mask, int len, int matchLen, int mode) {
    bool isMatch = false;
    float sum = 0.0;
    // float firstNonZeroMaskSign = 0.0;
    for (int i = 0; i < len; ++i) {
      sum += arr[i] * mask[i];
      // if (me_abs(firstNonZeroMaskSign) < 0.1 && me_abs(mask[i]) > 0.1) {
      //   firstNonZeroMaskSign = mask[i];
      // }
    }

    switch (mode) {
    case PATTERN_MODE_SUM:
      // sum *= -firstNonZeroMaskSign * ((_params.bottomLeftSquare == Types::WHITE) ? 1 : -1);
      isMatch = (sum == len - matchLen); // sum (as opposed to me_abs(sum) determines that it begins with negative sp, TODO: 1. use params to choose 2. after cyclic mask can start with +1
      break;
    case PATTERN_MODE_MEAN:
      float mean = sum / (len - matchLen);
      isMatch = (mean > 4 && mean < 6); // TODO: make members or struct or something
      // something
      break;
    }

// #ifdef MEwin
//     debugPrintPatternMatch(isMatch, sum, arr, mask, len, matchLen); // TODO: make member functions just for availability of _useDebugPrints?
// #endif

    return isMatch;
  }
    
  void StationlessTargetFinder::estimateSquareSize(int side) {
    const int MAX_PTS = 2*MAX_SP_NUM_IN_TARGET;
    int ptNum = _poles[side].size();
    MEtypes::ptr_vector<float> diffy(MAX_PTS);
    diffy.clear();
    
    int i = 0;
    while (i < ptNum-1) {
      if (me_abs(_poles[side][i].score) < 0.1) {
        i++;
        continue;
      }
      int j = i + 1;
      for (; j < ptNum; ++j) {
        if (me_abs(_poles[side][j].score) > 0.1) {
          break;
        }
      }
      // diffy.push_back(_poles[side][j].y - _poles[side][i].y);
      diffy.push_back(_poles[side][j].ySub - _poles[side][i].ySub);
      i = j;
    }

    if (diffy.size() < 2) {
      return;
    }

    _squareSize[side] = median_f(diffy);
  }

  bool StationlessTargetFinder::finalizeTarget(int side) {
    const int MAX_TARGET_CANDS = 10;
    MEtypes::ptr_vector<TargetCandInfo> targetCands(MAX_TARGET_CANDS);
    int candNum = std::min(MAX_TARGET_CANDS, _polesPtNum[side]/2);

    // find first valid pt
    int grossPtNum = (int)_poles[side].size();
    int idxValidStart = 0;
    for (; idxValidStart < grossPtNum; ++idxValidStart) {
      if (me_abs(_poles[side][idxValidStart].score) > 0.1) {
        break;
      }
    }

    DEBUG_LOG_CLR("[finalizeTarget@STF] side: " << side << ", idxValidStart: " << idxValidStart << std::endl, 5);
    
    int maxScore = -1000;
    int idxMaxScore = 0;
    int idxLastStart = -1;
    for (int i = 0; i < candNum; ++i) {
      TargetCandInfo cand;
      cand.imSquareSize = _squareSize[side];
      int iStart = std::max(idxLastStart + 1, idxValidStart + i);
      idxLastStart = buildTargetCandidate(cand, side, iStart);
      DEBUG_LOG_CLR("[finalizeTarget@STF] #cand: " << i << ", #pts: " << cand.ptNum << ", score: " << cand.score
                    << ", idx: [" << cand.idx[0] << ", " << cand.idx[1] << ", " << cand.idx[2] << ", " << cand.idx[3] << ", ...]\n", 5);
      if (cand.score == maxScore) { // if found two cands with same score abort and fail
        return false;
      }
      if (cand.score > maxScore) {
        maxScore = cand.score;
        idxMaxScore = i;
      }
      targetCands.push_back(cand);
    }

    if (maxScore < _params.spNumInTarget - _params.maxMissingSpAllowed) {
      return false;
    }

    TargetCandInfo& chosenCand = targetCands[idxMaxScore];
    invalidateNonTargetPoints(side, chosenCand);

    if (chosenCand.ptNum < _params.spNumInTarget) {
      fillMissingPts(side, chosenCand);
      sortSpByY(_poles[side]);
    }

    return true;
  }

  int StationlessTargetFinder::buildTargetCandidate(TargetCandInfo &cand, int side, int iStart) {
    // find valid start
    int grossPtNum = (int)_poles[side].size();
    int idxValidStart = iStart;
    for (; idxValidStart < grossPtNum; ++idxValidStart) {
      if (me_abs(_poles[side][idxValidStart].score) > 0.1) {
        break;
      }
    }

    if (idxValidStart == grossPtNum - 1) {
      return idxValidStart;
    }
    
    buildAndScoreTargetCandidate(cand, side, idxValidStart, true);
    if (cand.score >= _params.spNumInTarget - _params.maxMissingSpAllowed) {
      return idxValidStart;
    }
    DEBUG_LOG_CLR("[buildTargetCandidate@STF] floor score: " << cand.score << "\n------------------------------------------\n", 6);
    buildAndScoreTargetCandidate(cand, side, idxValidStart, false);
    return idxValidStart;
  }

  void StationlessTargetFinder::buildAndScoreTargetCandidate(TargetCandInfo &cand, int side, int idxValidStart, bool doFloor) {
    int grossPtNum = (int)_poles[side].size();
    BottomLeftSquare lowColor = (_poles[side][idxValidStart].score < 0) ? WHITE : BLACK;

    for (int i = 0; i < MAX_SP_NUM_IN_TARGET; ++i) {
        cand.idx[i] = -1;
    }

    int iLow = (lowColor == _params.bottomLeftSquare) ? 0 : 1;
    cand.idx[iLow] = idxValidStart;
    // cand.score = (lowColor == _params.bottomLeftSquare) ? 1 : 0;

    cand.score = 1;
    cand.ptNum = 1;
    int lastNdiff = 0;
    for (int i = idxValidStart + 1; i < grossPtNum; ++i) {
      if (me_abs(_poles[side][i].score) < 0.1) {
        continue;
      }
      int nDiff;
      if (doFloor) {
        nDiff = floor((_poles[side][i].ySub - _poles[side][idxValidStart].ySub)*1.f/cand.imSquareSize);
      } else {
        nDiff = ceil((_poles[side][i].ySub - _poles[side][idxValidStart].ySub)*1.f/cand.imSquareSize);
      }
      // nDiff = me_roundf((_poles[side][i].ySub - _poles[side][idxValidStart].ySub)*1.f/cand.imSquareSize);
      nDiff = std::max(lastNdiff+1, nDiff);
      lastNdiff = nDiff;
      DEBUG_LOG_CLR("[buildAndScoreTargetCandidate@STF] idx: " << i << " - " << idxValidStart << ", dy/ds = "
                    << "(" << _poles[side][i].ySub << " - " << _poles[side][idxValidStart].ySub << ")/" << cand.imSquareSize
                    << " = " << (_poles[side][i].ySub - _poles[side][idxValidStart].ySub) << "/" << cand.imSquareSize
                    << " = " << (_poles[side][i].ySub - _poles[side][idxValidStart].ySub)*1.f/cand.imSquareSize << ", r(dy/ds) = " << nDiff << std::endl, 6);
      if (iLow + nDiff >= _params.spNumInTarget) {
        break;
      }
      BottomLeftSquare color = (_poles[side][i].score < 0) ? WHITE : BLACK;
      bool okColor = (nDiff %2 == 0) ? (color == lowColor) : (color != lowColor);
      cand.idx[iLow+nDiff] = i;
      cand.score += okColor ? 1 : -5;
      cand.ptNum++;
    }
  }

  void StationlessTargetFinder::invalidateNonTargetPoints(int side, TargetCandInfo &chosenCand) {
    int grossPtNum = (int)_poles[side].size();
    int iMatch = 0;

    for (int i = 0; i < grossPtNum; ++i) {
      while (chosenCand.idx[iMatch] == -1 && iMatch < _params.spNumInTarget-1) {
        iMatch++;
      }
      if (i == chosenCand.idx[iMatch]) {
        if (iMatch < _params.spNumInTarget-1) {
          iMatch++;
        }
        continue;
      }
      _poles[side][i].score = 0.f;
    }
  }

  void StationlessTargetFinder::fillMissingPts(int side, TargetCandInfo &chosenCand) {
    if (_poles[side].isFull()) {
      return;
    }

    int iRef = chosenCand.idx[0];
    
    if (chosenCand.idx[0] == -1) {
      int x = _poles[side][chosenCand.idx[1]].x;
      int y = _poles[side][chosenCand.idx[1]].y - chosenCand.imSquareSize;
      SaddlePoints::Types::SPInfo sp;
      sp.x = x;
      sp.y = y;
      sp.xSub = x;
      sp.ySub = y;
      sp.val = _poles[side][chosenCand.idx[1]].val;
      sp.score = (_params.bottomLeftSquare == WHITE) ? -1.f : 1.f;
      sp.type = (_params.bottomLeftSquare == WHITE) ? SaddlePoints::Types::WHITE_BOTTOM_LEFT : SaddlePoints::Types::BLACK_BOTTOM_LEFT;
      
      SaddlePoints::API::refine(sp, FROM_LEVEL_LM2[2], true);
      // _distortion->undistort(sp);
      rectify(sp);
      _poles[side].push_back(sp);
      iRef = (int)_poles[side].size() - 1;
    }

    for (int i = 1; i < _params.spNumInTarget; ++i) {
      if (_poles[side].isFull()) {
        return;
      }
      if (chosenCand.idx[i] > -1) {
        continue;
      }
      int x = _poles[side][iRef].x;
      int y = _poles[side][iRef].y + i*chosenCand.imSquareSize;
      SaddlePoints::Types::SPInfo sp;
      sp.x = x;
      sp.y = y;
      sp.xSub = x;
      sp.ySub = y;
      sp.val = _poles[side][iRef].val;
      if (i%2 == 0) {
        sp.score = (_params.bottomLeftSquare == WHITE) ? -1.f : 1.f;
        sp.type = (_params.bottomLeftSquare == WHITE) ? SaddlePoints::Types::WHITE_BOTTOM_LEFT : SaddlePoints::Types::BLACK_BOTTOM_LEFT;
      } else {
        sp.score = (_params.bottomLeftSquare == WHITE) ? 1.f : -1.f;
        sp.type = (_params.bottomLeftSquare == WHITE) ? SaddlePoints::Types::BLACK_BOTTOM_LEFT : SaddlePoints::Types::WHITE_BOTTOM_LEFT;
      }
      SaddlePoints::API::refine(sp, FROM_LEVEL_LM2[2], true);
      // _distortion->undistort(sp);
      rectify(sp);
      _poles[side].push_back(sp);
    }
  }

  void StationlessTargetFinder::debugPrintPts(SPInfoVec &sp, std::string msg, bool skipLowScore) {
    if (!_useDebugPrints) {
      return;
    }

    unsigned int ptNum = sp.size();
    std::cout << msg << " " << ptNum << std::endl;
    for (unsigned int j = 0; j < ptNum; ++j) {
      if (skipLowScore && me_abs(sp[j].score) < 0.1) {
        continue;
      }
      std::cout << "(" << sp[j].x << ", " << sp[j].y << "; " 
                << sp[j].xSub << ", " << sp[j].ySub << "; " 
                << sp[j].type << ", " << sp[j].score << "), ";
    }
    std::cout << std::endl;
  }

  void StationlessTargetFinder::debugPrintPtsShort(SPInfoVec &sp, std::string msg, bool skipLowScore, bool isSub) {
    if (!_useDebugPrints) {
      return;
    }

    unsigned int ptNum = sp.size();
    std::cout << msg << " " << ptNum << std::endl;
    for (unsigned int j = 0; j < ptNum; ++j) {
      if (skipLowScore && me_abs(sp[j].score) < 0.1) {
        continue;
      }
      MEtl::string clr("\033[0m");
      if (me_abs(sp[j].score) < 0.1) {
        clr = "\033[31m";
      } else {
        clr = (sp[j].score > 0) ? "\033[30m" : "\033[29m";
      }
      std::cout << clr << j << "-(";
      if (isSub) {
        std::cout << sp[j].xSub << ", " << sp[j].ySub;
      } else {
        std::cout << sp[j].x << ", " << sp[j].y;
      }
      std::cout << "), \033[0m";
    }
    std::cout << std::endl;
  }

  void StationlessTargetFinder::printXbins(unsigned int *xbins, const int XBIN_NUM, const int XBIN_RAD) {
    if (!_useDebugPrints) {
      return;
    }

    std::cout << "[targetFinder::findPolesCands] xbins" << std::endl;
    for (int i = 0; i < XBIN_NUM; ++i) {
      if (xbins[i] > 0) {
        std::cout << "bin " << i << ", [" << XBIN_RAD*i - 640 << ", " << XBIN_RAD*(i+2) - 640 << "), " << " n " << xbins[i] << std::endl;
      }
    }
  }

  void debugPrintPatternMatch(bool isMatch, float sum, float *arr, int *mask, int len, int matchLen) {
    std::cout << "[targetFinder::matchPatternSingle] isMatch: " << isMatch << ", sum: " << sum << ", len-mLen: " << len - matchLen << ", matchLen: " << matchLen << std::endl;
    std::cout << "arr:" << std::endl;
    for (int i = 0; i < len; ++i) {
      std::cout << arr[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "mask:" << std::endl;
    for (int i = 0; i < len; ++i) {
      std::cout << mask[i] << " ";
    }
    std::cout << std::endl << std::endl;
  }
    

} // namespace TAC2 {
