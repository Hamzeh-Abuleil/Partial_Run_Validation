/*
 * slc.cpp
 *
 *  Created on: Jul 03, 2016
 *      Author: urilo
 */

#include "technology/calibration/WMBC/common/slc.h"
#include "utilities/saddlePoints/saddlePoints_API.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/calibration/WMBC/common/wmbcProperties.h"
#include "technology/calibration/WMBC/common/wmbcOutputIF.h"
#include "technology/calibration/WMBC/common/wmbc_dbg.h"
#include "technology/calibration/WMBC/common/wmbcUtils.h"
#include "technology/calibration/utilities/cameraProjections/distortionCorrection_algoAPI.h"

#define WMBC_SERIAL_PARAM(x, i) ((int)((x).actualSize) > i ? (x)._seriesArray[i] : (x)._seriesArray[(x).actualSize-1])

#ifdef MEwin
#include "technology/calibration/WMBC/i386/slcDebugger.h"
#define DEBUGGER(foo) do { if (_debugger!=NULL) _debugger->foo; } while (0)
#else
#define DEBUGGER(foo)
#endif

namespace WMBC {

  FOEFinder_Stationless::FOEFinder_Stationless(WmbcProperties *properties)  : FOEFinder(properties), _sp(MAX_SP_CAND_NUM), _camhData(MAX_FRAME_NUM) , _debugger(nullptr) {
    setParams(properties);
    for (int i = 0; i < 2; ++i) {
      _targetSearchWindow[i] = Float::MEgeo::Rect();
    }
    SaddlePoints::API::init(); 
    TFParams tfParams(false, _targetParams.spNumInTarget, properties->slcBottomLeftSquare()._seriesArray[0], _data.target.level); // TODO: organize,  remove bool prints, merge tfparams with targetparams
    // TODO: use setter
    // TODO: change targetFinder origins calls
    // _targetFinder = StationlessTargetFinder(tfParams, &_origins[CameraInfo::e_FORWARD]); // TODO: understand why STF is already created (see empty constructor of STF)
    _targetFinder.setParams(tfParams, &_data.camera.origin[CameraInfo::e_FORWARD]);
#ifdef MEwin
    if (Debug::Args::instance().existsParameter("-sSlc-debugTargetFinder")) {
      _targetFinder.setDebugger();
    }
    if (Debug::Args::instance().existsParameter("-sSlc-debug")) {
      _debugger = new FOEFinder_Stationless_Debugger(this);
    }
    DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_INIT));
#endif
    _validTargets = false;
    _validTargetFramesNum = 0;
    _successiveNonValidTargetFramesNum = 0;
    _firstValidFrame = 0;
    _validAlgoFrameNumAtTargetDetection = 0;

    _currDistanceToTarget = INVALID_VAL;
    _lastValidDistanceToTarget = INVALID_VAL;
    _lastValidDistanceToTargetFrame = -1;
    _isEndTraj = false;
    _postEndTrajFrames = -1;
    _imageTargetWidth = INVALID_VAL;
    _invImageTargetWidth = INVALID_VAL;
    _camhData.clear();
    _distanceEstimator.reset(DE_PROC_NOISE, DE_MEAS_NOISE);
    static float teProcNoise = Debug::Args::instance().getStickyValue("-sSlc-teProcNoise", TE_PROC_NOISE);
    static float teMeasNoise = Debug::Args::instance().getStickyValue("-sSlc-teMeasNoise", TE_MEAS_NOISE);
    for (int i = 0; i < 4; ++i) {
      _targetImgPosEstimator[i].reset(teProcNoise, teMeasNoise);
    }
    _enforcedPitch = Debug::Args::instance().getStickyValue("-sSlc-enforcedPitch", 100.f);
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
        _calib[i] = 0;
        _hist[i].binSize(HIST_BIN_SIZE[i]);
        _hist[i].reset();
        _hist[i].debugPrintMask(_debugPrintMask);
    }
  }

  FOEFinder_Stationless::~FOEFinder_Stationless() {
    SaddlePoints::API::clear();
  }

  void FOEFinder_Stationless::setParams(WmbcProperties *properties) {
    // FOEFinder::setParams(properties);
    for (int i = 0; i < 2; ++i) {
      _targetParams.rect[i] = WMBC_SERIAL_PARAM(properties->slcTargetRect(), i);
      _targetParams.dim[i] = WMBC_SERIAL_PARAM(properties->slcTargetDim(), i);
      
    }
    _targetParams.height = properties->slcTargetHeight() + _targetParams.rect[1];
    _targetParams.width = properties->slcTargetWidth();
    _targetParams.spNumInTarget = (_targetParams.dim[0]-1)*(_targetParams.dim[1]-1);
    ASSERT(_targetParams.spNumInTarget > 0 && _targetParams.spNumInTarget <= MAX_SP_NUM_IN_TARGET);
    
    for (int i = 0; i < SLC_MAX_TARGET_NUM; ++i) {
      _targetParams.bottomLeftSquare[i] = WMBC_SERIAL_PARAM(properties->slcBottomLeftSquare(), i);
    }

    static bool useGlobalBoundaries = Debug::Args::instance().existsParameter("-sSLC-useGlobalBoundaries");
    static float maxHeightDiff = Debug::Args::instance().getStickyValue("-sSLCmaxHeightDiff", SLC_MAX_HEIGHT_DIFF);
    static int minValidFrames = Debug::Args::instance().getStickyValue("-sSLCminValidFrames", SLC_MIN_VALID_FRAMES_DEFAULT);
    static int minValidFramesCamh = Debug::Args::instance().getStickyValue("-sSLCminValidFramesCamh", 5);
    static int maxRunFrames = Debug::Args::instance().getStickyValue("-sSLCmaxRunFrames", SLC_MAX_RUN_FRAMES_DEFAULT);
    static const int maxSuccNonValidTgtFrmNum = Debug::Args::instance().getStickyValue("-sSlc-maxSuccNonValidTgtFrmNum", 10);

    _targetParams.patternMatchingThreshold = properties->slcPatternMatchingThreshold();
    _targetParams.shiftWinX = properties->slcShiftWinX();
    _targetParams.searchWinRadX = properties->slcSearchWinRadX();
    _targetParams.searchWinRadY = properties->slcSearchWinRadY();
    _targetParams.useCalibShiftWin = properties->slcUseCalibShiftWin();
    _targetParams.maxDistance = properties->slcMaxDistance();
    _targetParams.minDistance = properties->slcMinDistance();
    _targetParams.camhMaxDistance = useGlobalBoundaries ? _targetParams.maxDistance : properties->slcCamhMaxDistance();
    _targetParams.camhMinDistance = useGlobalBoundaries ? _targetParams.minDistance : properties->slcCamhMinDistance();
    _targetParams.maxHeightDiff = maxHeightDiff;
    _targetParams.cameraHeightGuess = properties->slcCameraHeight();
    _targetParams.maxSuccNonValidTgtFrmNum = maxSuccNonValidTgtFrmNum;

    _convParams.minValidFrames = minValidFrames;
    _convParams.minValidFramesCamh = minValidFramesCamh;
    _convParams.maxRunFrames = maxRunFrames;
    _convParams.maxTravelDist = properties->slcMaxTravelDistance();

    _data.vehicle.vehicleRoll = properties->slcVehicleRoll();
  }

// ------------------------------------------- slcFindTargets Agenda -----------------------------------------------------------

  void FOEFinder_Stationless::runTargets() {
    DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_START));
    CalibStatus &status = _data.algo.total.status;
    bool isTimeout = (status == e_CALIB_TIMEOUT || status == e_CALIB_TIMEOUT_TIME || status == e_CALIB_TIMEOUT_DISTANCE);
    if (_data.algo.total.conv || isTimeout) {
      for (int i = 0; i < 2; ++i) {
        _targetSearchWindow[i].update(0.f, 0.f, 0.f, 0.f);
      }
      setTarget();
      DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_END));
      return;
    }

    float ds = _data.vehicle.ds; 
    _distanceEstimator.predict(-ds); // TODO: account for curvature

    if (_currDistanceToTarget < 0) {
      DEBUG_LOG_CLR_FRAME("[runTargets] passed targets, skip, dist: " << _currDistanceToTarget, WmbcDbgClr::e_BROWN);
      _distanceEstimator.correct(0.f, true);
      _currDistanceToTarget = _distanceEstimator.getMean();
      for (int i = 0; i < 2; ++i) {
        _targetSearchWindow[i].update(0.f, 0.f, 0.f, 0.f);
      }
      setTarget();
      DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_END));
      return;
    }

    if (_data.vehicle.speed < 0) {
      DEBUG_LOG_CLR_FRAME("[runTargets] driving in reverse, skip, dist: " << _currDistanceToTarget, WmbcDbgClr::e_BROWN);
      _distanceEstimator.correct(0.f, true);
      _currDistanceToTarget = _distanceEstimator.getMean();
      _validTargets = false;
      // _validTargetFramesNum = 0; do I need this after reset?
      _successiveNonValidTargetFramesNum = _targetParams.maxSuccNonValidTgtFrmNum; // reset search window
      setTarget();
      DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_END));
      return;
    }
    
    if (_validTargetFramesNum > 0) {
      float imShiftRat = me_abs(_currDistanceToTarget - ds) > EPS ? ds / (_currDistanceToTarget - ds) : 1.f;
      for (int i = 0; i < 4; ++i) {
        _targetImgPosEstimator[i].predict(imShiftRat*_targetImgPosEstimator[i].getMean());
      }

      DEBUG_LOG_CLR_FRAME("[runTargets] imShiftRat: " << imShiftRat 
                          << ", x: " << _targetImgPosEstimator[0].getMean() << " [" << (int)(_targetImgPosEstimator[0].getMean()) << "]"
                          << ", dxl: " << imShiftRat*_targetImgPosEstimator[0].getMean()
                          << ", xP: " << _targetImgPosEstimator[0].getMeanPred() << " [" << (int)(_targetImgPosEstimator[0].getMeanPred()) << "]"
                          << ", y: " << _targetImgPosEstimator[1].getMean() << " [" << (int)(_targetImgPosEstimator[1].getMean()) << "]"
                          << ", dyl: " << imShiftRat*_targetImgPosEstimator[1].getMean()
                          << ", yP: " << _targetImgPosEstimator[1].getMeanPred() << " [" << (int)(_targetImgPosEstimator[1].getMeanPred()) << "]", WmbcDbgClr::e_BROWN);
    }

    if (!_data.algo.img->available()) { 
      DEBUG_LOG_CLR_FRAME("[runTargets] img unavailable!", 1);
      for (int i = 0; i < 4; ++i) {
        _targetImgPosEstimator[i].correct(0.f, true);
      }
      _distanceEstimator.correct(0.f, true);
      _currDistanceToTarget = _distanceEstimator.getMean();
      _validTargets = false;
      _successiveNonValidTargetFramesNum += 1;
      setTarget();
      DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_END));
      return;
    }   
    
    findSp(); // fill _sp
    _targetFinder.init(_data.algo.img); // todo: why does targetFinder needs this img if it recieved the origins?
    bool ok = _targetFinder.run(_sp); // _sp is overwritten
    DEBUG_LOG_CLR_FRAME("[runTargets] targetFinder status: " << ok << ", #sp: " << _sp.size(), 5);
    if (!ok) {
      for (int i = 0; i < 4; ++i) {
        _targetImgPosEstimator[i].correct(0.f, true);
      }
      _distanceEstimator.correct(0.f, true);
      _currDistanceToTarget = _distanceEstimator.getMean();
      _validTargets = false;
      _successiveNonValidTargetFramesNum += 1;
      setTarget();
      WmbcOutputIF::instance().toItrkSlcTargets(_validTargets, &_sp, INVALID_VAL, _imageTargetWidth, _currDistanceToTarget, &_targetParams, _targetSearchWindow, _data.target.level);
      DEBUG_LOG_CLR_FRAME("[runTargets] targetFinder not ok, x: " << _targetImgPosEstimator[0].getMean()
                          << " [" << (int)(_targetImgPosEstimator[0].getMean()) << "]"
                          << ", xP: " << _targetImgPosEstimator[0].getMeanPred() << " [" << (int)(_targetImgPosEstimator[0].getMeanPred()) << "]"
                          << ", y: " << _targetImgPosEstimator[1].getMean() << " [" << (int)(_targetImgPosEstimator[1].getMean()) << "]"
                          << ", yP: " << _targetImgPosEstimator[1].getMeanPred() << " [" << (int)(_targetImgPosEstimator[1].getMeanPred()) << "]", WmbcDbgClr::e_BROWN);
      DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_END));
      return;
    }

    verifySaddlePts(); // _sp is rectified inside
    if (!_validTargets) {
      _successiveNonValidTargetFramesNum += 1;
      setTarget();
      // _targetImgPosEstimator predict->correct inside
      DEBUG_LOG_CLR_FRAME("[runTargets] verifySP invalid, x: " << _targetImgPosEstimator[0].getMean() << " [" << (int)(_targetImgPosEstimator[0].getMean()) << "]"
                          << ", xP: " << _targetImgPosEstimator[0].getMeanPred() << " [" << (int)(_targetImgPosEstimator[0].getMeanPred()) << "]"
                          << ", y: " << _targetImgPosEstimator[1].getMean() << " [" << (int)(_targetImgPosEstimator[1].getMean()) << "]"
                          << ", yP: " << _targetImgPosEstimator[1].getMeanPred() << " [" << (int)(_targetImgPosEstimator[1].getMeanPred()) << "]", WmbcDbgClr::e_BROWN);
      DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_END));
      return;
    }

    _validTargetFramesNum++;
    _successiveNonValidTargetFramesNum = 0;
    int n = _targetParams.spNumInTarget;
    
    if (_validTargetFramesNum == 1) {
      _firstValidFrame = globalFrameIndex;
      _validAlgoFrameNumAtTargetDetection = _data.algo.total.validFrameNum;
      // _targetImgPosEstimator[0].init(_sp[(n-1)>>1].xSub);
      // _targetImgPosEstimator[1].init(_sp[(n-1)>>1].ySub);
      for (int i = 0; i < 3; i+=2) {
        int idx = (n-1)>>1;
        idx += (i > 0) ? n : 0;
        // _targetImgPosEstimator[i].init(_sp[sh + ((n-1)>>1)].xSub);
        // _targetImgPosEstimator[i+1].init(_sp[sh + ((n-1)>>1)].ySub);
        _targetImgPosEstimator[i].init(_sp[idx].xSub);
        _targetImgPosEstimator[i+1].init(_sp[idx].ySub);
      }
        
      DEBUG_LOG_CLR_FRAME("[runTargets] targetImPosEst init, x: " << _targetImgPosEstimator[0].getMean() << " [" << (int)(_targetImgPosEstimator[0].getMean()) << "]"
                          << ", xP: " << _targetImgPosEstimator[0].getMeanPred() << " [" << (int)(_targetImgPosEstimator[0].getMeanPred()) << "]"
                          << ", y: " << _targetImgPosEstimator[1].getMean() << " [" << (int)(_targetImgPosEstimator[1].getMean()) << "]"
                          << ", yP: " << _targetImgPosEstimator[1].getMeanPred() << " [" << (int)(_targetImgPosEstimator[1].getMeanPred()) << "]", WmbcDbgClr::e_BROWN);
    } else {
      // _targetImgPosEstimator[0].correct(_sp[(n-1)>>1].xSub);
      // _targetImgPosEstimator[1].correct(_sp[(n-1)>>1].ySub);
      for (int i = 0; i < 3; i+=2) {
        int idx = (n-1)>>1;
        idx += (i > 0) ? n : 0;
        _targetImgPosEstimator[i].correct(_sp[idx].xSub);
        _targetImgPosEstimator[i+1].correct(_sp[idx].ySub);
      }
      DEBUG_LOG_CLR_FRAME("[runTargets] targetImPosEst correct, x: " << _targetImgPosEstimator[0].getMean() << " [" << (int)(_targetImgPosEstimator[0].getMean()) << "]"
                          << ", xP: " << _targetImgPosEstimator[0].getMeanPred() << " [" << (int)(_targetImgPosEstimator[0].getMeanPred()) << "]"
                          << ", y: " << _targetImgPosEstimator[1].getMean() << " [" << (int)(_targetImgPosEstimator[1].getMean()) << "]"
                          << ", yP: " << _targetImgPosEstimator[1].getMeanPred() << " [" << (int)(_targetImgPosEstimator[1].getMeanPred()) << "]", WmbcDbgClr::e_BROWN);
    }

    setTarget();

    unrectify(_sp);
    static bool dumpTargetPts = Debug::Args::instance().existsParameter("-sSLCdumpTargetPts");
    if (dumpTargetPts) {
      WmbcOutputIF::instance().toCextTargetPts(&_sp, _data.algo.img);
    }
    DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_TARGETS));
    DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_END));
  }

  void FOEFinder_Stationless::findSp() {
    float focalLm2 = _data.camera.focalLm2;
    int spDethThresh[2] = {1, 1};
    float spPatMatchThresh[2] = {_targetParams.patternMatchingThreshold, _targetParams.patternMatchingThreshold};
    // int n = (int)_sp.size();

    static float dethThFac = Debug::Args::instance().getStickyValue("-sSlc-dethThFac", 0.6f);
    static float patThDelta = Debug::Args::instance().getStickyValue("-sSlc-patThDelta", 0.15f);
    if (_validTargets && (int)_sp.size() == 2*_targetParams.spNumInTarget) {
      int n = _targetParams.spNumInTarget;
      for (int i = 0; i < 2; ++i) {
        spDethThresh[i] = _sp[0 + i*n].val;
        spPatMatchThresh[i] = me_abs(_sp[0 + i*n].score);
        float meanDeth = 0.f;
        float stdDeth = 0.f;
        DEBUG_LOG_CLR_FRAME("[findSp], prev frame sp, side: " << (i==0 ? "Left" : "Right"), WmbcDbgClr::e_CYAN);
        
        for (int j = i*n; j < (i+1)*n; ++j) {
          DEBUG_LOG_CLR_FRAME("[findSp], val[" << j << "]: " << _sp[j].val << ", score[" << j << "]: " << _sp[j].score, WmbcDbgClr::e_CYAN);
          if (_sp[j].val < 0 && _sp[j].val > spDethThresh[i]) {
            spDethThresh[i] = _sp[j].val;
          }
          float score = me_abs(_sp[j].score);
          meanDeth += score;
          stdDeth += score*score;
          if (score < spPatMatchThresh[i]) {
            spPatMatchThresh[i] = score;
          }
        }
        meanDeth /= n;
        stdDeth = sqrtf(stdDeth/n - meanDeth*meanDeth);
        DEBUG_LOG_CLR_FRAME("[findSp], maxVal(" << (i==0 ? "L): " : "R): ")  << spDethThresh[i]
                            << ", minScore(" << (i==0 ? "L): " : "R): ")  << spPatMatchThresh[i]
                            << ", meanScore(" << (i==0 ? "L): " : "R): ")  << meanDeth
                            << ", stdScore(" << (i==0 ? "L): " : "R): ")  << stdDeth
                            , WmbcDbgClr::e_CYAN);

        spDethThresh[i] = (int)(dethThFac*spDethThresh[i]);
        spPatMatchThresh[i] -= (patThDelta > 0) ? patThDelta : stdDeth;
        DEBUG_LOG_CLR_FRAME("[findSp], thresh" << (i==0 ? "L): " : "R): ") << spDethThresh[i] << ", " << spPatMatchThresh[i], WmbcDbgClr::e_CYAN);
      }
    }
    
    _sp.clear();

    // TODO: make sense with treshold params, use world square dim
    // const float TARGET_WIDTH_DEFAULT = 3.5f; // use this width for first search position if none given
    static const float TARGET_WIDTH_DEFAULT = Debug::Args::instance().getStickyValue("-sSlc-defaultTargetWidth", 2.f);
    static float threshFactor = Debug::Args::instance().getStickyValue("-sSLCthreshFactor", 1.f);
    static bool skipGaussFilter = !Debug::Args::instance().existsParameter("-sSLCdontSkipGaussFilter");
    static bool useEntireImg = Debug::Args::instance().existsParameter("-sSLCentireImg");
    static int initSquareSideSize4SP = Debug::Args::instance().getStickyValue("-sSLCsss4sp", -1);
    // static int maxSuccNonValidTgtFrmNum = Debug::Args::instance().getStickyValue("-sSlc-maxSuccNonValidTgtFrmNum", 10);
    int squareSideSize4SP = 2;

    int spXSearchRadius = _targetParams.searchWinRadX;
    int spYSearchRadius = _targetParams.searchWinRadY;

    int sRect[2][4]; // [target left/right][rect left,right,bottom,top]
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 4; ++j) {
        sRect[i][j] = 0;
      }
    }

    static int xRadMinMan = Debug::Args::instance().getStickyValue("-sSLCxRadMin", -1);
    static int xRadMaxMan = Debug::Args::instance().getStickyValue("-sSLCxRadMax", -1);
    static int yRadMinMan = Debug::Args::instance().getStickyValue("-sSLCyRadMin", -1);
    static int yRadMaxMan = Debug::Args::instance().getStickyValue("-sSLCyRadMax", -1);

    // static bool useCalib = Debug::Args::instance().existsParameter("-sSLCcalib4sp");
    bool useCalib = _targetParams.useCalibShiftWin;
    bool useTargetPosEstimation = (_targetImgPosEstimator[0].wasInit()
                                   && _targetImgPosEstimator[1].wasInit()
                                   && _targetImgPosEstimator[2].wasInit()
                                   && _targetImgPosEstimator[3].wasInit()
                                   && _successiveNonValidTargetFramesNum < _targetParams.maxSuccNonValidTgtFrmNum
                                  );

    int xRadMin = xRadMinMan;
    int xRadMax = xRadMaxMan;
    int yRadMin = yRadMinMan;
    int yRadMax = yRadMaxMan;

    float distance = _validTargetFramesNum > 0 ? _distanceEstimator.getMeanPred() : _targetParams.maxDistance;
    float invDistance = 1.f / (distance > EPS ? distance : _targetParams.maxDistance);
    int sz = focalLm2 * _targetParams.rect[1] * invDistance;
    static int squareSideSize4SP_fac = Debug::Args::instance().getStickyValue("-sSlc-sz4spFac", 1);
    if (initSquareSideSize4SP < 0 && distance < _targetParams.maxDistance) {
      squareSideSize4SP = (int)(sz * FROM_LEVEL_LM2[-_data.target.level]);
      if (squareSideSize4SP > 2*squareSideSize4SP_fac) {
        squareSideSize4SP /= squareSideSize4SP_fac;
      }
    } else {
      squareSideSize4SP = initSquareSideSize4SP;
    }

    int halfTargetHeight = ((_targetParams.spNumInTarget-1) * sz) >> 1;
    DEBUG_LOG_CLR_FRAME("[findSp] yradmin: " << yRadMin << ", dist: " << distance << ", sz: " << sz << ", squareSideSize4SP: " << squareSideSize4SP, 1);

    if (!useEntireImg) {
      if (!useTargetPosEstimation) {
        if (xRadMin < 0) {
          float targetWidth = (_targetParams.width > EPS) ? _targetParams.width : TARGET_WIDTH_DEFAULT;
          int x0 = 0.5 * focalLm2 * targetWidth * invDistance;
          x0 *= FROM_LEVEL_LM2[-_data.target.level];
          xRadMin = x0 - spXSearchRadius;
          xRadMax = x0 + spXSearchRadius;
          
          int y0 = focalLm2 * (_targetParams.height - _targetParams.cameraHeightGuess) * invDistance;
          y0 += halfTargetHeight;
          y0 *= FROM_LEVEL_LM2[-_data.target.level];
          yRadMin = y0 - spYSearchRadius;
          yRadMax = y0 + spYSearchRadius;
      
          DEBUG_LOG_CLR_FRAME("[findSp] sz: " << sz << ", y0: " << y0 << ", yRadMin: " << yRadMin << ", yRadMax: " << yRadMax, WmbcDbgClr::e_BLUE);
        }      
      
        xRadMin = std::max(0, xRadMin);
        
        //int shiftX = (int)(_foe.yawDeltaLm2FromEtcPrimary_f*useCalib*FROM_LEVEL_LM2[-_data.target.level]) + _targetParams.shiftWinX;
        int shiftX = (int)(_data.results.curr.foeLm2.X()*useCalib*FROM_LEVEL_LM2[-_data.target.level]) + _targetParams.shiftWinX;
        sRect[0][0] = shiftX - xRadMax;
        sRect[0][1] = shiftX - xRadMin;
        sRect[1][0] = shiftX + xRadMin;
        sRect[1][1] = shiftX + xRadMax;

        DEBUG_LOG_CLR_FRAME("[findSp] hDelta: " << _foe.horizonDeltaLm2FromEtcPrimary_f << ", useCalib: " << useCalib
                            << ", fromLvl: " << FROM_LEVEL_LM2[-_data.target.level]
                            << ", *: " << _foe.horizonDeltaLm2FromEtcPrimary_f*useCalib*FROM_LEVEL_LM2[-_data.target.level], WmbcDbgClr::e_BLUE);
        //int shiftY = (int)(_foe.horizonDeltaLm2FromEtcPrimary_f*useCalib*FROM_LEVEL_LM2[-_data.target.level]);
        int shiftY = (int)(_data.results.curr.foeLm2.Y()*useCalib*FROM_LEVEL_LM2[-_data.target.level]);
        sRect[0][2] = shiftY + yRadMin;
        sRect[0][3] = shiftY + yRadMax;
        sRect[1][2] = shiftY + yRadMin;
        sRect[1][3] = shiftY + yRadMax;
      } else {
        float x0u = _targetImgPosEstimator[0].getMeanPred();
        float y0u = _targetImgPosEstimator[1].getMeanPred();
        float x0d=0, y0d=0;
        float xCov = _targetImgPosEstimator[0].getCovPred();
        float yCov = _targetImgPosEstimator[1].getCovPred();

        // _distortion.distort(x0u, y0u, x0d, y0d);
        if (DistortionCorrectionAPI::isDistortionValid()) {
          DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, x0u, y0u, x0d, y0d);
        } else {
          // throw warning/error
        }
        int x0 = (int)(FROM_LEVEL_LM2[-_data.target.level] * x0d);
        int y0 = (int)(FROM_LEVEL_LM2[-_data.target.level] * y0d);
        float x1u = _targetImgPosEstimator[2].getMeanPred();
        float y1u = _targetImgPosEstimator[3].getMeanPred();
        float x1d=0, y1d=0;
        // _distortion.distort(x1u, y1u, x1d, y1d);
        if (DistortionCorrectionAPI::isDistortionValid()) {
          DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, x1u, y1u, x1d, y1d);
        } else {
          // throw warning/error
        }
        int x1 = (int)(FROM_LEVEL_LM2[-_data.target.level] * x1d);
        int y1 = (int)(FROM_LEVEL_LM2[-_data.target.level] * y1d);

        static int szRat = Debug::Args::instance().getStickyValue("-sSlc-searchWinSzRatio", 5);
        spYSearchRadius = FROM_LEVEL_LM2[-_data.target.level] * (halfTargetHeight + szRat*sz + xCov);
        spXSearchRadius = FROM_LEVEL_LM2[-_data.target.level] * (szRat*sz + yCov);
        
        spYSearchRadius = std::min(_targetParams.searchWinRadY, spYSearchRadius);
        spXSearchRadius = std::min(_targetParams.searchWinRadX, spXSearchRadius);

        sRect[0][0] = x0 - spXSearchRadius;
        sRect[0][1] = x0 + spXSearchRadius;
        sRect[1][0] = x1 - spXSearchRadius;
        sRect[1][1] = x1 + spXSearchRadius;
        sRect[0][2] = y0 - spYSearchRadius;
        sRect[0][3] = y0 + spYSearchRadius;
        sRect[1][2] = y1 - spYSearchRadius;
        sRect[1][3] = y1 + spYSearchRadius;
      }

      sRect[0][0] = std::max((*_data.target.img)->left(), sRect[0][0]);
      sRect[0][1] = std::min(0, sRect[0][1]);
      sRect[1][0] = std::max(0, sRect[1][0]);
      sRect[1][1] = std::min((*_data.target.img)->right(), sRect[1][1]);
      sRect[0][2] = std::max((*_data.target.img)->bottom(), sRect[0][2]);
      sRect[0][3] = std::min((*_data.target.img)->top(), sRect[0][3]);
      sRect[1][2] = std::max((*_data.target.img)->bottom(), sRect[1][2]);
      sRect[1][3] = std::min((*_data.target.img)->top(), sRect[1][3]);

      if (sRect[0][1] - sRect[0][0] <= (MAX_SEARCH_AREA<<1) ||
          sRect[1][1] - sRect[1][0] <= (MAX_SEARCH_AREA<<1) ||
          sRect[0][3] - sRect[0][2] <= (MAX_SEARCH_AREA<<1) ||
          sRect[1][3] - sRect[1][2] <= (MAX_SEARCH_AREA<<1)) {
        DEBUG_LOG_CLR_FRAME("[findSp] aborting", 1);
        return;
      }      

      for (int i = 0; i < 2; ++i) {
        _targetSearchWindow[i].update(sRect[i][0], sRect[i][1], sRect[i][2], sRect[i][3]);
      }

      DEBUG_LOG_CLR_FRAME("[findSp] xs: " << sRect[0][0] << ", " << sRect[0][1] << ", " << sRect[1][0] << ", " << sRect[1][1]
                          << ", xRadMin: " << xRadMin << ", xRadMax: " << xRadMax
                          << ", imgOx: " << (*_data.target.img)->origin().x << ", imgRight: " << (*_data.target.img)->right() << ", effWidth: " << (*_data.target.img)->effectiveWidth(), 1);

      DEBUG_LOG_CLR_FRAME("[findSp] ys: " << sRect[0][2] << ", " << sRect[0][3] << ", yRadMin: " << yRadMin << ", yRadMax: " << yRadMax 
                          << ", imgOy: " << (*_data.target.img)->origin().y
                          << ", imgTop: " << (*_data.target.img)->top() << ", effHeight: " << (*_data.target.img)->effectiveHeight(), 1);

      DEBUG_LOG_CLR_FRAME("[findSp] imgDistTop: " << (*_img)->top() << ", imgDistRight: " << (*_img)->right()
                          << ", imgDistOy: " << (*_img)->origin().y << ", dist effWidth: " << (*_img)->effectiveWidth() 
                          << ", dist effHeight: " << (*_img)->effectiveHeight(), 1);
    }

    DEBUG_LOG_CLR_FRAME("[findSp] -IMG- available: " << _data.target.img->available() << ", lastUpdate: " << _data.target.img->lastUpdate(), WmbcDbgClr::e_BLUE);

    if (useEntireImg) {
      SaddlePoints::Types::Params params(SLC_MAX_IM_WIDTH, SLC_MAX_IM_HEIGHT, false, skipGaussFilter, false, threshFactor, 0.5*(spDethThresh[0]+spDethThresh[1]));
      SaddlePoints::API::setParams(params);
    SaddlePoints::API::setImage(_data.target.img->getObj()); // TODO: should I update do something with the image somewhere
      SaddlePoints::API::find(squareSideSize4SP, _sp, 0.5*(spPatMatchThresh[0]+spPatMatchThresh[1]));
    } else {
      static bool reverse = Debug::Args::instance().existsParameter("-sSlc-reverseTargetOrder");
      for (int i = 0; i < 2; ++i) {
        i = reverse ? 1-i: i;
        SPInfoVec sideSp(MAX_SP_CAND_NUM);
        sideSp.clear();
        Fix::MEimage::Image sideImg;
        sideImg.shallowCopy(_data.target.img->getObj());
        sideImg.setImageRect(_targetSearchWindow[i]);

        SaddlePoints::Types::Params params(SLC_MAX_IM_WIDTH, SLC_MAX_IM_HEIGHT, false, skipGaussFilter, false, threshFactor, spDethThresh[i]);
        SaddlePoints::API::setParams(params);
        SaddlePoints::API::setImage(sideImg); // TODO: should I update do something with the image somewhere
        SaddlePoints::API::find(squareSideSize4SP, sideSp, spPatMatchThresh[i]);

        int spsize = sideSp.size();
        for (int j = 0; j < spsize; ++j) {
          _sp.push_back(sideSp[j]);
        }
      }
      SaddlePoints::API::setImage(_data.target.img->getObj());
    }

    if (_data.target.level > -2) {
      int spsize = _sp.size();
      for (int i = 0; i < spsize; ++i) {
        _sp[i].x *= TO_LEVEL_LM2[-_data.target.level];
        _sp[i].y *= TO_LEVEL_LM2[-_data.target.level];
        _sp[i].xSub *= TO_LEVEL_LM2[-_data.target.level];
        _sp[i].ySub *= TO_LEVEL_LM2[-_data.target.level];
      }
    }

    DEBUGGER(updateStage(FOEFinder_Stationless_Debugger::e_SEARCH_WIN));
    DEBUG_LOG_CLR_FRAME("[findSp] #sp = " << _sp.size(), 3);
  }

  void FOEFinder_Stationless::verifySaddlePts() {
    // TODO: create a target class which computes its dimensions and its validity
    // static bool subpixelInterpEdges = Debug::Args::instance().existsParameter("-sSLCsubpixelInterpEdges");
    // static bool subpixelInterp = Debug::Args::instance().existsParameter("-sSLCsubpixelInterp") && !subpixelInterpEdges;

    // test ratios of world params and respective image ratios
    _validTargets = ((int)_sp.size() == 2*_targetParams.spNumInTarget);
    if (!_validTargets) {
      _imageTargetWidth = INVALID_VAL;
      _invImageTargetWidth = INVALID_VAL;
      _targetImgPosEstimator[0].correct(0.f, true);
      _targetImgPosEstimator[1].correct(0.f, true);
      _distanceEstimator.correct(0.f, true);
      _currDistanceToTarget = _distanceEstimator.getMean();
      WmbcOutputIF::instance().toItrkSlcTargets(_validTargets, &_sp, INVALID_VAL, _imageTargetWidth, _currDistanceToTarget, &_targetParams, _targetSearchWindow, _data.target.level);
      DEBUG_LOG_CLR("[verifySaddlePts] #sp: " << _sp.size() << ", distance: " << _currDistanceToTarget, WmbcDbgClr::e_RED);
      return ;
    }

      // SaddlePoints::API::subpixelInterpolation(_sp, FROM_LEVEL_LM2[-_data.target.level]);
      // SaddlePoints::API::refine(_sp, FROM_LEVEL_LM2[-_data.target.level]);

    int n = _targetParams.spNumInTarget;
    // if (subpixelInterpEdges) {
    //   int idx[4] = {0, n-1, n, 2*n-1}; // bottom-left, top-left, bottom-right, top-right
    //   for (int i = 0; i < 4; ++i) {
    //     SaddlePoints::API::refine(_sp, FROM_LEVEL_LM2[-_data.target.level], idx[i]);
    //   }
    // }

    for (int i = 0; i < 2*n; ++i) {
      // _distortion.undistort(_sp[i].xSub, _sp[i].ySub, _spRect[i][0], _spRect[i][1]); 
      _spRect[i][0] = _sp[i].xSub;
      _spRect[i][1] = _sp[i].ySub;
    }

    // im target width and distance to targets
    _imageTargetWidth = 0.f;
    for (int i = 0; i < n; ++i) {
      float dx = _spRect[i+n][0] - _spRect[i][0];
      float dy = _spRect[i+n][1] - _spRect[i][1];
      _imageTargetWidth += sqrtf(dx*dx + dy*dy);
    }
    _imageTargetWidth /= n;

    _validTargets = (_imageTargetWidth > IM_TARGET_WIDTH_TH);
    if (!_validTargets) {
      DEBUG_LOG_CLR("[verifySaddlePts] imWidth: " << _imageTargetWidth<< ", distance: " << _currDistanceToTarget, WmbcDbgClr::e_RED);
      _imageTargetWidth = INVALID_VAL;
      _invImageTargetWidth = INVALID_VAL;
      _targetImgPosEstimator[0].correct(0.f, true);
      _targetImgPosEstimator[1].correct(0.f, true);
      _distanceEstimator.correct(0.f, true);
      _currDistanceToTarget = _distanceEstimator.getMean();
      WmbcOutputIF::instance().toItrkSlcTargets(_validTargets, &_sp, INVALID_VAL, _imageTargetWidth, _currDistanceToTarget, &_targetParams, _targetSearchWindow, _data.target.level);
      
      return;
    }
    _invImageTargetWidth = 1.0 / _imageTargetWidth;
    
    // check height of two targets in image is the same
    float dx = _spRect[n-1][0] - _spRect[0][0];
    float dy = _spRect[n-1][1] - _spRect[0][1];
    float lenLeftTargetIm = sqrtf(dx*dx + dy*dy);

    dx = _spRect[2*n-1][0] - _spRect[n][0];
    dy = _spRect[2*n-1][1] - _spRect[n][1];
    float lenRightTargetIm = sqrtf(dx*dx + dy*dy);

    DEBUG_LOG_CLR("[verifySaddlePts] dLenIm: |" << lenLeftTargetIm << " - " << lenRightTargetIm << "| = "
                  << me_abs(lenLeftTargetIm - lenRightTargetIm) << ", corners: (" << _spRect[0][0] << ", " << _spRect[0][1]
                  << ")-(" << _spRect[n-1][0] << ", " << _spRect[n-1][1] << ")-(" << _spRect[n][0] << ", " << _spRect[n][1]
                  << ")-(" << _spRect[2*n-1][0] << ", " << _spRect[2*n-1][1]
                  << "), imWidth: " << _imageTargetWidth << ", distance: " << _currDistanceToTarget, 3);

    bool isLenTargetExist = (lenLeftTargetIm > EPS) && (lenRightTargetIm > EPS);
    bool isLenTargetsEqual = (me_fabs(lenLeftTargetIm - lenRightTargetIm) < _targetParams.maxHeightDiff); 
    _validTargets = _validTargets && isLenTargetExist && isLenTargetsEqual;
    if (!_validTargets) {
      WmbcOutputIF::instance().toItrkSlcTargets(_validTargets, &_sp, INVALID_VAL, _imageTargetWidth, _currDistanceToTarget, &_targetParams, _targetSearchWindow, _data.target.level);
      return;
    }

    float lenTargetIm = 0.5*(lenLeftTargetIm + lenRightTargetIm);
    float lenTargetWorld = (_targetParams.spNumInTarget-1) * _targetParams.rect[1];
    
    // update distance to targets
    float focalLm2 = _data.camera.focalLm2;
    float distToTargetMeas = 0.f;
    if (_targetParams.width > EPS) { // use width if exist
      distToTargetMeas = _targetParams.width * focalLm2 * _invImageTargetWidth;
    } else {
      distToTargetMeas = lenTargetWorld * focalLm2 / lenTargetIm;
    }
    
    if (_validTargetFramesNum > 0) {
      _distanceEstimator.correct(distToTargetMeas);
    } else {
      _distanceEstimator.init(distToTargetMeas);
    }
    _currDistanceToTarget = _distanceEstimator.getMean();

    _lastValidDistanceToTarget = _currDistanceToTarget;
    _lastValidDistanceToTargetFrame = globalFrameIndex;

    WMBC_PRINT(WmbcDbgStg::e_TARGETS, 3, "[verifySaddlePts] Z_w = (f/w)W = (%.0f/%.2f)%.2f = %.2f, Z_l = (f/l)L = (%.0f/%.2f)%.2f = %.2f, W_l = (w/l)L = (%.2f/%.2f)%.2f = %.2f\n",
               _data.camera.focalLm2, _imageTargetWidth, _targetParams.width,
               _targetParams.width * focalLm2 * _invImageTargetWidth,
               focalLm2, lenTargetIm, lenTargetWorld,
               lenTargetWorld * focalLm2 / lenTargetIm,
               _imageTargetWidth, lenTargetIm, lenTargetWorld,
               lenTargetWorld * _imageTargetWidth / lenTargetIm);

    // check height to width ratios image vs world
    // float lenTargetIm = 0.5 * (lenLeftTargetIm + lenRightTargetIm);
    // float widthIm = _targetFinder.getImageTargetWidth();
    // widthIm = (widthIm > EPS) ? widthIm : 1.0;
    // float ratioIm = lenTargetIm / widthIm;

    // float lenTargetWorld = (SP_NUM_IN_TARGET - 1) * _params.squareSideSize;
    // float ratioWorld = lenTargetWorld / (_targetWidth);
    
    // DEBUG_LOG_CLR("[verifySaddlePts] ratioIm: " << ratioIm << ", ratioWorld: " << ratioWorld 
    //               << ", diff: " << me_abs(ratioIm - ratioWorld) 
    //               << ", th: " << 0.1 * me_abs(ratioWorld) << std::endl, 3);
    // _validTargets = _validTargets && (me_abs(ratioIm - ratioWorld) < 0.1 * me_abs(ratioWorld));

    WmbcOutputIF::instance().toItrkSlcTargets(_validTargets, &_sp, lenTargetIm, _imageTargetWidth, _currDistanceToTarget, &_targetParams, _targetSearchWindow, _data.target.level);
  }

  void FOEFinder_Stationless::setTarget() {
    _targetDraw.valid = _validTargets;

    int n = _targetParams.spNumInTarget;
    if ((int)_sp.size() != 2*n) {
      for (int i = 0; i < 2; ++i) {
        _targetDraw.imCorners[i].clear();
      }
      _targetDraw.distance = 0;
      return;
    }
    
    for (int i = 0; i < 2; ++i ) {
      int ib = i*n;
      int it = (i+1)*n - 1;
      float s = (_sp[it].y - _sp[ib].y)*1.f/(n-1);
      float left = _sp[ib].x - s;
      float right = _sp[ib].x + s;
      float bottom = _sp[ib].y - s;
      float top = _sp[it].y + s;

      if (left >= right || bottom >= top) {
        _targetDraw.imCorners[i].clear();
        continue;
      }
      _targetDraw.imCorners[i] = Float::MEgeo::Rect(left, right, bottom, top);
    }
    _targetDraw.distance = _currDistanceToTarget;
  }

// ------------------------------------------- runWMBC Agenda -----------------------------------------------------------
  void FOEFinder_Stationless::run() {
    CalibStatus &status = _data.algo.total.status;
    bool isTimeout = (status == e_CALIB_TIMEOUT || status == e_CALIB_TIMEOUT_TIME || status == e_CALIB_TIMEOUT_DISTANCE);
    if (isTimeout) {
      return;
    }

    updateReset();
    setInputData();
    // if (globalFrameIndex == 0) {
    //   _foe.reset(_worldModelData.invFocalLm2, &_origins[CameraInfo::e_FORWARD]);
    // }

    validateFrameVehicle();
    validateFrameAlgo();
    calcRoll();
    calcFoe();
    calcCamHeight();

    setResults();
    updateConvergence();
    updateStatus();    
    setDebugShowData();
  }

  void FOEFinder_Stationless::calcFoe() {
    // static float enforcedPitch = Debug::Args::instance().getStickyValue("-sSlc-enforcedPitch", 100.f);
    bool valid = (_data.algo.single[e_YAW].validFrame && _data.algo.single[e_HORIZON].validFrame);

    if (valid) {
      _hist[e_YAW].update(_data.em.yaw);
      _hist[e_HORIZON].update(_data.em.pitch);
      if (!_data.algo.single[e_YAW].conv) {
        _calib[e_YAW] = _hist[e_YAW].median();
      }
      if (!_data.algo.single[e_HORIZON].conv) {
        _calib[e_HORIZON] = _hist[e_HORIZON].median();
      }
    }

    if (_enforcedPitch < 100) {
      _calib[e_HORIZON] = _enforcedPitch*DEG2RAD;
    }

    WmbcOutputIF::instance().toItrkFoe(&_hist[e_YAW], &_hist[e_HORIZON], _data.camera.focalLm2, _data.algo.quickMode); // TODO: turn on

    if (!valid && _enforcedPitch > 99) {
      return;
    }

    if (_validTargets && !_camhData.isFull()) {
      CamhData cd;
      cd.frame = globalFrameIndex;
      cd.foer[0] = _data.em.yaw * _data.camera.focalLm2;
      cd.foer[1] = _data.em.pitch * _data.camera.focalLm2;
      cd.distance = _currDistanceToTarget;
      cd.distanceMeas = _distanceEstimator.getMeas();
      cd.invImageTargetWidth = _invImageTargetWidth;
      cd.camh = INVALID_VAL;
      cd.camhErr = 0;
      cd.valid = false;
      cd.chosen = false;
      _camhData.push_back(cd);
    }
  }

  void FOEFinder_Stationless::calcRoll() {
    static float enforcedRoll = Debug::Args::instance().getStickyValue("-sSlc-enforcedRoll", 100.f);
    if (enforcedRoll < 100) {
      _calib[e_ROLL] = enforcedRoll*DEG2RAD;
      return;
    }
    
    if (!_data.algo.single[e_ROLL].validFrame) {
      return;
    }

    float frameRoll = 0.0;
    int n = _targetParams.spNumInTarget;

    for (int i = 0; i < n; ++i) {
      float dx = _spRect[i+n][0] - _spRect[i][0];
      float dy = _spRect[i+n][1] - _spRect[i][1];
      if (me_abs(dx) < EPS) {
        // throw error
        return;
      }
      frameRoll +=  dy / dx;
    }

    frameRoll /= n;
    _hist[e_ROLL].update(frameRoll);
    if (!_data.algo.total.conv) {
      _calib[e_ROLL] = _hist[e_ROLL].median();
    }

    // TODO: write to itrk also when exit abnormally
    DEBUG_LOG_CLR("[calcRoll@SLC] frame roll: " << frameRoll << ", median roll: " << _hist[e_ROLL].median() 
                  << ", mrollBin: " << _hist[e_ROLL].lastBin(), 2);
  }

 void FOEFinder_Stationless::calcCamHeight() {
    if (_data.algo.total.conv) {
      return;
    }

    // enter compute phase if trajectory ended and yaw,hor,roll converged
    bool isRestConv = _isEndTraj && (_data.algo.single[e_YAW].conv
                                     && _data.algo.single[e_HORIZON].conv
                                     && _data.algo.single[e_ROLL].conv);
    // accumulate frame if frame valid and match frame in container
    bool isMatchFrame = (_data.algo.single[e_CAM_HEIGHT].validFrame && _camhData.size() > 0 
                         && (_camhData.back()).frame == globalFrameIndex);
    // enter calculation if enough camh data and beyond calc distance
    bool isCompute = ((int)_camhData.size() >= _convParams.minValidFramesCamh
                       && (_currDistanceToTarget < _targetParams.camhMinDistance || _isEndTraj));

    if (!isRestConv) { // if other dof did not converge, collect target data
      if (isMatchFrame) {
        fillCamhDataTargetCenter();
        WMBC_PRINT(WmbcDbgStg::e_RESULTS, WmbcDbgClr::e_BROWN, "[calcCamHeight@SLC] collecting %d: camhData-size=%d, distance=%.2f",
                   globalFrameIndex, _camhData.size(), _camhData.back().distance);
      } else {
        WMBC_PRINT(WmbcDbgStg::e_RESULTS, 1, "[calcCamHeight@SLC] skip collecting: valid-camh-frame=%d (target=%d, dist=%d), valid-hor-frame=%d, "
                   "camhData-size=%d, match-frame (camhData/current): %d/%d",
                   _data.algo.single[e_CAM_HEIGHT].validFrame, _validTargets, (_currDistanceToTarget < _targetParams.camhMaxDistance),
                   _data.algo.single[e_HORIZON].validFrame, _camhData.size(), (_camhData.size()>0 ? _camhData.back().frame : -999), globalFrameIndex);
      }
    } else {
      if (isCompute) {
        computeCamh();
      } else {
        WMBC_PRINT(WmbcDbgStg::e_RESULTS, 1, "[calcCamHeight@SLC] skip calculating: restConverged=%d, camh-data-size=%d (min-size=%d)",
                   isRestConv, _camhData.size(), _convParams.minValidFramesCamh);
      }
    }

    // TODO: write to itrk also when exit abnormally
    bool phaseStates[3] = {isRestConv, isMatchFrame, isCompute};
    WmbcOutputIF::instance().toItrkRoll(&_hist[e_ROLL], &_hist[e_CAM_HEIGHT]);
    WmbcOutputIF::instance().toItrkSlcCamHeight(&_camhData, &_convParams, phaseStates, 0.f);
  }

  void FOEFinder_Stationless::computeCamh() {
    static bool useProjMean = Debug::Args::instance().existsParameter("-sSlc-useProjMean");
    if (useProjMean) {
      computeCamhProjMean();
    } else {
      computeCamhLinearRegression();
    }
  }

  void FOEFinder_Stationless::computeCamhProjMean() {
    CamhDataIter begin = _camhData.begin();
    CamhDataIter end = _camhData.end();
    CamhDataIter closestValid = _camhData.begin();

    float startDistance = _targetParams.camhMaxDistance;
    float endDistance = _targetParams.camhMinDistance;
    float cosr = me_cosf(-_calib[e_ROLL]);
    float sinr = me_sinf(-_calib[e_ROLL]);
    float horFinal = sinr * _calib[e_YAW] + cosr * _calib[e_HORIZON];

    float camhErrMean = 0.f;
    int validNum = 0;
    float camhSum = 0;

    for(CamhDataIter it = begin+1; it != end; ++it) {
      if (!it->valid) {
        continue;
      }

      float hor = sinr * it->foer[0] + cosr * it->foer[1];
      float Y_b = _targetParams.height;
      float y_b[2] = {it->targetCornersY[0], it->targetCornersY[2]};
      float camh_bl = Y_b - (y_b[0] - horFinal) * it->distance * _data.camera.invFocalLm2;
      float camh_br = Y_b - (y_b[1] - horFinal) * it->distance * _data.camera.invFocalLm2;
      it->camh = (camh_bl+camh_br)/2;
      it->camhErr = -(horFinal - hor) * it->distance * _data.camera.invFocalLm2;

      if (it->distance >= INVALID_VAL || it->distance > startDistance || it->distance < endDistance) {
        it->valid = false;
        WMBC_PRINT(WmbcDbgStg::e_RESULTS, 1, "[calcCamHeight@SLC] camh skipping: %d, distance=%.2fm range: (%.2f, %.2f)",
                   it->frame, it->distance, startDistance, endDistance);
        continue;
      }

      validNum++;
      camhSum += it->camh;
      camhErrMean += it->camhErr;
      closestValid = it;

      WMBC_PRINT(WmbcDbgStg::e_RESULTS, 3, "[calcCamHeight@SLC] camh logging: %d, frame-pitch = %.10f + %.10f = %.10f, frame-camh = %.10f + %.10f = %.10f, "
                 "distance = %.10f distanceM = %.10f, y = %.10f, Y = %.10f",
                 it->frame, horFinal*_data.camera.invFocalLm2*RAD2DEG, (hor-horFinal)*_data.camera.invFocalLm2*RAD2DEG,
                 hor*_data.camera.invFocalLm2*RAD2DEG, it->camh, it->camhErr, it->camh+it->camhErr, it->distance,
                 it->distanceMeas, 0, 0);
    }

    static bool useLastFrame = Debug::Args::instance().existsParameter("-sSLCcamhUseLastFrame");
    static bool useFramePitch = Debug::Args::instance().existsParameter("-sSLCcamhUseFramePitch");
    if (useLastFrame) {
      _data.algo.single[e_CAM_HEIGHT].conv = (validNum > 0);
      _calib[e_CAM_HEIGHT] = _data.algo.single[e_CAM_HEIGHT].conv ? closestValid->camh : INVALID_VAL;
      camhErrMean = _data.algo.single[e_CAM_HEIGHT].conv ? closestValid->camhErr : INVALID_VAL;
    } else {
      _data.algo.single[e_CAM_HEIGHT].conv = (validNum >= _convParams.minValidFramesCamh);
      _calib[e_CAM_HEIGHT] = _data.algo.single[e_CAM_HEIGHT].conv ? camhSum / validNum : INVALID_VAL;
      camhErrMean = _data.algo.single[e_CAM_HEIGHT].conv ? camhErrMean / validNum : INVALID_VAL;
    }

    if (useFramePitch) {
      _calib[e_CAM_HEIGHT] += camhErrMean;
    }

    WMBC_PRINT(WmbcDbgStg::e_RESULTS, 2, "[calcCamHeight@SLC] camh calculating: #valids=%d, final pitch=%.2fdeg, result camh=%.3f\n",
               validNum, horFinal*_data.camera.invFocalLm2*RAD2DEG, _calib[e_CAM_HEIGHT]);
  }

  void FOEFinder_Stationless::computeCamhLinearRegression() {
    CamhDataIter begin = _camhData.begin();
    CamhDataIter end = _camhData.end();

    float startDistance = _targetParams.camhMaxDistance;
    float endDistance = _targetParams.camhMinDistance;
    int validNum = 0;
    const int MAX_FRAME_CAMH_NUM = 50;
    double y[4][MAX_FRAME_CAMH_NUM]; // bot-left, top-left, bot-right, top-right
    double invZ[MAX_FRAME_CAMH_NUM];
    for (int i = 0; i < MAX_FRAME_CAMH_NUM; ++i) {
      for (int j = 0; j < 4; ++j) {
        y[j][i] = 0.0;
      }
      invZ[i] = 0.0;
    }

    int pIdx = 0;
    double ym[4] = {0.0, 0.0, 0.0, 0.0};
    double ys[4] = {0.0, 0.0, 0.0, 0.0};
    double invZm = 0.0, invZs = 0.0;

    for(CamhDataIter it = begin+1; it != end; ++it) {
      if (!it->valid) {
        continue;
      }

      if (it->distance > INVALID_VAL || it->distance > startDistance || it->distance < endDistance) {
        it->valid = false;
        WMBC_PRINT(WmbcDbgStg::e_RESULTS, 1, "[calcCamHeight@SLC] camh skipping %d: Z=%.2f is out of range: (%.2f, %.2f)",
                   it->frame, it->distance, startDistance, endDistance);
        continue;
      }

      for (int i = 0; i < 4; ++i) {
        y[i][pIdx] = it->targetCornersY[i];
        ym[i] += y[i][pIdx];
        ys[i] += y[i][pIdx]*y[i][pIdx];
      }
      invZ[pIdx] = 1.f/it->distance; // TODO: assert endDistance>0
      invZm += invZ[pIdx];
      invZs += invZ[pIdx]*invZ[pIdx];

      pIdx++;
      validNum++;

      WMBC_PRINT(WmbcDbgStg::e_RESULTS, 3, "[calcCamHeight@SLC] camh logging: %d, distance = %.10f distanceM = %.10f, "
                 "y_bl = %.10f, y_tl = %.10f, y_br = %.10f, y_tr = %.10f",
                 it->frame, it->distance, it->distanceMeas, y[0][pIdx-1], y[1][pIdx-1], y[2][pIdx-1], y[3][pIdx-1]);
    }

    if (validNum ==0) {
      _data.algo.single[e_CAM_HEIGHT].conv = false;
      _calib[e_CAM_HEIGHT] = INVALID_VAL;
      return;
    }

    for (int i = 0; i < 4; ++i) {
      ym[i] /= validNum;
      ys[i] = sqrt(ys[i]/validNum - ym[i]*ym[i]);
    }
    invZm /= validNum;
    invZs = sqrt(invZs/validNum - invZm*invZm);

    for (int i = 0; i < validNum; ++i) {
      for (int j = 0; j < 4; ++j) {
        y[j][i] = (y[j][i] - ym[j])/ys[j];
      }
      invZ[i] = (invZ[i] - invZm)/(invZs);
    }

    float Yb = _targetParams.height;
    float Yt = Yb + (_targetParams.spNumInTarget-1)*_targetParams.rect[1];
    float Y[4] = {Yb, Yt, Yb, Yt};
    float camhSum = 0.f;
    float coeff[3] = {0.f, 0.f, 0.f};
    int idxBest[MAX_FRAME_CAMH_NUM];
    const float thresh = 0.2*DEG2RAD*_data.camera.focalLm2;

    for (int i = 0; i < 4; ++i) {
      ransacBestLineSym(invZ, y[i], idxBest, validNum, coeff, thresh);
      camhSum += Y[i] - (coeff[2]/coeff[1])*_data.camera.invFocalLm2*(ys[i]/invZs);
    }

    // todo: average with weights according to residual in bestFit
    int n = 4;
    _data.algo.single[e_CAM_HEIGHT].conv = (n > 0);
    _calib[e_CAM_HEIGHT] = _data.algo.single[e_CAM_HEIGHT].conv ? camhSum/n : INVALID_VAL;

    WMBC_PRINT(WmbcDbgStg::e_RESULTS, 2, "[calcCamHeight@SLC] camh calculating: #valids=%d, result camh=%.3f\n",
               validNum, _calib[e_CAM_HEIGHT]);
  }

  void FOEFinder_Stationless::fillCamhDataTargetCenter() {
    int n = _targetParams.spNumInTarget;
    float cosr = me_cosf(-_calib[e_ROLL]); // this is interimRoll
    float sinr = me_sinf(-_calib[e_ROLL]);

    int idx[4] = {0, n-1, n, 2*n-1}; // bottom-left, top-left, bottom-right, top-right
    for (int i = 0; i < 4; ++i) {
      float xr = _spRect[idx[i]][0];
      float yr = _spRect[idx[i]][1];
      _camhData.back().targetCornersY[i] = sinr*xr + cosr*yr;
    }

    _camhData.back().valid = true;

    printSP();
  }  

  void FOEFinder_Stationless::validateFrameAlgo() {
    FOEFinder::validateFrameAlgo();
    int &m = _data.algo.pauseReason;

   if (!_validTargets && _currDistanceToTarget >= 0) {
      m |= e_LEFT_TARGET_NOT_FOUND;
      m |= e_RIGHT_TRAGET_NOT_FOUND;
    }

   _data.algo.single[e_YAW].validFrame = ((m & FOE_INVALID) == 0);
   _data.algo.single[e_HORIZON].validFrame = _data.algo.single[e_YAW].validFrame || (_enforcedPitch < 100);
   _data.algo.single[e_ROLL].validFrame = ((m & ROLL_INVALID_SLC) == 0) && (_data.vehicle.speed > 0);

    bool okCamhDist = (_currDistanceToTarget < _targetParams.camhMaxDistance);
   _data.algo.single[e_CAM_HEIGHT].validFrame = _validTargets && okCamhDist && _data.algo.single[e_HORIZON].validFrame;

   for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
     if (_data.algo.single[i].validFrame) {
       _data.algo.single[i].validFrameNum++;
       _data.algo.single[i].validDistance += _data.vehicle.ds;
       _data.algo.single[i].validTime     += _data.vehicle.dt;
       _data.algo.single[i].status         = e_CALIB_CAL;
       _data.algo.total.validFrame = true;
     }
     else {
       _data.algo.single[i].status         = e_CALIB_PAUSED;
     }
   }

   if (_data.algo.total.validFrame) {
       _data.algo.total.validFrameNum++;
       _data.algo.total.validDistance += _data.vehicle.ds;
       _data.algo.total.validTime     += _data.vehicle.dt;
   }

    WmbcOutputIF::instance().toItrkFrameValidation(&_data);
    WMBC_PRINT(WmbcDbgStg::e_VALIDATE_FRAME, 3, "[validateFrame@SLC] valid foe: %d, speed: %d, driving: %d, radius: %d, yawrate: %d, straight: %d, accel: %d, "
               "valid roll: %d, valid targets: %d, pauseReason: %d (%s)",
               _data.algo.single[e_YAW].validFrame,
               (m & (e_SPEED_TOO_LOW | e_SPEED_TOO_HIGH)) == 0,
               (m & e_REVERSE_IS_ON) ==0,
               (m & e_RADIUS_TOO_SMALL) ==0,
               (m & e_YAWRATE_TOO_HIGH) ==0,
               (m & e_EM_STRAIGHT) == 0,
               (m & e_ACCELERATION_TOO_HIGH) ==0,
               _data.algo.single[e_ROLL].validFrame,
               _validTargets,
               m,
               dec2binStr(m, 14).c_str());
    WMBC_PRINT(WmbcDbgStg::e_VALIDATE_FRAME, 3, "[validateFrame@SLC] straight: (%.2f, %.2f, %.2f) <? (%.2f, %.2f, %.2f)",
               _data.em.ypr[0], _data.em.ypr[1], _data.em.ypr[2],
               _data.validParams.rotThreshold[0], _data.validParams.rotThreshold[1], _data.validParams.rotThreshold[2]);
    WMBC_PRINT(WmbcDbgStg::e_VALIDATE_FRAME, 3, "[validateFrame@SLC] valid: %d (%d, %d), yaw: %d (%d), hor: %d (%d), roll: %d (%d), camh: %d (%d)",
               _data.algo.total.validFrame, _data.algo.total.validFrameNum, ((_validTargetFramesNum > 0) ? _data.algo.total.validFrameNum - _validAlgoFrameNumAtTargetDetection : 0),
               _data.algo.single[e_YAW].validFrame, _data.algo.single[e_YAW].validFrameNum,
               _data.algo.single[e_HORIZON].validFrame, _data.algo.single[e_HORIZON].validFrameNum,
               _data.algo.single[e_ROLL].validFrame, _data.algo.single[e_ROLL].validFrameNum,
               _data.algo.single[e_CAM_HEIGHT].validFrame, _data.algo.single[e_CAM_HEIGHT].validFrameNum);
  }

  void FOEFinder_Stationless::updateConvergence() { // TODO: organize mess with inheritance
    if (_data.algo.total.conv) {
      return;
    }

    float trajLength = (_targetParams.maxDistance - _targetParams.minDistance);
    bool noDistance = me_fabsf(_currDistanceToTarget - INVALID_VAL) < EPS; // && _lastValidDistanceToTargetFrame > 0;
    bool useMaxTravelDist = (_convParams.maxTravelDist > 0);
    _isEndTraj = _isEndTraj || ((_currDistanceToTarget < _targetParams.minDistance) && (_data.vehicle.trajLengthAbs > trajLength));
    _isEndTraj = _isEndTraj || (noDistance && useMaxTravelDist && (_data.vehicle.trajLength > _convParams.maxTravelDist));
    _postEndTrajFrames += _isEndTraj ? 1 : 0;
    
    for (int i = 0; i < e_CAM_HEIGHT; ++i) { // e_CALIB_DOF_NUM; ++i) {
      _data.algo.single[i].conv = (_isEndTraj && _data.algo.single[i].validFrameNum > _convParams.minValidFrames);
      if (_data.algo.single[i].conv) {
        _data.algo.single[i].status = e_CALIB_OK;
      }
    }

    bool allConverged = (_data.algo.single[e_YAW].conv
                         && _data.algo.single[e_HORIZON].conv
                         && _data.algo.single[e_ROLL].conv
                         && _data.algo.single[e_CAM_HEIGHT].conv);
    _data.algo.total.conv = _isEndTraj && allConverged;

    // quality based on sample size
    _data.algo.total.quality = _data.algo.single[e_YAW].quality;
    for (int i = e_YAW; i < e_CALIB_DOF_NUM; ++i) {
      float quality_f = 0.f;
      if (SLC_QUALITY_MAX_FRAMES[i] > _convParams.minValidFrames) {
        float x = _data.algo.single[i].validFrameNum - _convParams.minValidFrames;
        float x0 = SLC_QUALITY_MAX_FRAMES[i] - _convParams.minValidFrames;
        quality_f = (x > x0) ? 1.f : std::max(0, std::min(1, x*(2*x0-x)/(x0*x0)));
      } else { // not reasonable, minValidFrames is too high
        quality_f =  std::max(0, std::min(1, _data.algo.single[i].validFrameNum/_convParams.minValidFrames));
      }
      _data.algo.single[i].quality = me_roundf(100*quality_f);
      if (_data.algo.total.quality > _data.algo.single[i].quality) {
        _data.algo.total.quality = _data.algo.single[i].quality;
      }
    }

    // progress
    // _data.algo.single[e_YAW].progress = _data.algo.single[e_YAW].conv ? 50 : std::min(50, me_roundf(50*_data.algo.single[e_YAW].validFrameNum / _convParams.minValidFrames));
    // _data.algo.single[e_HORIZON].progress = _data.algo.single[e_YAW].progress;
    // _data.algo.single[e_ROLL].progress = _data.algo.single[e_ROLL].conv ? 25 : std::min(25, me_roundf(25*_data.algo.single[e_ROLL].validFrameNum / _convParams.minValidFrames));
    // _data.algo.single[e_CAM_HEIGHT].progress = _data.algo.single[e_CAM_HEIGHT].conv ? 25 : std::max(0, std::min(25, me_roundf(25*(_targetParams.maxDistance - _currDistanceToTarget)/trajLength)));
    // _data.algo.total.progress = _data.algo.total.conv ? 100 : std::min(99, _data.algo.single[e_YAW].progress + _data.algo.single[e_ROLL].progress + _data.algo.single[e_CAM_HEIGHT].progress);
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      int progress = _data.algo.single[i].conv ? 100 : std::min(100, me_roundf(100*_data.algo.single[i].validFrameNum / _convParams.minValidFrames));
      _data.algo.single[i].progress = std::max(_data.algo.single[i].progress, progress);
    }
    int progressComp = std::min(99, 0.5*_data.algo.single[e_YAW].progress + 0.25*_data.algo.single[e_ROLL].progress + 0.25*_data.algo.single[e_CAM_HEIGHT].progress);
    int progressDist = std::max(0, std::min(99, me_roundf(100*(_targetParams.maxDistance - _currDistanceToTarget)/trajLength)));
    _data.algo.total.progress = _data.algo.total.conv ? 100 : std::min(progressDist, progressComp);

    WmbcOutputIF::instance().toItrkSlcConvergence(_isEndTraj, _validTargetFramesNum, &_data, &_convParams, &_targetParams);
    WMBC_PRINT(WmbcDbgStg::e_CONVERGENCE, 2, "[updateConvergence@SLC] distance to target: %.2f, traj len: %.2f, isEndTraj: %d (nodist: %d, lastVF: %d, maxTravelDist: %.2f)"
               ", postEndTraj: %d, progressComp: 0.5*%d + 0.25*%d + 0.25*%d = %d, progressDist = %d, progress: %d, converged: %d (%d, %d, %d, %d), quality: %d (%d, %d, %d, %d)",
               _currDistanceToTarget, _data.vehicle.trajLength, _isEndTraj, noDistance, _lastValidDistanceToTargetFrame,
               _convParams.maxTravelDist, _postEndTrajFrames,  _data.algo.single[e_YAW].progress,  _data.algo.single[e_ROLL].progress, _data.algo.single[e_CAM_HEIGHT].progress,
               progressComp, progressDist, _data.algo.total.progress,
               _data.algo.total.conv, _data.algo.single[e_YAW].conv, _data.algo.single[e_HORIZON].conv, _data.algo.single[e_ROLL].conv, _data.algo.single[e_CAM_HEIGHT].conv,
               _data.algo.total.quality, _data.algo.single[e_YAW].quality, _data.algo.single[e_HORIZON].quality, _data.algo.single[e_ROLL].quality, _data.algo.single[e_CAM_HEIGHT].quality);
  }

  void FOEFinder_Stationless::updateStatus() {
    ConvValidData &t = _data.algo.total;
    if (_data.algo.total.conv) {
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

    if (_convParams.maxTravelDist > 0 && _postEndTrajFrames > 2) {
      t.status = e_CALIB_TIMEOUT;
      return;
    }

    if (!_data.algo.total.validFrame && !_isEndTraj) {
      t.status = e_CALIB_PAUSED;
      return;
    }

    if (!_data.em.valid) { // never reached
      t.status = e_CALIB_RUN_ERROR;
      return;
    }

    t.status = e_CALIB_CAL;
  }

  bool FOEFinder_Stationless::isCalibInRange() {
    bool inRange = true;
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      inRange = inRange && (_calib[i] >= _data.metaParams.foeRange[i][0]);
      inRange = inRange && (_calib[i] <= _data.metaParams.foeRange[i][1]);
    }

    return inRange;
  }
  
  void FOEFinder_Stationless::setResults() {
    if (_data.vehicle.validFrameNum == 0) { // we are at start or after conv with no measurements yet
      setInitialResults_NoSeparation();
      return;
    }

    double angles[3] = {0.0, 0.0, 0.0};
    angles[e_YAW] = _calib[e_YAW];
    angles[e_HORIZON] = _calib[e_HORIZON];
    angles[e_ROLL] = _calib[e_ROLL];
    double camh = _calib[e_CAM_HEIGHT];
    _data.results.curr.update(angles, camh);

    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      setIsCalibInRange(i, _calib[i]);
    }

    setAllCamerasResults();
    setIsCalibInRange();
  }

  void FOEFinder_Stationless::setDebugShowData() {
    for (int i = 0; i < 2; ++i) {
      _debugShowData.targetSearchWindow[i] = _targetSearchWindow[i]*2;
    }
    
    _debugShowData.startDistanceToTarget = _targetParams.maxDistance;
    _debugShowData.targetWidthWorld = _targetParams.width;
    float distance = _validTargetFramesNum > 0 ? _distanceEstimator.getMeanPred() : _targetParams.maxDistance;
    float invDistance = 1.f / (distance > EPS ? distance : _targetParams.maxDistance);
    _debugShowData.targetWidthImage = _data.camera.focalLm2 * _targetParams.width * invDistance;
    
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      _debugShowData.validFramesNumIndividual[i] = _data.algo.single[i].validFrameNum;
      _debugShowData.validFramesNumIndividualPercentage[i] = _data.vehicle.validFrameNum > 0 ? me_roundf(100*_data.algo.single[i].validFrameNum/_data.vehicle.validFrameNum) : 0;
      _debugShowData.validFrameIndividual[i] = _data.algo.single[i].validFrame;
      _debugShowData.singleConverged[i] = _data.algo.single[i].conv;
      _debugShowData.singleStat[i] = _data.algo.single[i];
    }

    _debugShowData.currDistanceToTarget = _currDistanceToTarget;
    _debugShowData.trajLength = _data.vehicle.trajLength;
    _debugShowData.isEndTraj = _isEndTraj;
    _debugShowData.validFramesVehicle = _data.vehicle.validFrameNum;
    _debugShowData.validFramesVehiclePercentage = me_roundf(100*_data.vehicle.validFrameNum / (globalFrameIndex+1));

    _debugShowData.progress = _data.algo.total.progress;

    WmbcOutputIF::instance().toItrkDebug(&_data);
  }

  void FOEFinder_Stationless::fillModelIF() {
    FOEFinder::fillModelIF();
    
    WmbcIF *m = &_wmbcIF.editable();
    m->slcValidAlgoFrameNum = (_validTargetFramesNum > 0) ? _data.algo.total.validFrameNum - _validAlgoFrameNumAtTargetDetection : 0;
    m->slcTarget = _targetDraw;

    _wmbcIF.update();
  }
  
  // ------------------------------------------- Prints  -----------------------------------------------------------

  void FOEFinder_Stationless::printSP() {
#if defined (MEwin) && defined (WMBC_DEBUG_PRINT)
    int n = 2*_targetParams.spNumInTarget;
    int grab = *PrepSys_API::getGrabIndex(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
    float cosr = me_cosf(-_calib[e_ROLL]);
    float sinr = me_sinf(-_calib[e_ROLL]);

    fprintf(stderr, "csv-sp: %d %d ", globalFrameIndex, grab);
    for (int i = 0; i < n; ++i) {
        float xr = _spRect[i][0];
        float yr = _spRect[i][1];
        float x = cosr*xr - sinr*yr;
        float y = sinr*xr + cosr*yr;
        fprintf(stderr, "%.10f %.10f ", x, y);
    }
    fprintf(stderr, "\n");
#endif
  }

} // namespace WMBC
