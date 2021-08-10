/*
 * slc.h
 *
 *  Created on: Jul 03, 2016
 *      Author: urilo
 */

#ifndef _WMBC_SLC_H_
#define _WMBC_SLC_H_

#include "technology/calibration/WMBC/common/wmbc.h"
#include "technology/calibration/WMBC/common/slcTypes.h"
#include "technology/calibration/WMBC/common/stationlessTargetFinder.h"
// #include "technology/mobilib/float/common/MEgeo/point.h"
// #include "technology/calibration/WMBC/interface/wmbcIF.h"
// #include "technology/calibration/WMBC/common/wmbcTypes.h"
// #include "technology/mobilib/fix/common/MEXimage/sync.h"
// #include "technology/mobilib/fix/common/MEXimage/typeImages.h"
#include "technology/calibration/WMBC/common/SLC_utils.h"

namespace WMBC {

  class FOEFinder_Stationless_Debugger;

  class FOEFinder_Stationless : public FOEFinder {
    friend class FOEFinder_Stationless_Debugger;
  public:
    FOEFinder_Stationless(WmbcProperties *properties);
    virtual ~FOEFinder_Stationless();
    virtual void runTargets();
    virtual void run();
    
  private:
    virtual void setParams(WmbcProperties *properties);
    void findSp();
    void verifySaddlePts();
    void setTarget();
    virtual void validateFrameAlgo();
    void calcFoe();
    // virtual void calcRollAndCamHeight();
    void calcRoll();
    void calcCamHeight();
    void computeCamh();
    void computeCamhProjMean();
    void computeCamhLinearRegression();
    void fillCamhDataTargetCenter();
    void updateConvergence();
    virtual void updateStatus();
    virtual bool isCalibInRange();
    virtual void setResults();
    virtual void fillModelIF();
    virtual void setDebugShowData();
    void printSP();

    float _calib[e_CALIB_DOF_NUM];
    HistogramOneDim _hist[e_CALIB_DOF_NUM];

    SPInfoVec _sp;
    float _spRect[2*MAX_SP_NUM_IN_TARGET][2];
    Fix::MEimage::Rect _targetSearchWindow[2];
    StationlessTargetFinder _targetFinder;
    TargetDrawInfo _targetDraw;
    bool _validTargets;
    TargetParams _targetParams;
    SlcConvParams _convParams;
    int _validTargetFramesNum;
    int _successiveNonValidTargetFramesNum;
    int _firstValidFrame;
    int _validAlgoFrameNumAtTargetDetection;
    float _currDistanceToTarget;
    float _lastValidDistanceToTarget;
    int _lastValidDistanceToTargetFrame;
    bool _isEndTraj;
    int _postEndTrajFrames; // how many frames past end of trajectory TODO: replace _isEndTraj
    float _imageTargetWidth;
    float _invImageTargetWidth;
    CamhDataVec _camhData;
    LinearKalman1D _distanceEstimator;
    LinearKalman1D _targetImgPosEstimator[4]; // estimator of left-center and right-center pt (x, y)
    FOEFinder_Stationless_Debugger *_debugger;
    float _enforcedPitch;
  };

} // namespace WMBC

#endif
