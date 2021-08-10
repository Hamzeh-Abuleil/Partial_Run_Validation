/*
 * stationlessTargetFinder.h
 *
 *  Created on: Nov 19, 2015
 *      Author: urilo
 */

#ifndef STATIONLESS_TARGET_FINDER_H_
#define STATIONLESS_TARGET_FINDER_H_

// namespace Float {
//   namespace MEmath {
//     template <int ROWS, class T = double> class Vec;
//     template <int ROWS, int COLS, class T = double> class Mat;
//   }
// }

#include "technology/mobilib/float/common/MEmath/mat.h" // TODO: how to declare templates properly?
#include "technology/calibration/WMBC/common/slcTypes.h"
// #include "technology/calibration/TAC2/TAC2_types.h"
// #include "technology/calibration/TAC2/SLC_types.h"

namespace SaddlePoints {namespace Types {struct SPInfo;}}

namespace WMBC {
  class StationlessTargetFinder_Debugger;

  class StationlessTargetFinder {
    friend class StationlessTargetFinder_Debugger;
  public:
    //StationlessTargetFinder(TFParams &params, OriginData *origins);
    //StationlessTargetFinder() {} // workaround of initing STF in SLC
    StationlessTargetFinder();
    ~StationlessTargetFinder() {}

    // void init(const Fix::MEimage::Image *img);
    void init(const Prep::SafeImg *img);
    //bool run(Types::SPInfoVec &rawSp, Float::MEmath::Vec<3, double> *approvedSp); 
    bool run(SPInfoVec &rawSp);
    float getImageTargetWidth() { return _imageTargetWidth;}
    float getSquareSize(int side) const { return _squareSize[side];}
    float getSquareSize() const { return 0.5*(_squareSize[LEFT]+_squareSize[RIGHT]);}
    void setParams(TFParams &params) { _params = params;}
    void setParams(TFParams &params, OriginData *origins) { _params = params; _origins = origins;}
    void setDebugger();

  private:
    bool findPolesCands();
    bool findTwoPoles();
    void reset();
    bool eliminateOutliersBySign(int side);
    void estimateSquareSize(int side);
    bool finalizeTarget(int side);
    int buildTargetCandidate(TargetCandInfo &cand, int side, int iStart);
    void buildAndScoreTargetCandidate(TargetCandInfo &cand, int side, int idxValidStart, bool doFloor);
    void invalidateNonTargetPoints(int side, TargetCandInfo &chosenCand);
    void fillMissingPts(int side, TargetCandInfo &chosenCand);

    void debugPrintPts(SPInfoVec &sp, std::string msg, bool skipLowScore);
    void debugPrintPtsShort(SPInfoVec &sp, std::string msg, bool skipLowScore, bool isSub);
    void printXbins(unsigned int *xbins, const int XBIN_NUM, const int XBIN_RAD);

    SPInfoVec _tmpSp;    
    SPInfoVec _candsSp;
    SPInfoVec _poles[SIDE_NUM];
    int _polesPtNum[SIDE_NUM];
    //float _polesSquareSize[SIDE_NUM];    
    float _imageTargetWidth;
    bool _useDebugPrints;
    // const Fix::MEimage::Image *_img;
    TFParams _params;
    const Prep::SafeImg *_img;
    int _xBinRad;
    float _tfRansacTh;
    float _squareSize[SIDE_NUM];
    OriginData *_origins;
    StationlessTargetFinder_Debugger *_debugger;
    bool _useSpRefine;
  };

} // namespace WMBC

#endif // STATIONLESS_TARGET_FINDER_H_

