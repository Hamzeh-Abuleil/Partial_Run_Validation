#ifndef _WMBC_SLC_UTILS_H_
#define _WMBC_SLC_UTILS_H_


#include "technology/mobilib/float/common/MEmath/mat.h"
#include "technology/calibration/WMBC/common/slcTypes.h"

// namespace Float {
//   namespace MEmath {
//     template <int ROWS, class T = double> class Vec;
//     template <int ROWS, int COLS, class T = double> class Mat;
//   }
// }

namespace WMBC {
  bool bestFitLine(double *X, double *Y, int numPts, float *coeff);
  void bestFitLineSym(double *X, double *Y, int numPts, float *coeff);
  void bestFitLineSym(double P[][2], int numPts, float *coeff);
  void ransacBestLineSym(double *X, double *Y, int *idxBest, int numPts, float *coeff, float threshold = 0.5);
  void ransacBestLineSymSp(SPInfoVec &sp, SPInfoVec &spBest, float threshold = 0.5);
  bool xyToHalfSinCos(double y, double x, float &cosHalf, float &sinHalf);
  Float::MEmath::Mat<3, 3, double> yprToRotAndDerivative(Float::MEmath::Vec<3, double> &ypr, int m = -1, int n = -1);
  double sinc(double x);
  double cosinc(double x);
  void getLogRotation(Float::MEmath::Mat<3, 3, double> &L, Float::MEmath::Mat<3, 3, double> &L2, Float::MEmath::Vec<3, double> &ypr);
  void getLogRotationFirstDerivative(Float::MEmath::Mat<3, 3, double> &Lx, Float::MEmath::Mat<3, 3, double> &L2x, Float::MEmath::Vec<3, double> &ypr, int n);
  void getLogRotationSecondDerivative(Float::MEmath::Mat<3, 3, double> &L2xy, int p);
  void log3(const Float::MEmath::Mat<3, 3, double>& rotMat, Float::MEmath::Mat<3, 3, double>& logMat);
  Float::MEmath::Vec<3, double> rotationToYawPitchRoll(Float::MEmath::Mat<3, 3, double> R);

  void bubbleSort_f(MEtypes::ptr_vector<float>& fvec);
  float median_f(MEtypes::ptr_vector<float>& fvec);

  bool rectify(SPInfoVec &sp);
  bool unrectify(SPInfoVec &sp);
  bool rectify(SaddlePoints::Types::SPInfo &sp);
  bool unrectify(SaddlePoints::Types::SPInfo &sp);

  struct KalmanState {
    float mean;
    float cov;
    float meanPred;
    float covPred;

    KalmanState() { reset();}
    void reset() { mean = INVALID_VAL; cov = INVALID_VAL; meanPred = INVALID_VAL; covPred = INVALID_VAL;}
  };

  class LinearKalman1D {
  public:
    LinearKalman1D();
    LinearKalman1D(float procNoise, float measNoise);

    float getDistance() { return _state.mean;}
    float getMean() { return _state.mean;}
    float getMeanPred() { return _state.meanPred;}
    float getCov() { return _state.cov;}
    float getCovPred() { return _state.covPred;}
    float getMeas() { return _meas;}
    bool wasInit() { return _init;}

    void reset(float procNoise, float measNoise);
    void init(float z);
    void predict(float u, bool copyc2p = false);
    void correct(float z, bool copyp2c = false);

  private:
    KalmanState _state;
    float _meas;
    float _Q; // process noise;
    float _R; // measurement noise;
    bool _init;
  };

}

#endif // _WMBC_SLC_UTILS_H_
