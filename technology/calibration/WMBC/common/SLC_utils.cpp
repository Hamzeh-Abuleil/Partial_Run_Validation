#include "SLC_utils.h"
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "technology/mobilib/std/me_math.h"
#include "technology/calibration/WMBC/common/wmbc_dbg.h"
#include "technology/calibration/utilities/cameraProjections/distortionCorrection_algoAPI.h"


using namespace Float::MEmath;

namespace WMBC {

  static const int RANSAC_MIN_SAMPLE_NUM = 3;

  //bool bestFitLine(double *X, double *Y, int numPts, float *coeff, float &residual) {
  bool bestFitLine(double *X, double *Y, int numPts, float *coeff) {
    // best fit: Y = coeff0 * X + coeff1
    float sumX, sumY, sumX2, sumY2, sumXY;
    sumX = sumY = sumX2 = sumY2 = sumXY = 0.0;
    for (int i = 0; i < numPts; ++i) {
      sumX += X[i];
      sumY += Y[i];
      sumX2 += X[i] * X[i];
      sumY2 += Y[i] * Y[i];
      sumXY += X[i] * Y[i];
    }

    float det = numPts * sumX2 - sumX * sumX;
    if (det < 1E-6) {
      return false;
    }
  
    float invDet = 1.0/det;
    coeff[0] = invDet * (numPts * sumXY - sumX * sumY);
    coeff[1] = invDet * (sumX2 * sumY - sumX * sumXY);
    //residual = sumY2 - coeff[0] * sumXY - coeff[1] * sumY;
  
    return true;
  } 

  void bestFitLineSym(double *X, double *Y, int numPts, float *coeff) {
    // best fit: rho = Y*cos(theta) - X*sin(theta), with coeff0 = rho, coeff1 = cos(theta), coeff2 = sin(theta)
    // theta is angle of line with respect to x-axis, rho is the distance of line from origin
    const int MAX_NUM_PTS = 200;
    numPts = std::min(numPts, MAX_NUM_PTS);

    float meanX, meanY;
    meanX = meanY = 0.0;
    for (int i = 0; i < numPts; ++i) {
      meanX += X[i];
      meanY += Y[i];
    }

    meanX /= (numPts > 0) ? numPts : 1;
    meanY /= (numPts > 0) ? numPts : 1;

    double a = 0.0;
    double b = 0.0;
    double Xs[MAX_NUM_PTS], Ys[MAX_NUM_PTS];

    for (int i = 0; i < numPts; ++i) {
      Xs[i] = X[i] - meanX;
      Ys[i] = Y[i] - meanY;
      a += Xs[i]*Ys[i];
      b += Xs[i]*Xs[i] - Ys[i]*Ys[i];
    }
    a *= 2;

    // coeff[1] = 0.5 * me_atan2d(a, b);
    // coeff[0] = -meanX * me_sinf(coeff[1]) + meanY * me_cosf(coeff[1]);
    xyToHalfSinCos(a, b, coeff[1], coeff[2]);
    coeff[0] = -meanX * coeff[2] + meanY * coeff[1];
  } 

  void bestFitLineSym(double P[][2], int numPts, float *coeff) {
    const int MAX_NUM_PTS = 200;
    numPts = std::min(numPts, MAX_NUM_PTS);
    double X[MAX_NUM_PTS];
    double Y[MAX_NUM_PTS];
    for (int i = 0; i < numPts; ++i) {
      X[i] = P[i][0];
      Y[i] = P[i][1];
    }
    bestFitLineSym(X, Y, numPts, coeff);
  }

  void ransacBestLineSym(double *X, double *Y, int *idxBest, int numPts, float *coeff, float threshold) {
    if (numPts < RANSAC_MIN_SAMPLE_NUM) {
      return;
    }
    const int MAX_NUM_PTS = 200;
    const int ITER_NUM = 1000; // TODO: make function parameter? use nchoosek to get order of magnitude?
    int inliersNum = -1;
    
    int idxTmp[MAX_NUM_PTS]; // TODO: memset
    double Xbest[MAX_NUM_PTS], Ybest[MAX_NUM_PTS];

    for (int i = 0; i < ITER_NUM; ++i) {
      int idx[RANSAC_MIN_SAMPLE_NUM];
      idx[0] = rand() % numPts;
      idx[1] = idx[0];
      idx[2] = idx[0];
      while (idx[1] == idx[0]) {
        idx[1] = rand() % numPts;
        while (idx[2] == idx[0] || idx[2] == idx[1]) {
          idx[2] = rand() % numPts;
        }
      }

      double Xs[RANSAC_MIN_SAMPLE_NUM], Ys[RANSAC_MIN_SAMPLE_NUM];
      for (int j = 0; j < RANSAC_MIN_SAMPLE_NUM; ++j) {
        Xs[j] = X[idx[j]];
        Ys[j] = Y[idx[j]];
      }

      bestFitLineSym(Xs, Ys, RANSAC_MIN_SAMPLE_NUM, coeff);
      
      float res;
      int newInliersNum = 0;
      for (int j = 0; j < numPts; ++j) {
        // res = X[j]*me_sinf(coeff[1]) - Y[j]*me_cosf(coeff[1]);
        res = X[j]*coeff[2] - Y[j]*coeff[1];
        res = me_fabs(me_fabs(res) - me_fabs(coeff[0]));

        if (res < threshold) {
          idxTmp[newInliersNum++] = j;
        }
      }

      if (inliersNum < newInliersNum) {
        for (int j = 0; j < newInliersNum; ++j) {
          idxBest[j] = idxTmp[j];
        }
        for (int j = newInliersNum; j < numPts; ++j) {
          idxBest[j] = -1;
        }
        inliersNum = newInliersNum;
      }
      
      for (int i = 0; i < inliersNum; ++i) {
        Xbest[i] = X[idxBest[i]];
        Ybest[i] = Y[idxBest[i]];
      }
    }
    bestFitLineSym(Xbest, Ybest, inliersNum, coeff);
  }

  bool xyToHalfSinCos(double y, double x, float &cosHalf, float &sinHalf) {
    // input: y, x which defines tan(2a) = y/x
    // output: cos(a), sin(a)
    const double EPS = 1e-10;
    const float COS45 = 0.5*sqrt(2); 
    double ya = me_abs(y);
    double xa = me_abs(x);
    int ys = (y > 0) ? 1 : -1;
    int xs = (x > 0) ? 1 : -1;
    
    if (ya < EPS && xa < EPS) {
      return false;
    }

    if (xa < EPS) {
      cosHalf = COS45;
      sinHalf = ys * COS45;
      return true;
    }

    float t2 = ya / xa; // tan(2*theta)
    float c2 = xs / sqrt(1 + t2*t2); // cos(2*theta)
    cosHalf = sqrt(0.5*(1 + c2));
    sinHalf = ys * sqrt(0.5*(1 - c2)); 
    return true;
  }

  void ransacBestLineSymSp(SPInfoVec &sp, SPInfoVec &spBest, float threshold) {
    int ptNum = sp.size();
    if (ptNum < RANSAC_MIN_SAMPLE_NUM) {
      spBest.clear();
      for (int i = 0; i < ptNum; ++i) {
        spBest.push_back(sp[i]);
      }
    }
      
    const int MAX_NUM_PTS = 200;
    double x[MAX_NUM_PTS];
    double y[MAX_NUM_PTS];
    for (int i = 0; i < ptNum; ++i) {
      x[i] = sp[i].xSub;
      y[i] = sp[i].ySub;
    }
    int idxBest[MAX_NUM_PTS];
    float coeff[3] = {0.0f, 0.0f, 0.0f};
    ransacBestLineSym(x, y, idxBest, ptNum, coeff, threshold);

    spBest.clear();
    for (int i = 0; i < ptNum; ++i) {
      if (idxBest[i] > -1) {
        spBest.push_back(sp[idxBest[i]]);
      }
    }
  }

  Mat<3, 3, double> yprToRotAndDerivative(Vec<3, double> &ypr, int m, int n) {
    Mat<3, 3, double> R = identity<3, double>();
    double a2 = ypr[0]*ypr[0] + ypr[1]*ypr[1] + ypr[2]*ypr[2];
    // double a = me_sqrt(a2);
    double a = sqrt(a2);

    if (a < 1e-8) { // TODO: check analytically if derivatives are finite when a2->0
      return R;
    }

    Mat<3, 3, double> L, L2;
    getLogRotation(L, L2, ypr);


    double s = sinc(a);
    double c = cosinc(a);

    if (m < 0 && n < 0) { // calc rotation
      Mat<3, 3, double> id = identity<3, double>();
      R = id + s*L + c*L2;
      return R;
    }

    if (m < 0 && n >= 0) {
      m = n;
      n = 0;
    }

    double ia2 = 1/a2;
    double x = ypr[m];
    double s1 = 1 - a2*c - s;
    double c1 = s - 2*c; 
    Mat<3, 3, double> Lx, L2x;
    getLogRotationFirstDerivative(Lx, L2x, ypr, m);

    if (n < 0) { // calc 1st derivative
      R = s*Lx + c*L2x + x*ia2*(s1*L + c1*L2);
      return R;
    }

    // calc 2nd derivative
    double y = ypr[n];
    double ia4 = ia2*ia2;
    Mat<3, 3, double> Ly, L2y, L2xy;
    getLogRotationFirstDerivative(Ly, L2y, ypr, n);
    int p = (m+1)*(n+1);
    getLogRotationSecondDerivative(L2xy, p);

    R = c*L2xy + x*ia2*(s1*Ly + c1*L2y) + y*ia2*(s1*Lx + c1*L2x);
    R += -x*y*ia4 * ((2*a2*c + a2*c1 + 3*s1)*L + (4*c1 - s1)*L2);
        
    if (p == 1 || p == 4 || p == 9) {
      R += ia2 * (s1*L + c1*L2);
      s = 0;
    }
    return R;
  }

  double sinc(double x) {
    // if (x < 1e-8) {
    //   return 1;
    // }

    // return me_sin(x) / x;
    return 1 - 0.5*x*x;
  }

  double cosinc(double x) {
    // if (x < 1e-8) {
    //   return 0.5;
    // }

    // return (1 - me_cos(x)) / (x*x);
    return 0.5 * (1 - x*x/12);
  }

  void getLogRotation(Mat<3, 3, double> &L, Mat<3, 3, double> &L2, Vec<3, double> &ypr) {
    /*
        [0 -r  y]
    L = [r  0  p]
        [-y -p 0]
    
           [y^2+r^2 yp       pr    ]
    L^2 = -[yp      p^2+r^2 -yr    ]
           [pr      -yr     y^2+p^2]
    */
    L(0,0) = L(1,1) = L(2,2) = 0.0;
    L(0, 1) = -ypr[2];
    L(0, 2) = ypr[0];
    L(1, 2) = ypr[1];
    L(1, 0) = -L(0, 1);
    L(2, 0) = -L(0, 2);
    L(2, 1) = -L(1, 2);

    L2(0,0) = -ypr[0]*ypr[0] - ypr[2]*ypr[2];
    L2(0,1) = -ypr[0]*ypr[1];
    L2(0,2) = -ypr[1]*ypr[2];
    L2(1,1) = -ypr[1]*ypr[1] - ypr[2]*ypr[2];
    L2(1,2) = ypr[0]*ypr[2];
    L2(2,2) = -ypr[0]*ypr[0] - ypr[1]*ypr[1];
    L2(1, 0) = L2(0, 1);
    L2(2, 0) = L2(0, 2);
    L2(2, 1) = L2(1, 2);
  }

  void getLogRotationFirstDerivative(Mat<3, 3, double> &Lx, Mat<3, 3, double> &L2x, Vec<3, double> &ypr, int n) {
    Vec<3, double> zeroV = zeros<3, double>();
    Lx = diag(zeroV);

    switch (n) {
    case 0:
      Lx(0,2) = 1;
      Lx(2,0) = -1;
      L2x(0,0) = -2*ypr[0];
      L2x(0,1) = -ypr[1];
      L2x(0,2) = 0.0;
      L2x(1,1) = 0.0;
      L2x(1,2) = ypr[2];
      L2x(2,2) = -2*ypr[0];
      L2x(1, 0) = L2x(0, 1);
      L2x(2, 0) = L2x(0, 2);
      L2x(2, 1) = L2x(1, 2);
      break;
    case 1:
      Lx(1,2) = 1;
      Lx(2,1) = -1;
      L2x(0,0) = 0.0;
      L2x(0,1) = -ypr[0];
      L2x(0,2) = -ypr[2];
      L2x(1,1) = -2*ypr[1];
      L2x(1,2) = 0.0;
      L2x(2,2) = -2*ypr[1];
      L2x(1, 0) = L2x(0, 1);
      L2x(2, 0) = L2x(0, 2);
      L2x(2, 1) = L2x(1, 2);
      break;
    case 2:
      Lx(0,1) = -1;
      Lx(1,0) = 1;
      L2x(0,0) = -2*ypr[2];
      L2x(0,1) = 0.0;
      L2x(0,2) = -ypr[1];
      L2x(1,1) = -2*ypr[2];
      L2x(1,2) = ypr[0];
      L2x(2,2) = 0.0;
      L2x(1, 0) = L2x(0, 1);
      L2x(2, 0) = L2x(0, 2);
      L2x(2, 1) = L2x(1, 2);
      break;      
    }
  }

  void getLogRotationSecondDerivative(Mat<3, 3, double> &L2xy, int p) {
    Vec<3, double> zeroV = zeros<3, double>();
    L2xy = diag(zeroV);

    switch (p) {
    case 1:
      L2xy(0,0) = L2xy(2,2) = -2;
      break;
    case 2:
      L2xy(0,1) = L2xy(1,0) = -1;
      break;
    case 3:
      L2xy(1,2) = L2xy(2,1) = 1;
      break;
    case 4:
      L2xy(1,1) = L2xy(2,2) = -2;
      break;
    case 6:
      L2xy(2,0) = L2xy(0,2) = -1;
      break;
    case 9:
      L2xy(0,0) = L2xy(1,1) = -2;
      break;
    }
  }

  // copy from the late sfmUtils
  void log3(const Float::MEmath::Mat<3, 3, double>& rotMat, Float::MEmath::Mat<3, 3, double>& logMat) {
    // R eigen values are [1 exp(i*theta) exp(-i*theta)] so S=R-R' eigen-values are [0 i*sin(theta) -i*sin(theta)] and S is assymetric.
    // In exp3 we get the formula : L=f(S)=S^2*((c-a)/(t^2))+S*(b/t)+I*c where here t=sin(theta).
    // here a=0,b=theta,c=0 so L=S*theta/sin(theta)
    // (The result is also direct since [0 i*sin(theta) -i*sin(theta)]*theta/sin(theta)=[0 i*theta -i*theta] which are the L eigen-values.
    // for theta calculation, see in getAngleOfRotationFromRotMat

    //assert(isRotMat(rotMat)); temporary remove - for memem usage
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        logMat(i, j) = (rotMat(i, j) - rotMat(j, i)) / 2;
      }
    }
    double val = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        val += logMat(i, j) * logMat(i, j);
      }
    }
    double theta = me_asin(sqrt(val / 2));
    // assert(theta == getAngleOfRotationFromRotMat(rotMat));
    double sinTheta = me_sin(theta);
    double factor = (me_fabs(theta) > 1e-8) ? (theta / sinTheta) : (1);

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        logMat(i, j) = logMat(i, j) * factor;
      }
    }
  }

  Vec<3, double> rotationToYawPitchRoll(Float::MEmath::Mat<3, 3, double> R) {
    Float::MEmath::Vec<3, double> ypr;
    Float::MEmath::Mat<3, 3, double> logR;

    log3(R, logR);

    ypr[0] = logR(0, 2); // yaw
    ypr[1] = logR(1, 2); // pitch
    ypr[2] = -logR(0, 1); // roll

    return ypr;
  }

  void bubbleSort_f(MEtypes::ptr_vector<float>& fvec) {
    int size = fvec.size();
    for (int i = size; i > 0; --i) {
      for (int j = 0; j < i - 1; ++j) {
        if (fvec[j] > fvec[j+1]) {
          float tmp = fvec[j];
          fvec[j] = fvec[j+1];
          fvec[j+1] = tmp;
        }
      }
    }
  }

  

  float median_f(MEtypes::ptr_vector<float>& fvec) {
    int size = fvec.size();
    if (size < 2) {
      return 0.f;
    }
    bubbleSort_f(fvec);
    return fvec[me_lround(0.5*size)];
  }

  bool distortionTransform(SaddlePoints::Types::SPInfo &sp, bool doUndist) {
    if (!DistortionCorrectionAPI::isDistortionValid()) {
      return false;
    }
    // bool (*warp)(const CameraInfo::CameraInstance, const int, const float ,const float, float&, float&) = doUndist ? DistortionCorrectionAPI::rectifySafe : DistortionCorrectionAPI::unrectifySafe;
    float dx = sp.xSub;
    float dy = sp.ySub;
    // bool ok = warp(CameraInfo::e_FORWARD, -2, dx, dy, sp.xSub, sp.ySub);
    bool ok = false;
    if (doUndist) {
      ok = DistortionCorrectionAPI::rectifySafe(CameraInfo::e_FORWARD, -2, dx, dy, sp.xSub, sp.ySub);
    } else {
      ok = DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, dx, dy, sp.xSub, sp.ySub);
    }
    return ok;
  }

  bool distortionTransform(SPInfoVec &sp, bool doUndist) {
    bool ok = false;
    unsigned int size = sp.size();
    if (size == 0) {
      return ok;
    }

    for (unsigned int i = 0; i < size; ++i) {
      bool oki = distortionTransform(sp[i], doUndist);
      ok = ok || oki;
    }
    return ok;
  }

  bool rectify(SPInfoVec &sp) {
    return distortionTransform(sp, true);
  }

  bool unrectify(SPInfoVec &sp) {
    return distortionTransform(sp, false);
  }

  bool rectify(SaddlePoints::Types::SPInfo &sp) {
    return distortionTransform(sp, true);
  }
  
  bool unrectify(SaddlePoints::Types::SPInfo &sp) {
    return distortionTransform(sp, false);
  }  

  LinearKalman1D::LinearKalman1D() {
    reset(0.f, 0.f);
  }

  LinearKalman1D::LinearKalman1D(float procNoise, float measNoise) {
    reset(procNoise, measNoise);
  }

  void LinearKalman1D::reset(float procNoise, float measNoise) {
    _state.reset();
    _meas = INVALID_VAL;
    // _Q = DE_PROC_NOISE;
    // _R = DE_MEAS_NOISE;
    _Q = procNoise;
    _R = measNoise;
    _init = false;
  }

  void LinearKalman1D::init(float z) {
    _meas = z;
    _state.mean = z;
    _state.meanPred = z;
    _state.cov = _Q;
    _state.covPred = _R;
    _init = true;
  }

  void LinearKalman1D::predict(float u, bool copyc2p) {
    if (!_init) {
      return;
    }
    if (copyc2p) {
      _state.meanPred = _state.mean;
      _state.covPred = _state.cov + _Q;
      DEBUG_LOG_CLR_FRAME("[predict@KF] copy correct to predict", WmbcDbgClr::e_PURPLE);
      return;
    }

    float x = _state.mean;
    float P = _state.cov;

    float xm = x + u;
    float Pm = P + _Q;

    _state.meanPred = xm;
    _state.covPred = Pm;
    DEBUG_LOG_CLR_FRAME("[predict@KF] u: " << u << ", x: " << x << ", P: " << P << ", Q: " << _Q
                        << ", xm: " << xm << ", Pm: " << Pm, WmbcDbgClr::e_PURPLE);
  }

  void LinearKalman1D::correct(float z, bool copyp2c) {
    _meas = z;
    if (!_init) {
      return;
    }
    if (copyp2c) {
      _state.mean = _state.meanPred;
      _state.cov = _state.covPred;
      DEBUG_LOG_CLR_FRAME("[correct@KF] copy predict to correct", WmbcDbgClr::e_PURPLE);
      return;
    }

    float xm = _state.meanPred;
    float Pm = _state.covPred;

    float K = (_R > EPS) ? Pm / (Pm + _R) : 1.f;
    float x = xm + K*(z - xm);
    float P = (1.f - K)*Pm;

    _state.mean = x;
    _state.cov = P;
    DEBUG_LOG_CLR_FRAME("[correct@KF] z: " << z << ", xm: " << xm << ", Pm: " << Pm << ", R: " << _R
                        << ", K: " << K << ", x: " << x << ", P: " << P, WmbcDbgClr::e_PURPLE);
  }


} // namespace WMBC
