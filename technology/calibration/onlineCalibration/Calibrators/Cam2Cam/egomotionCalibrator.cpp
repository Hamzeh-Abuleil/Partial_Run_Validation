/**
 * \file egomotionCalibrator.cpp
 * \brief Implementation of EgomotionCalibrator class.
 *
 * \author Amit Hochman
 * \date Dec 13, 2018
 */
#include "egomotionCalibrator.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibMatUtils.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibMath.h"
#include "technology/mobilib/float/common/MEmath/SVDmat.h"
#include "technology/mobilib/std/math/me_math_consts.h"
#include "technology/mobilib/std/me_math.h"
#include "technology/mobilib/std/math/sqrt.h"
#include "egomotionCovarianceMatrices.h"
#include <algorithm>
#ifdef CALIB_DEBUG
#include "technology/mobilib/fix/common/MEXstd/debug.h"
#include "technology/calibration/onlineCalibration/CalibUtils/MatArray.h"
#endif

using namespace CalibUtils;
namespace EgomotionCalibration
{
double butter_weight(double x, double xCritical, double xRoot, double exponent)
{
  // Computes a weight that is 1 at x = 1, 0 at x = xRoot, and like the bode-plot
  // of a butterworht filter, decreases monotonically, slowly at first,
  // and then linearly, after x > xCritical
  // exponent determines how sharp the 'knee' at xCritical is.
  double edxc = me_expd(exponent * xCritical);
  double edxe = me_expd(exponent * xRoot);
  double expTimesX = exponent * x;
  if ( expTimesX > 50 )
  {
    return -50.0;
  }
  double edx = me_expd(expTimesX);
  return 1 - me_logd((edx + edxc) / (1 + edxc)) / me_logd((edxe + edxc) / (1 + edxc));
}
void EgomotionCalibrator::reset(void)
{
  _R0 = Float::MEmath::identity<3, double>();
  _R = Float::MEmath::identity<3, double>();
  _RDynamic = Float::MEmath::identity<3, double>();
  _BAt = zeros<3, 3, double>();
  _BAtDynamic = zeros<3, 3, double>();
  _M = zeros<3, 3, double>();
  _t = zeros<3, double>();
  _V = zeros<3, double>();
  _U9 = zeros<9, double>();
  _A1tb1 = zeros<9, double>();
  _A2tb2 = zeros<9, double>();
  _K = zeros<9, 9, double>();
  _A1tA1 = zeros<9, 9, double>();
  _A2tA2 = zeros<9, 9, double>();
  _L93 = zeros<9, 3, double>();
  _nInlierFrames = 0;
  _gammaI = 0.0;
  _gammaII = 0.0;
  _outlierSuspects.clear();
  _invCovError = zeros<3, 3, double>();
  _bComputeErrorEstimate = true;
  _errorEstimate = 180.0;
  _confidence = 0.0;
  _dynamicNumFrames = 0;
  _dynamicOutliers = 0;
  std::fill(std::begin(_pastStates), std::begin(_pastStates) + _nPastStates, OnlineCalibration::State::SUSPECTED);
  _state = OnlineCalibration::State::SUSPECTED;
  _isOutlierEmConsistency = false;
  _isOutlierSolutionBased = false;
  mat44 I = me::identity<4, double>();
  updateStateInfo(I, I);
}

void EgomotionCalibrator::setCovErrorMatrices(mat33 const &covarianceCameraA, mat33 const &covarianceCameraB)
{
  _covErrorA = covarianceCameraA;
  _covErrorB = covarianceCameraB;
}

mat33 EgomotionCalibrator::compute_inv_cov_error_update(vec3 const &b) const
{
  mat33 W = pinv(_R0 * _covErrorA * _R0.transpose() + _covErrorB);

  mat33 out = zeros<3, 3, double>();
  for (unsigned int j = 0; j < 3; j++)
  {
    assignCol(out, j, cross(b, col(W, j)));
  }
  for (unsigned int j = 0; j < 3; j++)
  {
    assignRow(out, j, cross(b, row(out, j)));
  }
  return out;
}

void EgomotionCalibrator::update(mat44 const &emFrameA, mat44 const &emFrameB)
{
  // We work with [pitch, yaw, roll] vectors, and we take the sign of the pitch
  // to be the opposite of what is usually used in mobileye, i.e., according to a
  // right-hand convetion around the X axis. This choice makes
  // the [pitch, yaw, roll] vector an eigenvector (with eigenvalue = 1) of the
  // corresponding rotation matrix.

#ifdef CALIB_DEBUG
    static const std::string debugOutDir = Debug::Args::instance().getStickyValue("-sC2C_debugOutDir", "").c_str();
    if (!debugOutDir.empty())
    {
        static std::string outputFilenameA = debugOutDir + "/emA.dat";
        static std::string outputFilenameB = debugOutDir + "/emB.dat";
        static bool outputDeleted = false;
        if (!outputDeleted)
        {
            std::remove(outputFilenameA.c_str());
            std::remove(outputFilenameB.c_str());
            outputDeleted = true;
        }        
        MatArray<4, 4, double> tformA, tformB;
        tformA.data.push_back(emFrameA);
        tformA.append(outputFilenameA);
        tformB.data.push_back(emFrameB);
        tformB.append(outputFilenameB);
    }
#endif

  vec3 a, b, ta, tb;
  a = tform2PitchYawRoll(emFrameA);
  b = tform2PitchYawRoll(emFrameB);
  ta = tform2trvec(emFrameA);
  tb = tform2trvec(emFrameB);

  _dynamicNumFrames = _forgetFactor * _dynamicNumFrames + 1;
  if (!_outlierSuspects.empty() && _errorEstimate < _errorThreshold)
  {
    purgeOutliers();
  }
  bool isOutlierAny = false;
  bool isOutlierEmc = false;
  if (_doFilterOutliers)
  {
    isOutlierAny = isOutlier(a, b, emFrameA, emFrameB);
    isOutlierEmc = _isOutlierEmConsistency;
  }
  _dynamicOutliers = _forgetFactor * _dynamicOutliers + isOutlierEmc;
  if (!isOutlierEmc) // we don't want to reject based on the solution for the dynamic calibration
  {
    _BAtDynamic = _forgetFactor * _BAtDynamic + exteriorProd(b, a);
    updateDynamic();
  }
  if (isOutlierAny)
  {
    updateStateInfo(emFrameA, emFrameB);
    return;
  }

  _nInlierFrames++;

  // update _BAt matrix (b*a') and add it to the current _BAt
  _BAt = _BAt + exteriorProd(b, a);
  updateGuess();

  // update rotation equations
  mat33 I = me::identity<3, double>();
  _A1tA1 = _A1tA1 + kron(exteriorProd(a, a), I);
  _A1tb1 = _A1tb1 + vec(kron(me::Mat<3, 1>(a), me::Mat<3, 1>(b)));

  // update translation equations
  mat33 Rb = tform2rotm(emFrameB);
  _K = _K + kron(exteriorProd(ta, ta), I);
  _L93 = _L93 - kron(me::Mat<3, 1>(ta), I) * (Rb - I);
  _M = _M + 2.0 * I - Rb - Rb.transpose();
  _U9 = _U9 + vec(kron(me::Mat<3, 1>(ta), me::Mat<3, 1>(tb)));
  _V = _V - Rb.transpose() * tb + tb;
  // M is singular on the first iteration, and possibly a
  // few iterations afterwards if Rb does not change,
  // so we use pinv.
  mat33 pinvM = pinv(_M);
  _A2tA2 = _K - _L93 * pinvM * _L93.transpose();
  _A2tb2 = _U9 - _L93 * pinvM * _V;

  // update scalars needed for computing RMS of residues
  _gammaI = _gammaI + dot(b, b);
  _gammaII = _gammaII + dot(tb, tb);

  if (!_bComputeErrorEstimate || _nInlierFrames < 10) // We need at least a few good frames
  {
    updateStateInfo(emFrameA, emFrameB);
    return;
  }
  // Update _errorEstimate according to "Optimal estimation of
  // three-dimensional rotation and reliability evaluation"
  _invCovError = _invCovError + compute_inv_cov_error_update(b);

  double d = trace(pinv(_invCovError));
  if (d <= 0.0)
  {
    _errorEstimate = 180.0;
  }
  else
  {
    _errorEstimate = std::min(180.0, me_sqrtd(d) * DEGREES_PER_RADIAN);
  }
  updateStateInfo(emFrameA, emFrameB);
}

void EgomotionCalibrator::updateStateInfo(mat44 const &emFrameA, mat44 const &emFrameB)
{
  _stateInfo.setAllValues(
      (float)_translationWeight,
      (float)_doFilterOutliers,
      (float)_diffAnglesThreshold,
      (float)_diffProjTransThreshold,
      (float)_nInlierFrames,
      (float)_outlierSigmas,
      (float)_errorThreshold,
      (float)_errorEstimate,
      (float)_gammaI,
      (float)_gammaII,
      matd2f(_covErrorA),
      matd2f(_covErrorB),
      matd2f(_invCovError),
      matd2f(_R0),
      matd2f(_R),
      vecd2f(_t),
      matd2f(_BAt),
      matd2f(_M),
      matd2f(emFrameA),
      matd2f(emFrameB));
}

void EgomotionCalibrator::compute_residuals(double &rmsR, double &rmsT) const
{
  assert(_nInlierFrames > 0);
  vec9 r = vec(_R);
  rmsR = me_sqrtd(me_abs((dot(r, _A1tA1 * r) - 2 * dot(r, _A1tb1) + _gammaI) / (3 * _nInlierFrames)));
  double bIItbII = _gammaII - dot(_V, pinv(_M) * _V);
  rmsT = me_sqrtd(me_abs((dot(r, _A2tA2 * r) - 2 * dot(r, _A2tb2) + bIItbII) / (3 * _nInlierFrames)));
}

void EgomotionCalibrator::purgeOutliers(void)
{
  // compute errors of suspects with current R and t
  double rmsR, rmsT;
  compute_residuals(rmsR, rmsT);
  unsigned int nSuspects = _outlierSuspects.size();
#ifdef CALIB_DEBUG
  std::cerr << "Checking " << nSuspects << " suspects.\n";
#endif
  vec3 fr, ft;
  mat33 I = Float::MEmath::identity<3, double>();
  for (unsigned int i = 0; i < nSuspects; i++)
  {
    vec3 const &a = _outlierSuspects[i].a;
    vec3 const &b = _outlierSuspects[i].b;
    vec3 const &ta = _outlierSuspects[i].ta;
    vec3 const &tb = _outlierSuspects[i].tb;
    mat33 const &Rb = _outlierSuspects[i].Rb;

    fr = _R * a - b;
    ft = _R * ta - tb + (I - Rb) * _t;
    if (rms(fr) > _outlierSigmas * rmsR || rms(ft) > _outlierSigmas * rmsT) // actual outlier
    {
      _nInlierFrames--;
      _A1tA1 = _A1tA1 - kron(exteriorProd(a, a), I);
      _A1tb1 = _A1tb1 - kron(a, b);
      _K = _K - kron(exteriorProd(ta, ta), I);
      _L93 = _L93 + kron(ta, I) * (Rb - I);
      _M = _M - (2.0 * I - Rb - Rb.transpose());
      _U9 = _U9 - kron(ta, tb);
      _V = _V + (Rb.transpose() * tb - tb);
      _gammaI = _gammaI - dot(b, b);
      _gammaII = _gammaII - dot(tb, tb);
    }
  }
  mat33 pinvM = pinv(_M);
  _A2tA2 = _K - _L93 * pinvM * _L93.transpose();
  _A2tb2 = _U9 - _L93 * pinvM * _V;
  _outlierSuspects.clear();
}

bool EgomotionCalibrator::isOutlier(vec3 const &a, vec3 const &b, mat44 const &emFrameA, mat44 const &emFrameB)
{
  // invariants-based test
  double na = norm2(a);
  double nb = norm2(b);
  double diffAngle = me_abs(na - nb);
  vec3 ta = tform2trvec(emFrameA);
  vec3 tb = tform2trvec(emFrameB);
  double projectedTranslationA = 0;
  double projectedTranslationB = 0;
  if (na > 0)
  {
    projectedTranslationA = dot(a, ta) / na;
  }
  if (nb > 0)
  {
    projectedTranslationB = dot(b, tb) / nb;
  }
  double diffProjTrans = me_abs(projectedTranslationA - projectedTranslationB);
  _isOutlierEmConsistency = diffProjTrans > _diffProjTransThreshold || diffAngle > _diffAnglesThreshold;

  // residuals-based test
  _isOutlierSolutionBased = false;
  if (!_isOutlierEmConsistency && _nInlierFrames > 0)
  {
    double rmsR, rmsT;
    compute_residuals(rmsR, rmsT);
    vec3 fr = _R * a - b;
    mat33 Rb = tform2rotm(emFrameB);
    vec3 ft = _R * ta - tb + (Float::MEmath::identity<3, double>() - Rb) * _t;
    if (_errorEstimate < _errorThreshold)
    {
      _isOutlierSolutionBased = rms(fr) > _outlierSigmas * rmsR ||
                                rms(ft) > _outlierSigmas * rmsT;
    }
    else //check for suspicious EM
    {
      _isOutlierSolutionBased = false;
      if (rms(fr) > _outlierSigmas / 2 * rmsR || rms(ft) > _outlierSigmas / 2 * rmsT)
      {
        SuspectData suspect = {a, b, ta, tb, Rb};
        _outlierSuspects.push_back(suspect);
      }
    }
  }
  return _isOutlierEmConsistency || _isOutlierSolutionBased;
}

void EgomotionCalibrator::compute(void)
{
  if (_nInlierFrames < 2)
  {
    _R = Float::MEmath::identity<3, double>();
    _t = zeros<3, double>();
    return;
  }
  solveForR();

  // compute t from R
  _t = pinv(_M) * (_V - _L93.transpose() * vec(_R));

  updateConfidence();
#ifdef CALIB_DEBUG
  static const std::string debugOutDir = Debug::Args::instance().getStickyValue("-sC2C_debugOutDir", "").c_str();
  if (!debugOutDir.empty())
  {
    static std::string outputFilename = debugOutDir + "/computedC2C.dat";
    static bool outputDeleted = false;
    if (!outputDeleted)
    {
      std::remove(outputFilename.c_str());
      outputDeleted = true;
    }
    MatArray<4, 4, double> tform;
    tform.data.push_back(tformFromRAndT(_R, _t));
    tform.append(outputFilename);
  }
#endif
}

void EgomotionCalibrator::updateConfidence(void)
{
  double diffDynamic = rotmdiff(_R, _RDynamic);
  double dynamicOutlierRate = _dynamicNumFrames == 0 ? 0 : _dynamicOutliers / _dynamicNumFrames;
  double confErrorEstimate =
      butter_weight(_errorEstimate, _errorEstimateCritical, _errorEstimateRoot, _errorEstimateExp);
  double confOutlierRate =
      butter_weight(dynamicOutlierRate, _outlierRateCritical, _outlierRateRoot, _outlierRateExp);
  double confDiffDyanmic =
      butter_weight(diffDynamic, _diffDynamicCritical, _diffDynamicRoot, _diffDynamicExp);

  if (confOutlierRate < 0 || confDiffDyanmic < 0 || confErrorEstimate < 0)
  {
    _confidence = 0;
  }
  else
  {
    _confidence = confDiffDyanmic * confErrorEstimate * confOutlierRate;
  }
  // update state
  OnlineCalibration::State newState = OnlineCalibration::State::GOOD;
  if (_confidence < _stateThresholds[0])
  {
    newState = OnlineCalibration::State::SUSPECTED;
  }
  else if (_confidence < _stateThresholds[1])
  {
    newState = OnlineCalibration::State::UNVALIDATED;
  }

  for (unsigned int i = _nPastStates - 1; i > 0; i--)
  {
    _pastStates[i] = _pastStates[i - 1];
  }
  _pastStates[0] = newState;
  _state = odd_length_median(_pastStates);
  // If confidence is good except for difference from dynamic, reset.
  // We suspect that calibration has changed.
  if (confErrorEstimate > _resetThreshold && confOutlierRate > _resetThreshold && confDiffDyanmic < 0)
  {
    reset();
  }
}

void EgomotionCalibrator::solveForR(void)
{
  vec9 r0 = vec(_R0);
  vec3 vZeros = zeros<3, double>();

  mat99 P = _A1tA1 + _translationWeight * _A2tA2;
  vec9 y = _A1tb1 + _translationWeight * _A2tb2;

  vec3 r1, r2, r3, z;
  mat33 A1, A2, A3;
  Float::MEmath::Mat<9, 3> Ar0;
  for (unsigned int i = 0; i < 10; i++)
  {
    r1 = subvec<0, 2>(r0);
    r2 = subvec<3, 5>(r0);
    r3 = subvec<6, 8>(r0);
    // fill Ar0 like this:
    // Ar0 = [ -r2, -r3, vZeros; r1, vZeros, -r3; vZeros, r1, r2 ];
    // first column
    assign<0, 2>(Ar0, -r2);
    assign<3, 5>(Ar0, r1);
    assign<6, 8>(Ar0, vZeros);
    // second column
    assign<9 + 0, 9 + 2>(Ar0, -r3);
    assign<9 + 3, 9 + 5>(Ar0, vZeros);
    assign<9 + 6, 9 + 8>(Ar0, r1);
    // third column
    assign<18 + 0, 18 + 2>(Ar0, vZeros);
    assign<18 + 3, 18 + 5>(Ar0, -r3);
    assign<18 + 6, 18 + 8>(Ar0, r2);

    z = linsolve(Ar0.transpose() * (P * Ar0), Ar0.transpose() * (y - P * r0));
    _R = mat<3, 3>(r0 + Ar0 * z);
    r0 = vec(_R);
    if (norm2(z) < 1e-14)
    {
      break;
    }
  }
  _R = fixrotm(_R);
}

void EgomotionCalibrator::updateGuess(void)
{
  _R0 = fixrotm(_BAt);
}

void EgomotionCalibrator::updateDynamic(void)
{
  _RDynamic = fixrotm(_BAtDynamic);
}

mat33 EgomotionCalibrator::getRotation() const
{
  return _R;
}

mat33 EgomotionCalibrator::getDynamicRotation() const
{
  return _RDynamic;
}

double EgomotionCalibrator::getConfidence() const
{
  return _confidence;
}

OnlineCalibration::State EgomotionCalibrator::getState() const
{
  return _state;
}

vec3 EgomotionCalibrator::getTranslation() const
{
  return _t;
}

const OnlineCalibration::Cam2CamInternalStateInfo EgomotionCalibrator::getStateInfo() const
{
  return _stateInfo;
}
} // namespace EgomotionCalibration