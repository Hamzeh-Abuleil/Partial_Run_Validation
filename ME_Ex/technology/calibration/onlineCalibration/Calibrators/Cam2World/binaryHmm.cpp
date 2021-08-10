/**
 * \file binaryHmm.cpp
 * \brief BinaryHmm implementation
 * Equtations derivation: http://wiki.mobileye.com/mw/index.php/Binary_HMM
 *
 * \author Uri London
 * \date Jul 28, 2019
 */


#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/binaryHmm.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wConsts.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wOutputIF.h"

namespace OnlineCalibration {
  namespace Cam2World {

    float activationFunc(float x, ActivationFuncParams params) {
      float a  = params.xlimits[0];
      float b  = params.xlimits[1];
      float y0 = params.zero ? 0 : -1;
      float w  = params.weight;
      float y  = 0;

      if (me_abs(a-b) < EPS) {
        return y;
      }

      if (b < a) {
        // TODO: this together with params.up is stupid and redundant
        a = params.xlimits[1];
        b = params.xlimits[0];
      }

      if (params.zero) {
        y = params.up ? x-a : b-x;
      } else {
        y = params.up ? 2*x-a-b : a+b-2*x;
      }
      y /= (b-a);

      y = std::max(y0, std::min(1, y));

      switch (params.mode) {
        case e_SCALE_LINEAR:
          return y;
          break;
        case e_SCALE_POW:
          if (me_fabs(w-1) > 1E-2) {
            y = me_powf(y, w);
          }
          return y;
          break;
        case e_SCALE_EXP: // TODO: check this and sigmoid
          return me_expf(w*y);
          break;
        case e_SCALE_SIGMOID:
          return  1.f/(1.f + me_exp(-w*y));
          break;
        default:
          return y;
      }
    }

    BinaryHmm::BinaryHmm() : _rate(0.5f), _pEq(0.5f), _p(0.f) {}
    BinaryHmm::BinaryHmm(float rate, float pEq, float p0) : _rate(rate), _pEq(pEq), _p(p0) {}

    float BinaryHmm::update(float llr, bool copy) {
      float lr = me_exp(llr);
      _predict();
      _correct(lr, copy);
      return _p;
    }

    float BinaryHmm::update() {
      return update(1.f, true);
    }

    void BinaryHmm::_predict() {
    // TODO: should i keep separate member for predict for debugging?
    // Equtations derivation: http://wiki.mobileye.com/mw/index.php/Binary_HMM
#ifdef MEwin
      float pc = _p;
#endif
      _p = _p + _rate*(_pEq - _p);
      OC_C2W_PRINT(e_HMM, e_BO_YELLOW,
                   "[Hmm::predict] p=%.2f+%.2f(%.2f - %.2f) = %.2f",
                   pc, _rate, _pEq, pc, _p);
    }

    void BinaryHmm::_correct(float lr, bool copy) {
      if (copy) {
        OC_C2W_PRINT(e_HMM, e_BO_YELLOW,
                     "[Hmm::correct] p=%.2f (no meas)", _p);
        return;
      }

      // Equtations derivation: http://wiki.mobileye.com/mw/index.php/Binary_HMM
#ifdef MEwin
      float pr = _p;
#endif
      _p = lr*_p/(1 + (lr-1)*_p);
      OC_C2W_PRINT(e_HMM, e_BO_YELLOW,
                   "[Hmm::correct] p=%.2f*%.2f/[1 + (%.2f-1)%.2f]=%.2f",
                   lr, pr, lr, pr, _p);
    }

  } // Cam2World
} // OnlineCalibration
