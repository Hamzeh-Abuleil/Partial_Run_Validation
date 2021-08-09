/**
 * \file binaryHmm.h
 * \brief BinaryHmm header
 *
 * \author Uri London
 * \date Jul 28, 2019
 */


#ifndef __OC_CAM2WORLD_BHMM_H
#define __OC_CAM2WORLD_BHMM_H

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wConsts.h"

namespace OnlineCalibration {
  namespace Cam2World {

    // TODO: move activationFunc to utils
    struct ActivationFuncParams {
      ActivationFuncParams() : weight(1.f), zero(true), up(true), mode(e_SCALE_LINEAR) 
        { xlimits[0]=0.f; xlimits[1]=1.f;}

      float xlimits[2]; // x-values where function is cutoff
      float weight; // decay factor of non-linear function (typcal 1/decay length)
      bool zero; // does the lower cuttof is at 0 (else -1)
      bool up; // function direction is from 0 to 1 (else from 1 to 0)
      //bool linear; // linear function from 0 to 1 (else a sigmoid)
      int mode; // scaling mode
    };

    float activationFunc(float x, ActivationFuncParams params);

    class BinaryHmm {
      public:
        BinaryHmm();
        BinaryHmm(float rate, float pEq, float p0);
        ~BinaryHmm() {}

        void reset(float p0=0.f) { _p=p0;}
        float update(float llr, bool copy=false);
        float update();
        float p() const { return _p;}
        float rate() const { return _rate;}
        float pEq() const { return _pEq;}
        //float p0() const { return _p0;}
        void rate(float rate) { _rate = rate;}
        void pEq(float pEq) { _pEq = pEq;}

      private:
        void _predict();
        void _correct(float lr, bool copy=false);

        float _rate; // decay rate towards equilibrium
        float _pEq; // equilibrium probability
        //float _p0; // initial probability
        float _p;
    };

  } // Cam2World
} // OnlineCalibration

#endif // __OC_CAM2WORLD_BHMM_H
