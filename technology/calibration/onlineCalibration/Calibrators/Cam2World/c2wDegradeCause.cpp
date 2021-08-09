/**
 * \file c2wDegradeCause.cpp
 * \brief DegradeCause
 *
 * \author Uri London
 * \date May 11, 2020
 */

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wDegradeCause.h"

namespace OnlineCalibration {
  namespace Cam2World {

    Cam2WorldSignalDegradeReporter::Cam2WorldSignalDegradeReporter(): _mask(0) {
      _buff.alloc(CONVOLVE_WIN_SIZE);
      reset();
    }

    void Cam2WorldSignalDegradeReporter::reset() {
      _mask = 0;
      _buff.clear();
    }

    void Cam2WorldSignalDegradeReporter::update(int invalidMask) {
      // if this function is entered the assumption is that the frame is invalid
      DegradeFrameProperties p;
      p.emValid = ((invalidMask & e_EM_INVALID) == 0);
      p.rmValid = ((invalidMask & e_RM_INVALID) == 0);
      p.validFrame = !(p.emValid && p.rmValid);
      p.stable = false;
      p.highConf = false;

      _buff.push_back(p);
      update();
    }

    void Cam2WorldSignalDegradeReporter::update(float conf, bool stability) {
      // TODO: after adding em-epi conf use sma for all signals
      // and then update both
      DegradeFrameProperties p;
      if (stability) {
        p.stable = (conf > DEGRADE_CONF_TH);
        p.highConf = true;
      } else {
        p.highConf = (conf > DEGRADE_CONF_TH);
        p.stable = true;
      }

      _buff.push_back(p);
      update();
    }

    void Cam2WorldSignalDegradeReporter::update() {
      unsigned int size = _buff.size();
      float ratio[5] = {0.f, 0.f, 0.f, 0.f, 0.f};

      for (unsigned int i=0; i<size; ++i) {
        ratio[0] += _buff[-i].emValid    ? 1.f : 0.f;
        ratio[1] += _buff[-i].rmValid    ? 1.f : 0.f;
        ratio[2] += _buff[-i].stable     ? 1.f : 0.f;
        ratio[3] += _buff[-i].highConf   ? 1.f : 0.f;
        ratio[4] += _buff[-i].validFrame ? 1.f : 0.f;
      }
      float invConvWinSize = 1.f/CONVOLVE_WIN_SIZE;
      for (unsigned int i=0; i<5; ++i) {
        ratio[i] *= invConvWinSize;
      }

      // These are the enums in OnlineCalibrationDefs.h
      const float MIN_RATIO = 0.5f;
      _mask = (int)StateDegradeCause::NO_DEGRADE;

      if (ratio[0] < MIN_RATIO) { 
        _mask |= (int)StateDegradeCause::EM_INVALID;
      }
      if (ratio[1] < MIN_RATIO) { 
        _mask |= (int)StateDegradeCause::RM_INVALID;
      }
      if (ratio[4] < MIN_RATIO) { 
        _mask |= (int)StateDegradeCause::TIMEOUT;
      }

      if ((_mask & DEGRADE_FRAME_VALID) != 0) {
        return;
      }

      if (ratio[2] < MIN_RATIO) { 
        _mask |= (int)StateDegradeCause::CALIB_CHANGED;
      }
      if (ratio[3] < MIN_RATIO) { 
        _mask |= (int)StateDegradeCause::HIGH_VARIANCE;
      }
    }

  } // Cam2World
} // OnlineCalibration
