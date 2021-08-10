/**
 * \file c2wSPC.cpp
 * \brief special implementation for SPC mode
 *
 * \author Uri London
 * \date Jul 22, 2019
 */


#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wSignal.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/steadyStateCalibrator.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wOutputIF.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wUtils.h"

namespace OnlineCalibration {
  namespace Cam2World {

    void Cam2WorldSignal::spcUpdateConvergence() {
      float spcMaxVar = _id == e_CAM_HEIGHT ? SPC_MAX_VAR_DIST : SPC_MAX_VAR_ANG;

      static int SPC_MIN_SAMPLE_usr = Debug::Args::instance().getStickyValue("-sSPC-startResetFrame", SPC_MIN_SAMPLE);
      bool sampleOk = (_hist.sampleNum() >= SPC_MIN_SAMPLE_usr);
      bool stableOk = (_stableMedianCount[1] >= SPC_MIN_STABLE);
      bool varOk    = (_hist.emv(e_MEDIUM) < spcMaxVar + EPS);

      _conv = (sampleOk && stableOk && varOk);
      _sessionFailed = (sampleOk && !_conv);
      float progress_f = 100.f*_hist.sampleNum()/(SPC_MIN_SAMPLE + SPC_SAMPLE_EXTRA);
      _progress = _conv ? 100 : (int)(std::min(99, me_roundf(progress_f)));

      OC_C2W_PRINT(e_SPC, e_BO_CYAN, "[Sig::spcUpdateConvergence] id=%s: sample=%d (%sok) "
                   "stable=%d (%sok), std=%.4f%s (%sok)\n%sconv, session %sfailed, prog=%d (%.2f)",
                   s_C2WSig[_id].c_str(), _hist.sampleNum(), (sampleOk ? "" : "not "),
                   _stableMedianCount[1], (stableOk ? "" : "not "),
                   me_sqrtf(_hist.emv(e_MEDIUM))*(_id==e_CAM_HEIGHT ? 100.f : RAD2DEG),
                   (_id==e_CAM_HEIGHT ? "cm" : "deg"), (varOk ? "" : "not "), 
                   (_conv ? "" : "not "), (_sessionFailed ? "" : "not "), _progress, progress_f);
    }

    bool SteadyStateCalibrator::spcEnded() {
      bool spcEnded = (_data.spc.conv ||
                       _data.spc.stopped ||
                       _data.spc.error == SPCError_TIMEOUT);
      return spcEnded;
    }

    void SteadyStateCalibrator::spcUpdateValidFrame() {
      int &m = _data.algo.pauseReason;
      int &mspc= _data.spc.invalidFrameMask;

      const int INTERNAL = (e_SENSORS_UNAVAILABLE | // TODO: move to c2sConsts.h?
                            e_EM_INVALID |
                            e_RM_INVALID |
                            e_EM_STRAIGHT |
                            e_EM_RM_PITCHDIFF |
                            e_ROAD_CROWN
                           );

      mspc = NO_PAUSE;
      if ((m & e_SPEED_TOO_LOW) != 0) {
        mspc |= MIN_SPEED;
      }
      if ((m & e_SPEED_TOO_HIGH) != 0) {
        mspc |= MAX_SPEED;
      }
      if ((m & e_ACCELERATION_TOO_HIGH) != 0) {
        mspc |= MAX_ACCELERATION;
      }
      if ((m & e_YAWRATE_TOO_HIGH) != 0) {
        mspc |= MAX_YAWRATE;
      }
      if ((m & e_RADIUS_TOO_SMALL) != 0) {
        mspc |= MIN_RADIUS;
      }
      if ((m & INTERNAL) != 0) {
        mspc |= SPCInvalidReason_INTERNAL;
      }
    }

    void SteadyStateCalibrator::spcUpdateStatus() {
      SpcData &sd = _data.spc;
      if (!sd.spcMode) {
        return;
      }

      sd.validFrame  = false;
      sd.invalidSignalMask = 0;
      sd.frame_conv          = true;
      sd.sessionFailed = false;
      int progress     = 0;
      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        Cam2WorldSignal &s = _signal[i];
        sd.validFrame       = sd.validFrame || s.validFrame();
        sd.invalidSignalMask |= s.validFrame() ? 0 : (1<<i);
        sd.frame_conv          = sd.conv && s.conv();
        sd.sessionFailed = sd.sessionFailed || s.sessionFailed();
        progress        += s.progress()/4;
      }
      sd.frame_progress = sd.frame_conv ? 100 : std::min(99, progress);

      if (sd.conv) {
        sd.status = CONVERGED;
        sd.error = isCalibInRange() ? SPCError_NONE : SPCError_OUT_OF_RANGE;
        goto debugPrint;
        return;
      }

      {
      bool runForever = (sd.maxAttempts == 0);
      bool exceededAttempts = (sd.sessionNum >= sd.maxAttempts);
      if (sd.sessionFailed && !runForever && exceededAttempts) {
        sd.status = CalibrationStatus_ERROR;
        sd.error = SPCError_TIMEOUT;
        goto debugPrint;
        return;
      }
      }

      // if (!sd.validFrame) {
      //   sd.status = e_SPC_PAUSED;
      //   return;
      // }

      sd.status = RUNNING;
      sd.error = SPCError_NONE;
      goto debugPrint;

      debugPrint: 
        OC_C2W_PRINT(e_SPC, (sd.validFrame ? e_GREEN : e_RED),
                     "[SSC::spcUpdateStatus] %svalid frame, %s, frame_progress=%d, status: %s, error: %s\n"
                     "invalidSignal=%d (%s%s%s%s), invalidFrame=%d (%s%s%s%s%s%s)",
                     (sd.validFrame ? "" : "in"),
                     (sd.conv ? "converged" : (sd.sessionFailed ? "session-failed" : (sd.stopped ? "stopped" : "running"))),
                     sd.frame_progress,
                      s_CStatus[sd.status].c_str(), s_SPCError[sd.error].c_str(),
                      sd.invalidSignalMask,
                     ((sd.invalidSignalMask & YAW)   == 0 ? "" : "yaw"),
                     ((sd.invalidSignalMask & PITCH) == 0 ? "" : " pitch"),
                     ((sd.invalidSignalMask & ROLL)  == 0 ? "" : " roll"),
                     ((sd.invalidSignalMask & CAM_H) == 0 ? "" : " camh"),
                     sd.invalidFrameMask,
                     ((sd.invalidFrameMask & MIN_SPEED)                 == 0 ? "" : "min-speed"),
                     ((sd.invalidFrameMask & MAX_SPEED)                 == 0 ? "" : " max-speed"),
                     ((sd.invalidFrameMask & MAX_ACCELERATION)          == 0 ? "" : " accel"),
                     ((sd.invalidFrameMask & MAX_YAWRATE)               == 0 ? "" : " yawrate"),
                     ((sd.invalidFrameMask & MIN_RADIUS)                == 0 ? "" : " radius"),
                     ((sd.invalidFrameMask & SPCInvalidReason_INTERNAL) == 0 ? "" : " internal"));
    }

    void SteadyStateCalibrator::spcUpdateReset() {
      SpcData &sd = _data.spc;
      bool allowedMoreAttempts = (sd.maxAttempts == 0); // runForever mode
      allowedMoreAttempts = allowedMoreAttempts || (sd.sessionNum < sd.maxAttempts);

      OC_C2W_PRINT(e_SPC, e_BO_CYAN, "[SSC::spcUpdateReset] sFailed=%d, sNum=%d, maxAttempts=%d, allowed=%d",
                   sd.sessionFailed, sd.sessionNum, sd.maxAttempts, allowedMoreAttempts);

      if (sd.sessionFailed && allowedMoreAttempts) {
        spcRestartSession();
      }
    }

    void SteadyStateCalibrator::spcRestartSession() {
      for (int i=0; i<e_C2W_SIG_NUM; ++i) {
        if (_signal[i].sessionFailed()) {
          _signal[i].reset();
          OC_C2W_PRINT(e_SPC, e_BO_CYAN, "[SSC::spcRestartSession] %s restarted", s_C2WSig[i].c_str());
        }
      }
      _data.spc.sessionNum++;
    }

    void SteadyStateCalibrator::spcUpdateFoe() {
      float yaw = _signal[e_YAW].result();
      float pitch = _signal[e_PITCH].result();
      CalibUtils::PixelLm2_f foef(0.f, 0.f);
      anglesToPixel(yaw, pitch, foef); // foe relative to image origin

      // horizonFull must be even
      CalibUtils::PixelLm2 foe(0, 0);
      foe[0] = (int)me_roundf(foef[0]);
      foe[1] = (int)(me_roundf(foef[1]*0.5f)*2.f);
      _data.spc.foe = foe + _data.cam.origin; // foe relative to nominal image center

      OC_C2W_PRINT(e_SPC, e_BO_CYAN, "[SSC::spcUpdateFoe] yaw=%.2fdeg -> r(%.2f)+%d = %d, "
                   "hor=%.2fdeg -> r(%.2f)+%d = %d",
                   yaw, foef[0], _data.cam.origin[0], _data.spc.foe[0],
                   pitch, foef[1], _data.cam.origin[1], _data.spc.foe[1]);
    }

  } // Cam2World
} // OnlineCalibration
