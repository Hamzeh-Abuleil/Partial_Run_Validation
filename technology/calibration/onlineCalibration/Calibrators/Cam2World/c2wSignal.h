/**
 * \file signal.h
 * \brief Signal header
 *
 * \author Uri London
 * \date Jul 22, 2019
 */


#ifndef __OC_CAM2WORLD_SIGNAL_H
#define __OC_CAM2WORLD_SIGNAL_H

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/histogram1d.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/convolver1d.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wData.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wDegradeCause.h"

namespace OnlineCalibration {
  namespace Cam2World {

  class Cam2WorldSignal {
    public:
      Cam2WorldSignal();
      ~Cam2WorldSignal() {}

      void reset();
      void run(const Cam2WorldData *data);

      C2WSig id() const { return _id;}
      bool validFrame() const { return _validFrame;}
      int validFrameNum() const {return _validFrameNum;}
      float validDistance() const { return _validDistance;}
      float validTime() const { return _validTime;}
      int confType() const { return _confType;}
      float confidence(int i) const { return _confidence[i];}
      float confErr(int i) const { return _confErr[i];}
      float confTimeout() const { return _confTimeout;}
      bool inRange() const { return _inRange;}
      bool histValid() const { return _hist.valid();}
      const HistogramOneDim& hist() const { return _hist;}
      float result() const { return _hist.sma(e_MEDIUM);} //TODO: also in long time, how come sma follows calib better than smm? is there a problem with smm?
      float sf() const { return _sf;}
      int invalidFrameMask() const { return _invalidFrameMask;}
      int stableMedianCount(int i) const { return _stableMedianCount[i];}
      int unstableMedianCount(int i) const { return _unstableMedianCount[i];}
      int unsteadySigCount(int i) const { return _unsteadySigCount[i];}

      int progress() const { return _progress;}
      bool conv() const { return _conv;}
      bool sessionFailed() const { return _sessionFailed;}
      int degradeCause() const { return _degrader.mask();}

      void id(C2WSig id) { _id = id;}
      void invalidFrameMask(int mask) { _invalidFrameMask = mask;}
      void binSize(float binSize) { _hist.binSize(binSize);}
      void confType(int confType) { _confType = confType;}

      float getConvWidth(int i) { return _convolver[i].getWidth(); }

    private:
      void validateFrame();
      void setSingleFrame();
      bool setCalibInRange();
      bool setStableSignal();
      void setStabilityMetrics();
      void updateConfidence();
      void updateConfidenceBasic();
      void updateConfidenceSma2Linear();
      void updateConfidenceRmResidual();
      void updateConfidenceMixed();
      void updateConfidenceErrSepTimeout();
      void calcTimeoutConf();
      void calcErrConf();
      float mapVarTo01Conf(float var, float d);
      void spcUpdateConvergence();

      C2WSig _id;
      const Cam2WorldData *_data;
      HistogramOneDim _hist;
      BinaryHmm _confHmm[2];
      ConvolverOneDim _convolver[3];
      Cam2WorldSignalDegradeReporter _degrader;
      float _sf; // TODO: change to input
      float _result;

      int _pauseReason;
      int _invalidFrameMask;
      bool _validFrame;
      int _validFrameNum;
      float _validDistance;
      float _validTime;
      MEtypes::FastCyclicVector<int> _validFrameNumBuff; // buffer to holds older valid frame counter

      float _confidence[2];
      float _confErr[2];
      int _unstableMedianCount[2];
      int _stableMedianCount[2];
      int _unsteadySigCount[2];
      // bool _stableSignal[2]; 
      float _confTimeout;
      bool _inRange;
      int _confType;

      int _progress; // spc mode
      bool _conv; // spc mode
      bool _sessionFailed; // spc mode
  };

  } // Cam2World
} // OnlineCalibration

#endif // __OC_CAM2WORLD_SIGNAL_H

