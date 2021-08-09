/**
 * \file histogram1d.h
 * \brief HistogramOneDim header
 *
 * \author Uri London
 * \date Jul 22, 2019
 */


#ifndef __OC_CAM2WORLD_HIST1D_H
#define __OC_CAM2WORLD_HIST1D_H

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wConsts.h"
#include "basicTypes/containers/fastCyclicVector.h"

namespace OnlineCalibration {
  namespace Cam2World {

    class Cam2WorldSignal;

    struct MovingCenters {
      MovingCenters() : sma(0.f), smv(0.f), ema(0.f), emv(0.f),
                        smm(0.f), prevSmm(0.f), prevSma(0.f), autocov(0.f),
                        meanAutocovSize(0.f), valid(false),
                        emaFac(0.f), winSize(1) {}
      void init(float emaFac, int winSize);
      void reset();
      void fill(float mean, float var, float median);
      void updatePrevAvrgs() { prevSmm = smm; prevSma = sma;}

      float sma; // simple moving average
      float smv; // simple moving variance
      float ema; // exponential moving average
      float emv; // exponential moving variance
      float smm; // simple moving median
      float prevSmm; // simple moving median
      float prevSma; // simple moving mean
      float autocov; // auto-covariance
      float meanAutocovSize; // mean at autocov-win-size minus one
      bool valid;

      float emaFac; // exponential factor
      unsigned int winSize; // window size of sma and smm

      MEtypes::FastCyclicVector<float> meanBuff; // buffer to holds older means
      MEtypes::FastCyclicVector<float> varBuff;  // buffer to holds older variances
      MEtypes::FastCyclicVector<int> binBuff;   // buffer to holds older histogram bins for median
      int hist[HIST_BIN_NUM];
    };

    class HistogramOneDim {
      public:
        HistogramOneDim();
        HistogramOneDim(float lowerBound, float binSize);
        ~HistogramOneDim() {}

        void update(float x);
        void reset();

        float mean() const { return _mean;}
        float var() const { return _var;}
        float median() const { return _median;}
        float prevMedian() const { return _prevMedian;}
        float prevMean() const { return _prevMean;}
        float min() const { return _minVal;}
        float max() const { return _maxVal;}
        int minBin() const { return _minBin;}
        int maxBin() const { return _maxBin;}
        float currVal() const { return _currVal;}
        float currValHist() const;
        int currBin() const { return _currBin;}
        int sampleNum() const { return _totalCount;}
        float binSize() const { return _binSize;}
        float iqr() const { return _interquartileRange;}
        float sma(unsigned int i) const {return _moving[i].sma;}  // simple moving average (mean)  // todo: assert on 0<i<win_num
        float smv(unsigned int i) const {return _moving[i].smv;}  // simple moving variance
        float ema(unsigned int i) const {return _moving[i].ema;}  // exponential moving average
        float emv(unsigned int i) const {return _moving[i].emv;}  // exponential moving variance
        float smm(unsigned int i) const {return _moving[i].smm;}  // simple moving median
        float prevSmm(unsigned int i) const {return _moving[i].prevSmm;}
        float prevSma(unsigned int i) const {return _moving[i].prevSma;}
        float autocov(unsigned int i) const { return _moving[i].autocov;}
        float autocorr(unsigned int i) const { return (_var>EPS ? _moving[i].autocov/_var : 0.f);}
        float emaFac(unsigned int i) const {return _moving[i].emaFac;}
        unsigned int winSize(unsigned int i) const {return _moving[i].winSize;}
        bool valid() const { return _valid;}
        float corrTime() const { return _corrTime; }
        int lagSamps() const { return _lagSamps; }

        void binSize(float binSize) { _binSize = binSize;}
        void parent(Cam2WorldSignal *parent) { _parent = parent;}

      private:
        void updateHistogram();
        void calcAutocovPearson();
        void calcAutoCorrTime();
        void calcMoments();
        void calcMedian();
        void validate();

        float _lowerBound;
        float _binSize;

        int _hist[HIST_BIN_NUM];
        unsigned int _totalCount;

        float _firstValues[HIST_START_FRAME];
        float _currVal;
        float _sum;
        float _sqrSum;
        float _mean;
        float _var;
        float _median;
        float _prevMedian;
        float _prevMean;
        float _interquartileRange;
        float _minVal;
        float _maxVal;
        int _minBin;
        int _maxBin;
        int _currBin;
        bool _valid;
        float _corrTime;
        int _lagSamps;

        MovingCenters _moving[e_WIN_SIZE_NUM];
        Cam2WorldSignal *_parent;
        MEtypes::FastCyclicVector<float> _sfBuff; // buffer to hold older sf values, used to calc autocorr. The size of buffer here would be longer than the avereging window that feeds it.
    };

  } // Cam2World
} // OnlineCalibration

#endif // __OC_CAM2WORLD_HIST1D_H
