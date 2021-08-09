/**
 * \file histogram1d.cpp
 * \brief HistogramOneDim
 *
 * \author Uri London
 * \date Jul 22, 2019
 */

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/histogram1d.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wOutputIF.h"

namespace OnlineCalibration {
  namespace Cam2World {

    void MovingCenters::init(float emaFacIn, int winSizeIn) {
      emaFac = emaFacIn;
      winSize = winSizeIn;
      meanBuff.alloc(winSize);
      varBuff.alloc(winSize);
      binBuff.alloc(winSize);
    }

    void MovingCenters::reset() {
      sma = 0.f;  // simple moving average (mean)
      smv = 0.f;  // simple moving variance
      ema = 0.f;  // exponential moving average
      emv = 0.f;  // exponential moving variance
      smm = 0.f;  // simple moving median
      prevSmm = 0.f;
      prevSma = 0.f;
      autocov = 0.f;
      meanAutocovSize = 0.f;
      valid = true;
      meanBuff.clear();
      varBuff.clear();
      binBuff.clear();
    }

    void MovingCenters::fill(float mean, float var, float median) {
      sma = mean;
      smv = var;
      ema = mean;
      emv = var;
      smm = median;
      prevSmm = median;
      prevSma = median;
      autocov = var;
      meanAutocovSize = var;

      meanBuff.push_back(mean);
      varBuff.push_back(var);
    }

  HistogramOneDim::HistogramOneDim() : HistogramOneDim(0, 1) {}

  HistogramOneDim::HistogramOneDim(float lowerBound, float binSize) : _lowerBound(lowerBound),
                                                                      _binSize(binSize),
                                                                      _parent(nullptr)
  {
    const float emaFactorFast = Debug::Args::instance().getStickyValue("-sOCC2W-emaFactorFast", EMA_FACTOR[e_FAST]);
    const float emaFactorMedi = Debug::Args::instance().getStickyValue("-sOCC2W-emaFactorMedium", EMA_FACTOR[e_MEDIUM]);
    const float emaFactorSlow = Debug::Args::instance().getStickyValue("-sOCC2W-emaFactorSlow", EMA_FACTOR[e_SLOW]);
    const unsigned int winSizeFast = Debug::Args::instance().getStickyValue("-sOCC2W-histWinSizeFast", (int)HIST_WIN_SIZE[e_FAST]);
    const unsigned int winSizeMedi = Debug::Args::instance().getStickyValue("-sOCC2W-histWinSizeMedium", (int)HIST_WIN_SIZE[e_MEDIUM]);
    const unsigned int winSizeSlow = Debug::Args::instance().getStickyValue("-sOCC2W-histWinSizeSlow", (int)HIST_WIN_SIZE[e_SLOW]);

    const float emaFactor[e_WIN_SIZE_NUM] = {emaFactorFast, emaFactorMedi, emaFactorSlow};
    const unsigned int winSize[e_WIN_SIZE_NUM] = {winSizeFast, winSizeMedi, winSizeSlow};

    for (unsigned int i=0; i<e_WIN_SIZE_NUM; ++i) {
      _moving[i].init(emaFactor[i], winSize[i]);
    }
    _sfBuff.alloc(SF_BUFF_SIZE);  // set the buffer for sf values
    _lagSamps = Debug::Args::instance().getStickyValue("-sOCC2W-corrTimeLagsSampsNum", 10);
    reset();
  }

  void HistogramOneDim::reset() {
    for (int i = 0; i < HIST_BIN_NUM; ++i) {
      _hist[i] = 0;
      for(int j=0; j < e_WIN_SIZE_NUM; ++j) {
        _moving[j].hist[i] = 0;
      }
    }
    for(int i=0; i < HIST_START_FRAME; ++i) {
      _firstValues[i] = 0;
    }
    for(int i=0; i < e_WIN_SIZE_NUM; ++i) {
      _moving[i].reset();
    } 
    _sfBuff.clear();
    _totalCount = 0;
    _currVal = 0.f;
    _sum = 0.f;
    _sqrSum = 0.f;
    _mean = 0.f;
    _var = 0.f;
    _median = 0.f;
    _prevMedian = 0.f;
    _prevMean = 0.f;
    _interquartileRange = 0.f;
    _minVal = 0.f;
    _maxVal = 0.f;
    _minBin = 0;
    _maxBin = 0;
    _currBin = 0;
    _valid = true;
    _corrTime = (float)SF_BUFF_SIZE;  //TODO: what should be the default val for this? long or short?
  }

float HistogramOneDim::currValHist() const {
  if (_totalCount <= HIST_START_FRAME) {
    return _currVal;
  }
  return _lowerBound+_currBin*_binSize;
}

void HistogramOneDim::update(float x) {
    // in the first few frames only mean is calculated and is reported also as median.
    // after a few frames the histogram boundaries are set with the mean at the center
    _currVal = x;

    if (_totalCount == 0) {
      _minVal = _maxVal = _currVal;
      _mean = _currVal;
      _median = _currVal;
      _var = 0.f;
      _sum = _currVal;
      _sqrSum = _currVal*_currVal;
      _firstValues[0] = _currVal;
      for (unsigned int i=0; i<e_WIN_SIZE_NUM; ++i) { // todo: functionize
        _moving[i].fill(_mean, _var, _median);
      }
      _totalCount++;
      return;
    }

    if (_totalCount == HIST_START_FRAME) { // set histogram limits
      _lowerBound = _mean - (HIST_BIN_NUM/2)*_binSize; //according to mean of first HIST_START_FRAME values (not including _currVal - using _mean from last iter)
      for (int i=0; i<HIST_START_FRAME; ++i) { //updating the histogram with all the first HIST_START_FRAME values
        int bin = (int)std::floor(std::min(std::max((_firstValues[i] - _lowerBound)/_binSize, 0), HIST_BIN_NUM-1));
        _hist[bin]++;
        for (unsigned int j=0; j<e_WIN_SIZE_NUM; ++j) {
          _moving[j].hist[bin]++;
          _moving[j].binBuff.push_back(bin);
        }
      }
    }

    calcMoments();
    calcAutocovPearson(); // meanBuff updated here
    calcAutoCorrTime();

    if (_totalCount < HIST_START_FRAME) { // set median to mean and abort
      _maxVal = (_currVal > _maxVal) ? _currVal : _maxVal;
      _minVal = (_currVal < _minVal) ? _currVal : _minVal;
      _prevMedian = _median;
      _prevMean = _mean;
      _median = _mean;
      for (unsigned int i=0; i<e_WIN_SIZE_NUM; ++i) {
        _moving[i].updatePrevAvrgs();
        _moving[i].smm = _mean;
      }
      _firstValues[_totalCount] = _currVal;
      _totalCount++;
      return;
    }

    updateHistogram();
    calcMedian();
    _totalCount++;
    validate();
  }

  void HistogramOneDim::updateHistogram() {
    _currBin = (int)std::floor(std::min(std::max((_currVal - _lowerBound)/_binSize, 0), HIST_BIN_NUM-1));
    _hist[_currBin]++;

    for (unsigned int i=0; i<e_WIN_SIZE_NUM; ++i) {
      _moving[i].hist[_currBin]++;
      unsigned int ws = _moving[i].winSize;
      if (_totalCount > ws) {
        ASSERT(_moving[i].binBuff.size()>=ws);
        ASSERT(_moving[i].hist[_moving[i].binBuff[1-ws]]>0);
        unsigned int lastBin = _moving[i].binBuff[1-ws];
        _moving[i].hist[lastBin]--;
      }
      _moving[i].binBuff.push_back(_currBin);
    }

    float val = currValHist();
    if (val > _maxVal) {
      _maxVal = val;
      _maxBin = _currBin;
    }

    if (val < _minVal) {
      _minVal = val;
      _minBin = _currBin;
    }
  }

  void HistogramOneDim::calcMoments() {
    // TODO: calc _mean and _var in recursive form instead of using _sum and _sqrSum which can explode
    _sum += _currVal;
    _sqrSum += _currVal*_currVal;

    float invTotalCount = 1.f / (_totalCount + 1);
    _mean = _sum * invTotalCount;
    _var = _sqrSum * invTotalCount - _mean * _mean;

    for (unsigned int i=0; i<e_WIN_SIZE_NUM; ++i) {
      MovingCenters &m = _moving[i];
      // exponential
      float prevEma = m.ema;
      m.ema = (1 - m.emaFac)*_currVal + m.emaFac*prevEma; // m.ema = m.emaFac*_currVal + (1 - m.emaFac)*prevEma;
      m.emv = (1 - m.emaFac)*_currVal*_currVal + m.emaFac*(m.smv + prevEma*prevEma) - m.ema*m.ema;

      // simple
      if (_totalCount >= m.winSize) {
        ASSERT(_moving[i].meanBuff.size()>=m.winSize);
        float r = 1.f*(_totalCount+1)/m.winSize;
        float meanW = m.meanBuff[1-m.winSize]; // mean up to _totalCount-winSize
        float varW = m.varBuff[1-m.winSize];  // var up to _totalCount-winSize

        m.sma = r*_mean - (r-1)*meanW;
        m.smv = r*(_var + _mean*_mean) - (r-1)*(varW + meanW*meanW) - m.sma*m.sma;

      } else {
        m.sma = _mean;
        m.smv = _var;
      }
      // meanBuff updated in calcAutocovPearson()
      m.varBuff.push_back(_var);
    }
  }

  void HistogramOneDim::calcAutocovPearson() {
    // /homes/urilo/Documents/math/recursiveMoments/recursiveMoments.pdf
    int n = _totalCount;
    for (unsigned int i=0; i<e_WIN_SIZE_NUM; ++i) {
      MovingCenters &m = _moving[i];
      int a = m.winSize - 1; // autocov win size

      if (n == a-1) {
        m.meanAutocovSize = _mean; // mean at autocov-win-size minus one
      }

      if (n < a) {
        m.autocov = 0.f;
        m.meanBuff.push_back(_mean);
        continue;
      }

      float Q0 = n*(n+1-a)*m.meanBuff[1-a]; // <x>_{n-a} one before the end of buffer
      float Q1 = 0.f;
      if (n > a) { // buffer is full
        Q1 = (n+1)*(n-a)*m.meanBuff[-a]; // <x>_{n-1-a} at the end of buffer
      }
      float Q2 = a*(_mean + m.meanBuff[0] - m.meanAutocovSize);

      m.autocov = n*m.autocov/(n+1);
      m.autocov += 1.f*(_mean - m.meanBuff[0])*(Q0 - Q1 - Q2)/(n+1);
      m.meanBuff.push_back(_mean);
    }
  }

  void HistogramOneDim::calcAutoCorrTime() {
    _sfBuff.push_back(_currVal);  // TODO: this should consider the sf value independant of validity. Currently hard to get in this scope.
    int len = _sfBuff.size();
    if (len<MIN_FRAMES_4_CORR){ return; }

    float corrTimeTmp = 0;
    int normFac = _lagSamps-1;
    float fullCorr;
    int lagsIncrement = (int)(me_roundf(LAG_PORTION*len/_lagSamps));
    for (int lag=0 ; lag<(int)me_roundf(LAG_PORTION*len) ; lag+=lagsIncrement) {  // iterate upon lags to sample the correlation function
      float acorr_lag = 0.f;
      for (int j=0 ; j<=(len-lag-1) ; j++){ // iterate upon all multiplication products in this lag
        acorr_lag += _sfBuff[-j-lag]*_sfBuff[-j];
      }
      if (lag==0){
        fullCorr = acorr_lag;
      }else{
        float logArg = acorr_lag/fullCorr;  // if the log argument is too small break or continue
        if (logArg<=EPS) {
          normFac-=1;
          if (normFac>1){ continue; }else{ break; }
          
        }
        corrTimeTmp += me_abs(lag)/me_logf(logArg);
      }
    }
    _corrTime = std::min(std::max(-corrTimeTmp/(float)(normFac), (float)_moving[e_FAST].winSize), 10.f*(float)len);  //limit the size of the decay between [fast window size, 10*buffer length]
  }

  void HistogramOneDim::calcMedian() {
    int valuesNum[e_WIN_SIZE_NUM+1];
    int tmpSum[e_WIN_SIZE_NUM+1];
    int halfCount[e_WIN_SIZE_NUM+1];
    float iHalf[e_WIN_SIZE_NUM+1];
    bool bHalf[e_WIN_SIZE_NUM+1];
    for (unsigned int i=0; i<e_WIN_SIZE_NUM+1; ++i) {
      valuesNum[i] = i==0 ? _totalCount+1 : std::min(_moving[i-1].winSize, _totalCount+1);
      tmpSum[i] = 0;
      halfCount[i] = valuesNum[i]/2;
      iHalf[i] = 0.f;
      bHalf[i] = false;
    }

    int quartCount = valuesNum[0]/4;
    int threeQuartCount = 3*valuesNum[0]/4; //3*quartCount;
    int iQuart = 0, iThirdQuart = 0;

    for (int i = 0; i < HIST_BIN_NUM; ++i) {
      for (unsigned int j=0; j<e_WIN_SIZE_NUM+1; ++j) {
        tmpSum[j] += (j==0) ? _hist[i] : _moving[j-1].hist[i];

        // median and moving medians
        if (tmpSum[j] > halfCount[j] and !bHalf[j]) {
          if (valuesNum[j]%2 != 0 || iHalf[j] < EPS) { 
            iHalf[j] = i;
          } else {
            iHalf[j] = 0.5 * (i + iHalf[j]);
          }
          bHalf[j] = true;
        }
        if (tmpSum[j] == halfCount[j] && iHalf[j] < EPS) {
          iHalf[j] = i;
        }
      }

      // quartiles
      if (tmpSum[0] > quartCount && iQuart == 0) {
        iQuart = i;
      }
      if (tmpSum[0] > threeQuartCount && iThirdQuart == 0) {
        iThirdQuart = i;
      }
    }

    _prevMedian = _median;
    _prevMean = _mean;
    _median = _lowerBound + iHalf[0] *_binSize;
    _interquartileRange = (iThirdQuart - iQuart) * _binSize;

    for (unsigned int i=0; i<e_WIN_SIZE_NUM; ++i) {
      _moving[i].updatePrevAvrgs();
      _moving[i].smm = _lowerBound + iHalf[i+1]*_binSize;
    }
  }

  void HistogramOneDim::validate() {
    // avoid int overflow
    // TODO: change condition to maximal bin (not max value) > HIST_MAX_SIZE ?
    // todo: chekc totalCount, max/min, maxCount
    _valid = (_totalCount <= HIST_MAX_SIZE);
    if (!_valid) {
      for (unsigned int i=0; i<e_WIN_SIZE_NUM; ++i) {
        _moving[i].valid = false;
      }
      return;
    }

    // if the edge of the histogram is being filled it's a sign that the histogram boundaries are not set correctly
    // the histogram is being invalidated in this case.
    // int edgeCount = _hist[0] + _hist[HIST_BIN_NUM-1];
    int threshold = std::max(3, (int)_totalCount/100);
    // TODO: make this more of a permanent solution
    // _valid = (edgeCount < threshold);  // when this is commented out, histogram does not invalidate due to edges filling. Will depreceate restarts under change of calibration

    for (unsigned int i=0; i<e_WIN_SIZE_NUM; ++i) {
      int edgeCount = _moving[i].hist[0] + _moving[i].hist[HIST_BIN_NUM-1];
      _moving[i].valid = (edgeCount < threshold);
    }

    // OC_C2W_PRINT(e_HIST, e_YELLOW, "[validate@HistogramOneDim] edgeCount = %d + %d = %d, _totalCount = %d, threshold: %d, status: %svalid",
    //           _hist[0], _hist[HIST_BIN_NUM-1], edgeCount, _totalCount, threshold, (_valid ? "" : "in"));
    OC_C2W_PRINT(e_HIST, e_YELLOW, "[validate@HistogramOneDim] edgeCount = %d + %d, _totalCount = %d, threshold: %d, status: %svalid",
              _hist[0], _hist[HIST_BIN_NUM-1], _totalCount, threshold, (_valid ? "" : "in"));
  }

  } // Cam2World
} // OnlineCalibration
