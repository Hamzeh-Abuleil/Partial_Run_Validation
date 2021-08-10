/**
 * \file signal.cpp
 * \brief Signal implementation
 *
 * \author Uri London
 * \date Jul 22, 2019
 */


#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wSignal.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wOutputIF.h"

namespace OnlineCalibration {
  namespace Cam2World {


    Cam2WorldSignal::Cam2WorldSignal() : _id(e_YAW), _data(nullptr), _sf(0.f), _result(0.f), _pauseReason(0),
    _invalidFrameMask(0), _validFrame(false),
    _validFrameNum(0), _validDistance(0.f), _validTime(0.f), _validFrameNumBuff(VALID_WIN_SIZE),
    _inRange(false)
    {
      _confHmm[0].rate(Debug::Args::instance().getStickyValue("-sOCC2W-hmmRate", 0.005f));
      _confHmm[0].pEq(Debug::Args::instance().getStickyValue("-sOCC2W-hmmEq", 0.01f));
      _confHmm[1].rate(_confHmm[0].rate());
      _confHmm[1].pEq(_confHmm[0].pEq());
      _hist.parent(this);
      reset();
    }

    void Cam2WorldSignal::reset() {
      _hist.reset();
      _sf = 0.f;
      _result = 0.f; // TODO: remove
      _pauseReason = 0;
      _validFrame = false;
      _validFrameNum = 0;
      _validDistance = 0.f;
      _validTime = 0.f;
      _validFrameNumBuff.clear();
      _confidence[0] = 0.f;
      _confidence[1] = 0.f;
      _confErr[0] = 0.f;
      _confErr[1] = 0.f;
      _stableMedianCount[0] = 0;
      _stableMedianCount[1] = 0;
      _unstableMedianCount[0] = 0;
      _unstableMedianCount[1] = 0;
      _unsteadySigCount[0] = 0;
      _unsteadySigCount[1] = 0;
      _confTimeout = 0.f;
      _inRange = false;
      _confHmm[0].reset();
      _confHmm[1].reset();
      _convolver[0].reset();
      _convolver[1].reset();
      _convolver[2].reset();
      _progress = 0;
      _conv = false;
      _sessionFailed = false;
    }

    void Cam2WorldSignal::run(const Cam2WorldData *data) {
      _data= data;
      validateFrame();
      if (_validFrame) {
        setSingleFrame(); // TODO: should i update sf even when invalid for ease of itrk processing?
        _hist.update(_sf);
        setStabilityMetrics();
      }
      updateConfidence();
      if (_data->spc.spcMode) {
        spcUpdateConvergence();
      }
      _inRange = setCalibInRange();
      Cam2WorldOutputIF::instance().toItrkSignal(this);
    }

    bool Cam2WorldSignal::setCalibInRange() {
      bool inRange;
      if (_id == e_CAM_HEIGHT) {
        inRange = true;
      } else {
        inRange = (result() >= _data->metaParams.calibRange[_id][0] &&
                  result() <= _data->metaParams.calibRange[_id][1]);
      }
      return inRange;
    }

    void Cam2WorldSignal::setSingleFrame() {
      switch (_id) {
        case e_YAW:
          _sf = _data->em.yaw;
          break;
        case e_PITCH:
          _sf = _data->em.pitch;
          break;
        case e_ROLL:
          _sf = _data->plane.roll;
          break;
        case e_CAM_HEIGHT:
          _sf = _data->plane.camh;
          break;
        default:
          ASSERT(false && "no id");
          _sf = 0.f;
          break;
      }
    }

    void Cam2WorldSignal::validateFrame() {
      if ((_id == e_YAW || _id == e_PITCH) && _data->algo.enforcingFirstValid) {
        _validFrame = true;
      } else {
        int pauseReason = _data->algo.pauseReason;
        _validFrame = ((pauseReason & _invalidFrameMask) == 0);
      }
      if (_validFrame) {
        _validFrameNum += 1;
        _validDistance += _data->vehicle.ds;
        _validTime += _data->vehicle.dt;
      }
      _validFrameNumBuff.push_back(_validFrameNum);
      OC_C2W_PRINT(e_VALID, e_WHITE,
                   "[Sig::validateFrame] id=%s, validFrame=%d, #validFrame=%d, validDist=%.1f, validTime=%.1f",
                   s_C2WSig[_id].c_str(), _validFrame, _validFrameNum, _validDistance, _validTime);

    }

    void Cam2WorldSignal::setStabilityMetrics(){

      // ---- signal derivative:
      // float dAvrg = _hist.smm(e_MEDIUM) - _hist.prevSmm(e_MEDIUM); // diff simple moving median
      float dAvrg = _hist.sma(e_MEDIUM) - _hist.prevSma(e_MEDIUM); // diff simple moving mean

      // ---- signal difference from staedy state:
      // float dAvrg = _hist.smm(e_MEDIUM) - _hist.smm(e_FAST);
      float steadyDiff = _hist.sma(e_MEDIUM) - _hist.sma(e_FAST);

      //update tracking of "steadiness"
      for (int i=0; i<2; ++i) {
        const ConfParams &p = _data->confParams[_id][i];

        if (me_fabs(dAvrg) < p.maxMedianChange) {
          _stableMedianCount[i]++;
          _unstableMedianCount[i] = 0;
        } else {
          _stableMedianCount[i] = 0;
          _unstableMedianCount[i]++;
        }

        if (me_fabs(steadyDiff) > p.minSteadyThresh) {
          _unsteadySigCount[i]++;
        } else {
          _unsteadySigCount[i] = 0;
        }
      }

    }

    void Cam2WorldSignal::updateConfidence() {
      switch (_confType) {
        case 0:
          updateConfidenceBasic();
          break;
        case 1:
          updateConfidenceSma2Linear();
          break;
        case 2:
          updateConfidenceRmResidual();
          break;
        case 3:
          updateConfidenceMixed();
          break;
        case 4:
          updateConfidenceErrSepTimeout();
          break;
        default:
          updateConfidenceBasic();
      }
    }

    void Cam2WorldSignal::updateConfidenceBasic() {
      if (!_validFrame) {
        _confidence[0] = _confHmm[0].update();
        _confidence[1] = _confHmm[1].update();
        return;
      }

      for (int i=0; i<2; ++i) {
        const ConfParams &p = _data->confParams[_id][i];
        HistogramOneDim &h = _hist;

        float var = h.emv(e_MEDIUM);

        float llrSample = activationFunc(h.sampleNum(), p.sampleHmm);
        float llrStable = activationFunc(_stableMedianCount[i], p.stableHmm);
        float llrVar = activationFunc(var, p.varHmm);
        float w[3] = {0.5f, 0.25f, 0.25f};
        float llr = w[0]*llrSample + w[1]*llrStable + w[2]*llrVar;
        llr *= 2.f;

        _confidence[i] = _confHmm[i].update(llr);

        OC_C2W_PRINT(e_CONF, e_WHITE,
                     "[Sig::updateConfidence] id=%s%s: var=%.2f, #stable=%d, #sample=%d\n"
                     "lr=exp(lVar+lStb+lSam)=exp[(%.2f)+(%.2f)+(%.2f)]=exp[%.2f]=%.2f, conf=%.2f",
                     s_C2WSig[_id].c_str(),
                     (i==0 ? "" : " (stringent)"),
                     var, _stableMedianCount[i], h.sampleNum(),
                     llrVar, llrStable, llrSample, llr, me_exp(llr), _confidence[i]);
      }
    }

    void Cam2WorldSignal::updateConfidenceSma2Linear() {
      if (!_validFrame) {
        _confidence[0] = _confHmm[0].update();
        _confidence[1] = _confHmm[1].update();
        return;
      }

      ActivationFuncParams p;
      p.zero = false;
      p.mode = e_SCALE_LINEAR;

      HistogramOneDim &h = _hist;
      // valid frame num in recent VALID_WIN_SIZE frames.
      int validFrameNum = _validFrameNum;
      if (_validFrameNumBuff.size() >= VALID_WIN_SIZE) {
        validFrameNum -= _validFrameNumBuff[1-VALID_WIN_SIZE];
      }
      float smaDiff = me_abs(h.sma(e_MEDIUM) - h.sma(e_FAST));
      float w[2] = {0.25f, 0.75f};

      for (int i=0; i<2; ++i) {
        p.up = true;
        p.xlimits[0] = 0.f;
        p.xlimits[1] = (i==0) ? 10.f : 20.f;
        float llr_valid = activationFunc(validFrameNum, p);

        p.up = false;
        if (_id == e_CAM_HEIGHT) {
          p.xlimits[0] = 0.001f;
          p.xlimits[1] = (i==0) ? 0.01f : 0.005f;
        } else {
          p.xlimits[0] = 0.01f*DEG2RAD;
          p.xlimits[1] = ((i==0) ? 0.25f : 0.1f)*DEG2RAD;
        }
        float llr_sma = activationFunc(smaDiff, p);

        float llr = w[0]*llr_valid + w[1]*llr_sma;
        _confidence[i] = _confHmm[i].update(llr);

        OC_C2W_PRINT(e_CONF, e_WHITE,
                     "[Sig::updateConfidence] id=%s%s: #valid=%d, dSma=|%.3f-%.3f|=%.3f\n"
                     "llr=%.2f*%.2f + %.2f*%.2f=%.2f,"
                     "lr=exp[%.2f]=%.2f, conf=%.2f",
                     s_C2WSig[_id].c_str(),
                     (i==0 ? "" : " (stringent)"),
                     validFrameNum, h.sma(e_MEDIUM), h.sma(e_FAST), smaDiff,
                     w[0], llr_valid, w[1], llr_sma, llr,
                     llr, me_exp(llr), _confidence[i]);
      }

    }

    void Cam2WorldSignal::updateConfidenceRmResidual() {
      if (!_validFrame) {
        _confidence[0] = _convolver[0].update(0.f);
        _confidence[1] = _convolver[1].update(0.f);
        return;
      }

      _confidence[0] = _convolver[0].update(_data->rm.resConfMean[_id]);
      _confidence[1] = _convolver[1].update(_data->rm.resConfMeanStringent[_id]);
    }

    void Cam2WorldSignal::updateConfidenceMixed() {
      if (!_validFrame) {
        _confidence[0] = _convolver[0].update(0.f);
        _confidence[1] = _convolver[1].update(0.f);
        _degrader.update(_data->algo.pauseReason);
        return;
      }

      float conf_sf[2] = {0.f, 0.f};
      if (_id < e_ROLL) {
        float smaDiff = me_abs(_hist.sma(e_MEDIUM) - _hist.sma(e_FAST));
        ActivationFuncParams p;
        p.zero = true;
        p.up = false;
        p.mode = e_SCALE_LINEAR;
        p.xlimits[0] = 0.01f*DEG2RAD;
        p.xlimits[1] = 0.25f*DEG2RAD;
        conf_sf[0] = activationFunc(smaDiff, p);
        p.xlimits[1] = 0.1f*DEG2RAD;
        conf_sf[1] = activationFunc(smaDiff, p);
      } else {
        conf_sf[0] = _data->rm.resConfMean[_id];
        conf_sf[1] = _data->rm.resConfMeanStringent[_id];
      }

      _confidence[0] = _convolver[0].update(conf_sf[0]);
      _confidence[1] = _convolver[1].update(conf_sf[1]);
      _degrader.update(conf_sf[0], (_id < e_ROLL));
    }

    void Cam2WorldSignal::updateConfidenceErrSepTimeout(){
      // implementation of a function where timeout-originated and error-originated causes can be separated.
      //calculate confidence originating from timeout (invalid frames):
      calcTimeoutConf();
      //calculate confidence from calculation error (RM and EM):
      calcErrConf();
      // consider both confidence signals:
      _confidence[0] = _confErr[0]*_confTimeout;
      _confidence[1] = _confErr[1]*_confTimeout;
    }

    void Cam2WorldSignal::calcTimeoutConf(){
      float w = _hist.corrTime();
      if ((int)(me_abs(_convolver[2].getWidth()-w))>UPDATE_WIDTH_DIFF){ // update kernel only if change is significant, save memory
        _convolver[2].updateWidth(w);
      }
      _confTimeout = _convolver[2].update((float)_validFrame);
    }

    void Cam2WorldSignal::calcErrConf(){
    if (_validFrame){
      _confErr[0] = mapVarTo01Conf(_data->em.sigVar[_id], _data->confParams[_id][0].confScale);
      _confErr[1] = mapVarTo01Conf(_data->em.sigVar[_id], _data->confParams[_id][1].confScale);
    }
      return;
    }


  float Cam2WorldSignal::mapVarTo01Conf(float var, float d){
    float inv_d = 1/d;
    float N = (float)_hist.winSize(e_MEDIUM);
    float exp_arg = std::sqrt(var/N)*inv_d; // exponent argument = variance of sample normalized by smoothing length, then devided by the input scale
    return me_expf(-exp_arg);;
  }

  } // Cam2World
} // OnlineCalibration

