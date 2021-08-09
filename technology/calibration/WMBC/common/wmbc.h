/*
 * WMBC.h
 *
 *  Created on: Jun 08, 2016
 *      Author: urilo
 */

#ifndef _WMBC_H_
#define _WMBC_H_

#include "technology/calibration/WMBC/interface/wmbcIF.h"
#include "functionality/interface/autoFixIF.h"
#include "technology/calibration/WMBC/common/wmbcTypes.h"
#include "technology/calibration/WMBC/common/wmbcAutofixTypes.h"
#include "technology/mobilib/fix/common/MEXimage/sync.h"
#include "technology/mobilib/fix/common/MEXimage/typeImages.h"
#include "functionality/partialRun/wrappers/WmbcIF_wrapper.h"
#include "functionality/partialRun/wrappers/MultiCameraAutoFixIF_wrapper.h"
#include "basicTypes/containers/fastCyclicVector.h"

class WmbcProperties;

namespace WMBC {

  class CalibVariable;

  class HistogramOneDim {
  public:
    HistogramOneDim(float lowerBound, float binSize, float decFac, int startDecay);
    HistogramOneDim(float lowerBound, float binSize);
    HistogramOneDim();
    ~HistogramOneDim() {}

    void update(float x);
    void reset();

    float mean() const { return _mean;}
    float var() const { return _var;}
    float autocov() const { return _autocov;}
    float autocorr() const { return (_var>EPS ? _autocov/_var : 0.f);}
    float median() const { return _median;}
    float prevMedian() const { return _prevMedian;}
    float min() const { return _minVal;}
    float max() const { return _maxVal;}
    int minBin() const { return _minBin;}
    int maxBin() const { return _maxBin;}
    float lastVal() const { return _lastVal;}
    float lastValHist() const; // { return _lowerBound+_lastBin*_binSize;}
    int lastBin() const { return _lastBin;}
    int sampleNum() const { return _totalCount;}
    float binSize() const { return _binSize;}
    float meanDecay() const { return _meanDecay;}
    float varDecay() const { return _varDecay;}
    float decayFac() const { return _decFac;}
    int startDecay() const { return _startDecay;}
    float iqr() const { return _interquartileRange;}
    bool valid() const { return _valid;}

    void binSize(float binSize) { _binSize = binSize;}
    void debugPrintMask(int debugPrintMask) { _debugPrintMask = debugPrintMask;}
    void parent(CalibVariable *parent) { _parent = parent;}

    void printHist(std::string msg);
    void printMeanBuff();

  private:
    void updateHistogram();
    void calcMoments();
    void calcAutocov();
    void calcMedian();
    void validate();

    float _lowerBound;
    float _binSize;
    float _decFac;
    int _startDecay;

    int _hist[HIST_BIN_NUM];
    int _totalCount;

    float _firstValues[HIST_START_FRAME];
    float _lastVal;
    float _sum;
    float _sqrSum;
    float _mean;
    float _var;
    float _autocov;
    float _meanAutocovSize; // mean at autocov-win-size minus one
    float _median;
    float _prevMedian;
    float _interquartileRange;
    float _minVal;
    float _maxVal;
    int _minBin;
    int _maxBin;
    int _lastBin;
    float _meanDecay;
    float _varDecay;
    bool _valid;
    unsigned int _debugPrintMask;
    MEtypes::FastCyclicVector<float> _meanBuff; // buffer to holds older means
    CalibVariable *_parent;
  };

  class CalibVariable {
    public:
      CalibVariable();
      ~CalibVariable() {}

      void reset();
      void run(const WmbcData *dataGlobal);

      CALIB_DOF id() const { return _id;}
      bool validFrame() const { return _validFrame;}
      int validFrameNum() const {return _validFrameNum;}
      float validDistance() const { return _validDistance;}
      float validTime() const { return _validTime;}
      bool conv() const { return _conv;}
      int convNum() const { return _convNum;}
      int convLastFrame() const {return _convLastFrame;}
      int nonConvReason() const {return _nonConvReason;}
      int progress() const { return _progress;}
      int quality() const { return _quality;}
      CalibStatus status() const { return _status;}
      CalibStatus coreError() const { return _coreError;}
      CalibCoreStatus coreStatus() const { return _coreStatus;}
      float confidence(int idx) const { return _confidence[idx];}
      bool inRange() const { return _inRange;}
      bool histValid() const { return _hist.valid();}
      const HistogramOneDim& hist() const { return _hist;}
      float resultInterim() const { return _hist.median();}
      float sf() const { return _sf;}
      int invalidFrameMask() const { return _invalidFrameMask;}
      int convMovingSampleNumThreshold() const { return _convMovingSampleNumThreshold;}
      int stableMedianCount() const { return _stableMedianCount;}
      int emLastValidFrameNum() const { return _emLastValidFrameNum;}
      int emLastEpiQualityFrameNum() const { return _emLastEpiQualityFrameNum;}
      float crownQuality() const { return _crownQuality;}

      void id(CALIB_DOF id) { _id = id;}
      void invalidFrameMask(int mask) { _invalidFrameMask = mask;}
      void binSize(float binSize) { _hist.binSize(binSize);}
      void conv(bool conv, bool skip);
      void inRange(bool inRange) { _inRange=inRange;}
      void status(CalibStatus status) { _status=status;}
      void coreError(CalibStatus error) { _coreError=error;}
      void coreStatus(CalibCoreStatus status) { _coreStatus=status;}
      //void result(float result) { _result = result;}
      void debugPrintMask(int debugPrintMask) { _debugPrintMask = debugPrintMask; _hist.debugPrintMask(debugPrintMask);}

    private:
      void validateFrame();
      void setSingleFrame();
      void updateConvergence();
      void setSampleThreshold();
      virtual void setSampleThresholdMarks(float& x, float xm[], int& y, int ym[]);
      void updateConfidence();
      float calcConfidenceFunction(int idx);
      void updateStatus();
      void updateCoreStatus();

      CALIB_DOF _id;
      const WmbcData *_dataGlobal;
      HistogramOneDim _hist;
      ConvergenceDataStatistics _convStat;
      float _sf; // TODO: change to input
      float _result;

      int _pauseReason;
      int _invalidFrameMask;
      bool _validFrame;
      int _validFrameNum;
      float _validDistance;
      float _validTime;

      bool _conv;
      int _progress;
      int _quality;
      float _confidence[CONF_BIT_NUM];
      int _stableMedianCount;
      int _nonConvReason;
      int _convNum;
      int _convLastFrame;
      int _convMovingSampleNumThreshold;
      bool _inRange;

      int _emLastValidFrameNum;
      int _emLastEpiQualityFrameNum;
      float _crownQuality; // TODO: member only for itrk, could be remove when done debugging

      CalibStatus _status;
      CalibStatus _coreError;
      CalibCoreStatus _coreStatus;

      unsigned int _debugPrintMask;
  };

  struct Plane {
    Plane() : d(0.f), location(0.f), startFrame(0), endFrame(0) {
      for (int i=0; i<3; ++i) {
        N[i]=0.f;
      }
    }
    bool valid() const { return (N[1] > EPS);}
    float pitch() const { return valid() ? me_atanf(-N[2]/N[1]) : 0.f;}
    float roll() const { return valid() ? me_atanf(-N[0]/N[1]) : 0.f;}

    Float::MEmath::Vec<3,float> N; // WorldModel::EVec3 N;
    float d;
    float location;
    int startFrame;
    int endFrame;
    PlaneConfData confData;
  };
  
  class PlaneSync {
    
  public:
    PlaneSync();
    ~PlaneSync() {}
    
    void reset();
    bool run(const WmbcData *data);
    const Plane* underlyingPlane() const { return &_underlyingPlane;}
    
  private:
    void updatePlanes();
    void addPlaneToBuffer();
    bool calcUnderlyingPlane();
    
    MEtypes::FastCyclicVector<Plane> _planes;
    Plane _underlyingPlane;
    const WmbcData *_data;
    
  };
  
  class FOEFinder {
  public:
    FOEFinder(const WmbcProperties *properties);
    virtual ~FOEFinder();

    void initImages(const Prep::SafeImg* imgDist, const Prep::SafeImg* imgTargets, int level);
    virtual void runTargets() {}
    virtual void run();
    virtual void setInputDataEndFrame();
    void updateConvergence_NoSeparation();
    void updateConvergence();
    virtual void fillModelIF();

    Fix::MEimage::Sync<WmbcIF>* getWmbcIF() { return &_wmbcIF;}
    PartialRun::WmbcIF_wrapper* getWmbcIF_wrapper() {return &_wmbcIF_wrapper;}
    EWmbcIF* getEWmbcIF() { return &_eWmbcIF;}
    virtual Fix::MEimage::Sync<MultiCameraAutoFixIF>* getAutoFixIF() { return nullptr;}
    virtual PartialRun::MultiCameraAutoFixIF_wrapper* getAutoFixIF_wrapper() { return nullptr;}
    virtual EMultiCameraAutoFixIF* getEAutoFixIF() { return nullptr;} 
    virtual void fillAutofixFailSafe(bool &autoFixFailSafe, bool &autoFixFailSafeYaw, bool &autoFixFailSafeHorizon);

    // int getYawDelta() { return me_roundf(0.25*_foe.yawDeltaLm2FromEtcPrimary_f);}
    // int getHorizonDelta() { return me_roundf(0.25*_foe.horizonDeltaLm2FromEtcPrimary_f);}
    int getYawDelta() { return me_roundf(0.25*_data.results.curr.foeLm2.X());}
    int getHorizonDelta() { return me_roundf(0.25*_data.results.curr.foeLm2.Y());}
    bool getYawConvergence() { return _data.algo.single[e_YAW].conv;}
    bool getHorizonConvergence() { return _data.algo.single[e_HORIZON].conv;}

    int getConfidenceGrade() { return _data.algo.total.totalConfGrade;}
    float getConfidence() { return _data.algo.total.totalConfidence;}

    PixelLm2_f foeAllocLm2(CameraInfo::CameraInstance inst, bool interim=false);

  protected:
    //void setOrigins();
    virtual void reset();
    void reset(int idx);
    virtual void setParams(const WmbcProperties *properties);
    void setInputData();
    void calcUnderlyingPlaneData();
    void setCameraDataInit();
    void setCameraData();
    void setWmEgomotionData();
    void setWmRoadModelData();
    void setWmEgomotionTracking();
    void setWmRoadModelCov(const WorldModel::VRM::SegmentModel* model,
                           const WorldModel::EgoMotion::RoadModelStorage *storage);
    void updateFoeConfidence(TrackingConfidenceData &d, int level);
    void setVehicleData();
    void validateFrameVehicle();
    virtual void validateFrameAlgo();
#ifndef EYEQ_HW_IMPL
    bool validateFrameAdHoc();
#endif
    bool isStraightDrive();
    //virtual void calcFoe();
    // virtual void calcRollAndCamHeight();
    //virtual void calcRoll();
    //virtual void calcCamHeight();
    //virtual void updateConvergence();
    // bool isSingleCalibConverged(int calibInd);
    //virtual void updateSingleConvergence(int calibInd);
    //virtual void setSampleThreshold(int calibInd);
    virtual void updateStatus();
    void resetResultsToEtc();
    virtual void setResults();
    virtual void setResults_NoSeparation();
    void setInitialResults_NoSeparation();
    void setAllCamerasResults();
    void setFlags();
    virtual void fillWmbcIF();
    void updateVariablesTimeout(CalibStatus status);
    bool setIsCalibInRange(int idx, float val);
    void setIsCalibInRange();
    //void setFoe();
    virtual void updateReset();
    virtual void updateFoeResetFlag();
    virtual void setDebugShowData();
    void printDebugConv(int convMask) const;
    void printDebugSingleConv(int calibInd) const;
    void printDebugOrigins() const;
    void printDebugCsv(const WorldModel::VRM::SegmentModel& model,
                       const WorldModel::EgoMotion::RoadModelStorage &storage,
                       TrackingConfidenceData &cdata, int mode);
    bool validateSDM(bool conv, bool prev_value);

    CalibVariable _variable[e_CALIB_DOF_NUM];
    PlaneSync _planeSync;
    WmbcData _data;

    /*
    float _calib[e_CALIB_DOF_NUM];
    bool _validFrame;
    bool _validFrameIndividual[e_CALIB_DOF_NUM]; // individual frame validation for each quantity
    int _validFramesNumIndividual[e_CALIB_DOF_NUM];
    int _validAlgoFrameNum;
    int _validFramesForWMNum; // num of frames in which WM signal should be ok (used for signalling updateCalib to WM::CameraModel)
    int _pauseReason;
    int _delayValidCounter; // delay before can validate frame after failiure of several conditions
    int _progress;
    bool _rotateFoe;
    CalibStatus _status;
    bool _quickMode; // hack to identify which object it is
    bool _wmFoeReset;

    WorldModelData _worldModelData;
    VehicleData _vehicleData;
    WmbcFoe _foe;
    WmbcFoe _foePrevConv;
    WmbcFoe _foePrevWmReset;
    OriginData _origins[CameraInfo::e_NUM_OF_CAM_INSTANCES];
    bool _originsInit;
    VehicleToCam _v2c[CameraInfo::e_NUM_OF_CAM_INSTANCES];
    ValidationParams _params;
    ConvergenceData _conv;
    const Prep::SafeImg* _img; // image for distortion data
    const Prep::SafeImg* _imgTargets; // image for sp processing
    int _targetsLevel; // level of _imgTargets
    */
    DebugShowData _debugShowData;
    unsigned int _debugPrintMask;

    //HistogramOneDim _hist[e_CALIB_DOF_NUM];
    //int _stableMedianCount[e_CALIB_DOF_NUM];

    Fix::MEimage::Sync<WmbcIF> _wmbcIF;

    PartialRun::WmbcIF_wrapper _wmbcIF_wrapper;

    EWmbcIF _eWmbcIF;
  };

  class FOEFinder_Autofix : public FOEFinder {
  public:
    // FOEFinder_Autofix(WmbcProperties *properties) : FOEFinder(properties) {}
    FOEFinder_Autofix(WmbcProperties *properties);
    virtual void fillAutofixFailSafe(bool &autoFixFailSafe, bool &autoFixFailSafeYaw, bool &autoFixFailSafeHorizon);
    virtual Fix::MEimage::Sync<MultiCameraAutoFixIF>* getAutoFixIF() { return &_autoFixIF;}
    virtual PartialRun::MultiCameraAutoFixIF_wrapper* getAutoFixIF_wrapper() { return &_autoFixIF_wrapper;}
    virtual EMultiCameraAutoFixIF* getEAutoFixIF() { return &_eAutoFixIF;}

  protected:
    // virtual void run();
    void initAutofixState();
    void initAutofixState(int idx);
    virtual void updateReset();
    // virtual void updateConvergence();
    virtual void setResults();
    void setAutofixOutput(int idx);
    void verifyDeltaL0Limits(int idx);
    void chopByAngle(int idx, bool conv[2]);
    //virtual void setSampleThreshold(int calibInd);
    virtual void fillModelIF();
    void fillAutoFixIF();

    void debugPrintValue(int cidx, MEtl::string func, MEtl::string s);
    void debugPrintConv(int idx);
    void debugPrintToChop(int cidx, int vidx, float dAngle, bool toChop);
    void debugPrintChopped(int cidx, int vidx, float dAngle, float sgn);
    void debugPrintChopResult(int cidx, int vidx, float dAngle);
    void debugPrintCurrIsOk(int cidx, int vidx, float foeFixMagnitude);
    void debugPrintOutOfLimit(int cidx, PixelL0_i newFoe);

    Fix::MEimage::Sync<MultiCameraAutoFixIF> _autoFixIF;
    
    PartialRun::MultiCameraAutoFixIF_wrapper _autoFixIF_wrapper;
    EMultiCameraAutoFixIF _eAutoFixIF;

    AutofixState _autofixState;
    bool _singleConvergedFS[e_CALIB_DOF_NUM];
  };

  class FOEFinder_AutofixFailsafe : public FOEFinder_Autofix {
  public:
    FOEFinder_AutofixFailsafe(WmbcProperties *properties) : FOEFinder_Autofix(properties) { _data.algo.quickMode = true; _data.algo.wmFoeReset = e_WMRESET_NONE;}

  private:
    // virtual void run();
    //virtual void updateConvergence();
    //virtual void updateSingleConvergence(int calibInd);
    // void updateFoeResetFlag();
    virtual void setDebugShowData();
    // virtual void fillModelIF();

    // bool _wmFoeReset;
  };

  class FOEFinder_SPC : public FOEFinder {
  public:
    //FOEFinder_SPC(WmbcProperties *properties) : FOEFinder(properties), _sessionNumber(1) {}
    FOEFinder_SPC(WmbcProperties *properties) : FOEFinder(properties) {}

  protected:
    // virtual void setParams(const WmbcProperties *properties);
    virtual void run();
    virtual void updateStatus();
    virtual void updateReset();
    virtual void setDebugShowData();
    void restartSession();

    // int _sessionNumber;
  };
}

#endif // WMBC_H_
