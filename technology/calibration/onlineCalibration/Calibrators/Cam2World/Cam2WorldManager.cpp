#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/Cam2WorldManager.h"
#include "utilities/vehicleInformation/vehicleInformation_API.h"
#include "technology/mobilib/fix/common/MEXmisc/timeStampUtils.h"
#include "technology/worldModel/egoMotion/EgomotionPrivate_API.h"

#include "technology/calibration/cameraToCamera/cameraToCamera_API_internal.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wUtils.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wOutputIF.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wConsts.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"
#include "technology/calibration/onlineCalibration/onlineCalibration_ServicesUser.h"

namespace OnlineCalibration {
namespace Cam2World {

TargetsLists Cam2WorldManager::targetsList;

TargetsLists& Cam2WorldManager::getTargetLists() {
    return targetsList;
}

TargetsLists& Cam2WorldManager::setTargetLists() {
    targetsList.clear();
    targetsList.push_back({CoordSys::WORLD, CoordSys::FORWARD});
    return targetsList;
}

Cam2WorldManager::Cam2WorldManager(Targets targets) :
        ExtrinsicCalibratorManager(targets), _calibrator(targets) ,
        _prevState(State::UNVALIDATED), _innerState(State::UNVALIDATED), _prevInnerState(State::UNVALIDATED),
        _confLevel(ConfLevel::LOW), _prevConfLevel(ConfLevel::LOW), 
        _wasHighConf(false),  _confidence(0), _stringentConfidence(0), _stateTimeCounter(0), _prevStateTimeCounter(0), _confLevelTimeCounter(0) {
          _highConfParams[0] = Debug::Args::instance().getStickyValue("-sOCC2W-minHighTh", 20);
          _highConfParams[1] = Debug::Args::instance().getStickyValue("-sOCC2W-maxHighTh", 85);
          _highConfParams[2] = Debug::Args::instance().getStickyValue("-sOCC2W-incrementHighTh", 1);
          _highConfParams[3] = Debug::Args::instance().getStickyValue("-sOCC2W-stringentHighTh", 70);
          _highConfTh = _highConfParams[0];
          _thresholdsDelta = Debug::Args::instance().getStickyValue("-sOCC2W-stringentHighTh", 10);
          _stringentHighThRng[0] = _highConfParams[3];
          _stringentHighThRng[1] = std::min((_highConfParams[3]+_thresholdsDelta), (unsigned int)100);
          _highThRng[0] = _highConfParams[0];
          _highThRng[1] = std::min((_highConfParams[0]+_thresholdsDelta), _highConfParams[3]);
          _lowConfTh = Debug::Args::instance().getStickyValue("-sOCC2W-lowConfTh", 7);
          _stateMachine = Debug::Args::instance().getStickyValue("-sOCC2W-stateMachine", 3); //0: Uri's original with hysteresis. 1: a simple machine just quantizes the confidence
          _startDelayLength = Debug::Args::instance().getStickyValue("-sOCC2W-startDelayState", 10);  // number of frames to consider at the begining of a ride before falling to suspected
          _suspectTimeTh = Debug::Args::instance().getStickyValue("-sOCC2W-StateTransitionTime", 3);  // number of frames above which constant frames in medium conf will be stated as suspected
          _frameProgress = Debug::Args::instance().existsParameter("-sOCC2W-spcFrameProgress");
          Cam2WorldOutputIF::instance().toItrkHeaders();
}

void Cam2WorldManager::init() {
    prepSourcesInit();
    _calibrator.init(_sources);
}

void Cam2WorldManager::prepSourcesInit() {
  Cam2World::CameraRawData &c = _sources.cam;

  // get camera raw data
  c.focalLm2 = CameraInfo::getPreciseCamK(CameraInfo::e_FORWARD)(0, 0);

  c.foeFull[0] = CameraInfo::yawFull(CameraInfo::e_FORWARD);
  c.foeFull[1] = CameraInfo::horizonFull(CameraInfo::e_FORWARD);

  c.foeAfix[0] = CameraInfo::initialAutoFix_yaw(CameraInfo::e_FORWARD);
  c.foeAfix[1] = CameraInfo::initialAutoFix_horizon(CameraInfo::e_FORWARD);
  c.foeAfix   *= OnlineCalibration::Cam2World::L0_TO_LM2_SCALE;

  c.roll = CameraInfo::rollAngle(CameraInfo::e_FORWARD);
  c.camh = CameraInfo::cameraHeight(CameraInfo::e_FORWARD);

  c.minFoe[0] = CameraInfo::minYaw(CameraInfo::e_FORWARD);
  c.minFoe[1] = CameraInfo::minHorizon(CameraInfo::e_FORWARD);
  c.maxFoe[0] = CameraInfo::maxYaw(CameraInfo::e_FORWARD);
  c.maxFoe[1] = CameraInfo::maxHorizon(CameraInfo::e_FORWARD);
  c.minFoe   *= OnlineCalibration::Cam2World::L0_TO_LM2_SCALE;
  c.maxFoe   *= OnlineCalibration::Cam2World::L0_TO_LM2_SCALE;
  c.maxRoll   = CameraInfo::maxRoll(CameraInfo::e_FORWARD);
}

bool Cam2WorldManager::prepSources() {
  if (_targets.back() != CoordSys::FORWARD) {
    return true;
  }

  Cam2World::ParamsRaw &p = _sources.params;
  Cam2World::VehicleRawData &v = _sources.vh;
  Cam2World::EmRawData &e = _sources.em;
  Cam2World::RmRawData &r = _sources.rm;

  // get params
  p.spcMode = _calibrator.getBaseLineProperites()->getSpcMode();
  p.slowMode = _calibrator.getBaseLineProperites()->getSlowMode();

  // get vehicle raw data
  v.isYawRateAvailable = PrepSys_API::getYawRateAvailable(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
  // todo: check if there's better getter
  v.yawRateInPixels = PrepSys_API::getVehicleYawInPixels(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
  v.speedAvailable = VehicleInfo::speedAvailable();
  v.speed = VehicleInfo::egoSpeed();
  v.dt = dTime(*PrepSys_API::getTimeStamp(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD));

  // get WorldModel raw data
  auto * emApi=OnlineCalibrationServicesUser::instance().get<WorldModel::EgoMotion::EgoMotionService_API>();
  assert(emApi!=nullptr);
  e.valid = emApi->isOn();
  if (!e.valid) {
    e.emVisionMeasurementData = nullptr;
    r.valid = false;
    r.model = nullptr;
    r.storage = nullptr;
    return false;
  }

  e.emVisionMeasurementData = WorldModel::EgoMotion::getEmVisionMeasurement(WorldModel::EgoMotion::Exposure::T0, WorldModel::RealCamInstance::RCI_FORWARD);
  if ((e.valid = (e.emVisionMeasurementData and e.emVisionMeasurementData->valid()))) {}

  // I'ts important to check this after isOn, because it can return true for modelFound even if false for isOn
  auto * rmApi=OnlineCalibrationServicesUser::instance().get<WorldModel::RM::RoadModelService_API>();
  assert(rmApi!=nullptr);
  r.valid = rmApi->roadModelFound();
  if (!r.valid) {
    r.model = nullptr;
    r.storage = nullptr;
    return true;
  }

  r.model = &rmApi->getClosestPlane();
  // r.valid &= r.model->isValid;
  r.storage = &WorldModel::EgoMotion::getRoadModelStorage(MEtypes::RCI_FORWARD);

  return true;
}

RunConfig Cam2WorldManager::shouldRun() {
    if (!CameraInfo::exists(CameraInfo::e_FORWARD)) {
      return RunConfig::DONT_RUN;
    }
    return RunConfig::RUN; //TODO
}

void Cam2WorldManager::run() {
    _calibrator.run(_sources);
}

void Cam2WorldManager::calcResult(void) {
    calcResultHelper(_calibrator);
}

State Cam2WorldManager::calcState(int &degradeCause) {
    const Cam2WorldStateInfo& stateInfo = _calibrator.getStateInfo();
    // return calcStateHelper(stateInfo, _lastRunGFI);

    if (stateInfo.getSpcData().spcMode) {
      return calcStateSpc(degradeCause);
    }

    State state = State::NUM_OF_STATES;
    switch (_stateMachine) {
        case 0:
          stateMachineClassic(state);
          break;
        case 1:
          stateMachineSimpleThresholds(state);
          break;
        case 2:
          stateMachineThresholdWMemory(state);
          break;
        case 3:
          stateMachineTempName(state);
          break;
    }

    // degrageCause updates only when state changes to the worse
    degradeCause = (int)StateDegradeCause::NO_DEGRADE; 
    if (state > _prevState && state != State::BAD) {
      degradeCause = _calibrator.getDegradeCause();
    }
    _prevState = state;

    Cam2WorldOutputIF::instance().toItrkOutput(state, degradeCause, _highConfTh, _stateTimeCounter,
                                                _confLevelTimeCounter,
                                                _lowConfTh, _highConfParams, _confLevel, _innerState,
                                               _calibrator.getCurrentCalibration(),
                                               _calibrator.getStateInfo());
    OC_C2W_PRINT(e_STATE, e_GREEN, "[C2WManager::calcState] state: %s, conf=%d (%d), confTh: (%d, %d, %d)",
                 s_State[(int)state].c_str(), _confidence, _stringentConfidence, _lowConfTh,
                 _highConfParams[0], _highConfTh);

    return state;
}


State Cam2WorldManager::calcStateSpc(int &degradeCause) {
    _spcData = _calibrator.getStateInfo().getSpcData();
    State state = State::NUM_OF_STATES;
    calcSpcProgress();
    computeSpcStatusAndError();
    switch (_spcData.status) {
      case CONVERGED:
        if (_spcData.error==SPCError_NONE) {
          state = State::GOOD;
        } else {
          state = State::BAD; // out-of-range
        }
        break;
      case RUNNING:
        state = State::UNVALIDATED;
        break;
      case CalibrationStatus_ERROR:
        state = State::SUSPECTED;
        break;
      default:
        state = State::NUM_OF_STATES;
    }

    degradeCause = _calibrator.getDegradeCause();

    Cam2WorldOutputIF::instance().toItrkOutput(state, degradeCause, _highConfTh, _stateTimeCounter,
                                                _confLevelTimeCounter,
                                                _lowConfTh, _highConfParams, _confLevel, _innerState,
                                               _calibrator.getCurrentCalibration(),
                                               _calibrator.getStateInfo());
    Cam2WorldOutputIF::instance().toItrkSPC(_spcData);
    OC_C2W_PRINT(e_STATE, e_GREEN, "[C2WManager::calcStateSpc] state: %s", s_State[(int)state].c_str());

    return state;
}

void Cam2WorldManager::calcSpcProgress(){
  int prog = (int)(100.f*_calibrator.getStateInfo().getConfidence()/_highConfTh);
  _spcData.conf_progress =  std::min(100, prog);
  _spcData.progress = _frameProgress ? _spcData.frame_progress : _spcData.conf_progress;
}

void Cam2WorldManager::computeSpcStatusAndError(){
  // add the debugPrint option in this function
  if (_spcData.progress>=100) {
    _spcData.status = CONVERGED;
    _spcData.error = _calibrator.getStateInfo().getInRange() ? SPCError_NONE : SPCError_OUT_OF_RANGE;
    // TODO: add here debugPrint
    return;
    }

  bool runForever = (_spcData.maxAttempts == 0);
  bool exceededAttempts = (_spcData.sessionNum >= _spcData.maxAttempts);
  if (_spcData.sessionFailed && !runForever && exceededAttempts) {
    _spcData.status = CalibrationStatus_ERROR;
    _spcData.error = SPCError_TIMEOUT;
    // TODO: add here debugPrint
    return;
  }

  _spcData.status = RUNNING;
  _spcData.error = SPCError_NONE;
  // TODO: add here debugPrint
  return;
}

void Cam2WorldManager::updateSPCTrigger(bool spcTrigger) {
  static bool _spcRunning = true;
  // When the signal is triggered to 1, SPC restarts its calculations.
  // SPC is paused as long as the signal is 1,
  // will restart the calculation once the signal returns from 1 to 0.
  if (spcTrigger) {
    //SPC should flush its buffer// and start over
    if (_spcRunning) {
        _calibrator.stopSPC();
        _spcRunning = false;
    }
    return;
  }
  if (!spcTrigger && !_spcRunning) {
    _calibrator.restartSPC();
    _spcRunning = true;
  }

}

void Cam2WorldManager::stateMachineClassic(State &state){
  // this is Uri's original state machine
  _confidence = _calibrator.getStateInfo().getConfidence();
  _stringentConfidence = _calibrator.getStateInfo().getStringentConfidence();

  ASSERT(_stringentConfidence <= _confidence); // TODO: uncomment after conf research is completed

  bool inRange = _calibrator.getStateInfo().getInRange();
  bool highConf          = (_confidence > _highConfTh);
  bool mediumConf        = (_confidence > _lowConfTh);
  bool highStringentConf = (_stringentConfidence > _highConfParams[0]);

  if (highConf) {
    state = inRange ? State::GOOD : State::UNVALIDATED;
    _highConfTh = std::max(_highConfParams[0], _highConfTh - _highConfParams[2]);
    _wasHighConf = true;
  } else { 
    state = mediumConf ? State::UNVALIDATED : State::SUSPECTED;
    if (_wasHighConf) {
      _highConfTh = std::min(_highConfParams[1], _highConfTh + _highConfParams[2]);
    }
  }

  if (highStringentConf && !inRange) {
    state = State::BAD;
  }
}

void Cam2WorldManager::stateMachineSimpleThresholds(State &state){
  // A simple state machine with stupid thresholding of the 
  _confidence = _calibrator.getStateInfo().getConfidence();
  _stringentConfidence = _calibrator.getStateInfo().getStringentConfidence();

  ASSERT(_stringentConfidence <= _confidence); // TODO: uncomment after conf research is completed

  bool inRange = _calibrator.getStateInfo().getInRange();
  bool highConf           = (_confidence > _highConfTh);
  bool mediumConf         = (_confidence > _lowConfTh);
  bool highStringentConf  = (_stringentConfidence > _highConfParams[3]);
  bool rideBegining       = (globalFrameIndex <= (int)_startDelayLength);

  if (rideBegining & (state>State::UNVALIDATED)){
    state = State::UNVALIDATED;
    return;
  }

  if (highConf) {
    state = inRange ? State::GOOD : State::UNVALIDATED;
    _wasHighConf = true;
  } else { 
    state = mediumConf ? State::UNVALIDATED : State::SUSPECTED;
  }

  if (highStringentConf && !inRange) {
    state = State::BAD;
  }
}


void Cam2WorldManager::stateMachineThresholdWMemory(State &state){
  // A simple state machine with stupid thresholding of the 
  _confidence = _calibrator.getStateInfo().getConfidence();
  _stringentConfidence = _calibrator.getStateInfo().getStringentConfidence();

  ASSERT(_stringentConfidence <= _confidence); // TODO: uncomment after conf research is completed

  bool inRange = _calibrator.getStateInfo().getInRange();
  // set confidence range-value:
  _prevConfLevel = _confLevel;  // save the previous confidence level
  if (_stringentConfidence > _highConfParams[3]){ // stringent confidence cancels other confidence levels
    _confLevel = ConfLevel::HIGH_STRING;
  } else{
    if (_confidence > _highConfTh){
      _confLevel = ConfLevel::HIGH;
    }else if (_confidence > _lowConfTh) {
      _confLevel = ConfLevel::MED;
    } else {
      _confLevel = ConfLevel::LOW;
    }
  }

  // update how much time was spent in this confidence level:
  _confLevelTimeCounter = (_confLevel==_prevConfLevel) ? _confLevelTimeCounter+1 : 0 ;
  _confLevelTimeCounter = std::min(_confLevelTimeCounter, std::max(_suspectTimeTh, _startDelayLength)); // limit the size of _confLevelTimeCounter to prevent overloading
  switch (_confLevel) {
      case HIGH_STRING:
        state = !inRange ? State::BAD : State::GOOD;
        break;
      case HIGH:
        state = inRange ? State::GOOD : State::UNVALIDATED;
        break;
      case MED:
        state = (_confLevelTimeCounter<_suspectTimeTh) ? _prevState : State::SUSPECTED;
        break;
      case LOW:
        state = (_confLevelTimeCounter<_startDelayLength) ? _prevState : State::UNVALIDATED;
        break;
      case NUM_OF_CONF:
        break;
      default:
        break;
      }
  // exit state=BAD only if entered good:
  state = ((_prevState==State::BAD) && (state!=State::GOOD )) ? State::BAD : state;
  // how much time was spent in this state:
  _stateTimeCounter = (_prevState==state) ? _stateTimeCounter+1 : 0;
  _stateTimeCounter = std::min(_stateTimeCounter, std::max(_suspectTimeTh, _startDelayLength)); // limit the size of _stateTimeCounter to prevent overloading
}

void Cam2WorldManager::stateMachineTempName(State &state){
  // A state machine with simple thresholding, only differs between UNVALIDATED an GOOD.
  // SUSPECTED is forced when the change in confidence is too sudden or when coming back into GOOD with a large change in calibration value.
  // BAD is forced when confidence is very high (above a higher threshold) and calibration is OOR.
  // stringent-confidence and ConfLevel::MED remain unesed in this machine.

  _prevInnerState = _innerState;  // update prev state
  _confidence = _calibrator.getStateInfo().getConfidence(); // get current confidence
  bool inRange = _calibrator.getStateInfo().getInRange();   // is current calibration inRange?
  bool stableSig = _calibrator.getStateInfo().getStableSig();   // is the signal stable?
  // set current confidence range-value:
  _prevConfLevel = _confLevel;  // save the previous confidence level
  // _stringentHighThRng
  // _highThRng

  if (_confidence > _highConfParams[3]){
    _confLevel = ConfLevel::HIGH_STRING;
    _highConfParams[3] = std::min(_highConfParams[3]+_highConfParams[2], _stringentHighThRng[1]); //hysteresis: if above, rasie bar to make it harder to exit BAD
  } else if (_confidence >= _highConfTh) {
    _confLevel = ConfLevel::HIGH;
    _highConfParams[3] = std::max(_highConfParams[3]-_highConfParams[2], _stringentHighThRng[0]); //hysteresis: if below, lower bar to make it easier to enter BAD
    _highConfTh = std::max(_highConfTh-_highConfParams[2], _highThRng[0]); //hysteresis: if above, lower bar to make it harder to exit GOOD
  } else {
    _confLevel = ConfLevel::LOW;
    _highConfTh = std::min(_highConfTh+_highConfParams[2], _highThRng[1]); //hysteresis: if below, raise bar to increase stability of state
  }

  // update how much time was spent in this confidence level:
  _confLevelTimeCounter = (_confLevel==_prevConfLevel) ? _confLevelTimeCounter+1 : 0 ;
  _confLevelTimeCounter = std::min(_confLevelTimeCounter, std::max(_suspectTimeTh, _startDelayLength)); // limit the size of _confLevelTimeCounter to prevent overloading

// set current state:
switch (_confLevel) {
      case HIGH_STRING:
      // if very high conf: that's good unless you're OOR, then it's BAD.
        _innerState = !inRange ? State::BAD : State::GOOD;
        break;
      case HIGH:
        _innerState = inRange ? State::GOOD : State::UNVALIDATED;
        break;
      case LOW:
        _innerState = (_confLevelTimeCounter<_startDelayLength) ? _prevState : State::UNVALIDATED;  // make sure you spent in this conf level long enough before degrading state
        break;
      case NUM_OF_CONF:
        break;
      default:
        break;
      }
  

    if ((_prevState == State::BAD) | (_prevState==State::SUSPECTED)){
      bool exitBad = (_innerState==State::GOOD) & (_confLevel==ConfLevel::HIGH_STRING) & (_prevState==State::BAD);
      bool exitSus = (_innerState==State::GOOD) & (_prevState==State::SUSPECTED);
      if (!(exitBad|exitSus)){
          _innerState = _prevState;
      }
    }
  
  // force SUSPECTED if signal is unstable, and not BAD
  if ((!stableSig)&&(_innerState!=State::BAD)){ _innerState=State::SUSPECTED; }

  // how much time was spent in this state:
  _prevStateTimeCounter = _stateTimeCounter;
  _stateTimeCounter = (_prevInnerState==_innerState) ? _stateTimeCounter+1 : 0;
  _stateTimeCounter = std::min(_stateTimeCounter, std::max(_suspectTimeTh, _startDelayLength)); // limit the size of _stateTimeCounter to prevent overloading


  // set the state if innerState spent enough time stable (this is a type of smoothing to make sure we're making the right descision)
  bool setNewState = (_innerState==State::BAD) || (_stateTimeCounter>_suspectTimeTh);
  state = setNewState ? _innerState : _prevState;
}


void Cam2WorldManager::dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
    ExtrinsicCalibratorManager::dumpData(writer);
    int i=1;
    std::stringstream ss;
    for(Targets targets : getTargetLists()) {
        ss.str("");
        ss << i;
        writer.setData("targetList_" + ss.str(), reinterpret_cast<const int *>(&targets[0]), targets.size());
    }
    _calibrator.dumpData(writer);
    _sources.dumpData(writer);
#endif
}

void Cam2WorldManager::fillModelIF(Fix::MEimage::Sync<OnlineCalibrationModelIF> &OcModelIF)
{
    OnlineCalibrationModelIF *m = &OcModelIF.editable();
    m->data.C2W_yaw = _calibrationResults.calibration.getComponents().foe.X();
    m->data.C2W_pitch = _calibrationResults.calibration.getComponents().foe.Y();
    m->data.C2W_roll = _calibrationResults.calibration.getComponents().rollAngle;
    m->data.C2W_cameraHeight = -_calibrationResults.calibration.getT()[1];
    m->data.C2W_state = _calibrationResults.state;
    m->data.C2W_stateDegradeCause = _calibrationResults.stateDegradeCause;
    m->data.C2W_stateConf =  _calibrator.getStateInfo().getConfidence();

    switch(m->data.C2W_state) {
    case State::GOOD:
        m->data.FS_C2W_Calibration_Out_Of_Range = 1;
        break;
    case State::UNVALIDATED:
        m->data.FS_C2W_Calibration_Out_Of_Range = 2;
        break;
    case State::SUSPECTED:
        m->data.FS_C2W_Calibration_Out_Of_Range = 4;
        break;
    case State::BAD:
        m->data.FS_C2W_Calibration_Out_Of_Range = 5;
        break;
    default:
        m->data.FS_C2W_Calibration_Out_Of_Range = 0;
        break;
    }

    matf2arr(_calibrationResults.calibration.getR(), m->data.C2W_R);
    vecf2arr(_calibrationResults.calibration.getT(), m->data.C2W_T);
    if (globalFrameIndex <=0 ) {
      matf2arr(_calibrationResults.calibration.getComponents().R_init, m->data.C2W_HighState_R);
      vecf2arr(_calibrationResults.calibration.getComponents().t_init, m->data.C2W_HighState_T);
      CameraToCameraAPI::setC2W_RT(_calibrationResults.calibration.getComponents().R_init,
                                   _calibrationResults.calibration.getComponents().t_init,
                                   CameraToCameraTypes::e_ONLINE_CALIBRATION);
    }
    if (m->data.C2W_state == State::GOOD || m->data.C2W_state == State::UNVALIDATED) {
      copyArr(9, m->data.C2W_R, m->data.C2W_HighState_R);
      copyArr(3, m->data.C2W_T, m->data.C2W_HighState_T);
      CameraToCameraAPI::setC2W_RT(m->data.C2W_R, m->data.C2W_T,
                                   CameraToCameraTypes::e_ONLINE_CALIBRATION);
    }

    // SPC
    // const SpcData& spcData = _calibrator.getStateInfo().getSpcData();
    SpcData& spcData = _spcData;
    if (spcData.spcMode) {
      m->data.SPC_Status          = spcData.status;
      m->data.SPC_Progress        = spcData.progress;
      m->data.SPC_Error           = spcData.error;
      m->data.SPC_Session_Number  = spcData.sessionNum;
      m->data.SPC_Frame_Valid     = spcData.validFrame;
      m->data.SPC_Invalid_Signal  = spcData.invalidSignalMask;
      m->data.SPC_Invalid_Reason  = spcData.invalidFrameMask;
      m->data.SPC_Baseline_Yaw    = spcData.foe[0];
      m->data.SPC_Baseline_Pitch  = spcData.foe[1];
      m->data.SPC_Baseline_Height = m->data.C2W_cameraHeight;
      m->data.SPC_Baseline_Roll   = m->data.C2W_roll;
    }

    OcModelIF.update();
    Cam2WorldOutputIF::instance().toItrkModelIF(&OcModelIF.getObj());
}

void Cam2WorldManager::updateDriverProfileInput(dstruct_t* driverProfile)
{
    if (!driverProfile->bytes) { // verifies the driverProfile presence. if not active - exit
        return;
    }
    DKEYTRYLOCAL(driverProfile, spcTrigger);
    if DKEYVALID(spcTrigger) {
      bool _spcTrigger = dsgetb(dbool_t, driverProfile, spcTrigger);
      updateSPCTrigger(_spcTrigger);
    }
}

    bool Cam2WorldManager::verifyProperties() {
    return _calibrator.verifyProperties();
}

}
}
