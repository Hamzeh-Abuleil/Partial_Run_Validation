#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamEgoMotionManager.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamOutput.h"
#include "technology/worldModel/egoMotion/Sources/Data/EmSourceData.h"
#include "technology/worldModel/egoMotion/EgomotionPrivate_API.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "utilities/vehicleInformation/vehicleInformation_API.h"
#ifdef CALIB_DEBUG
    #include "technology/mobilib/fix/common/MEXstd/debug.h"
    #include "technology/calibration/onlineCalibration/CalibUtils/egomotionio.h"
    #include "technology/calibration/onlineCalibration/CalibUtils/MatArray.h"
#endif

namespace OnlineCalibration {
namespace Cam2Cam {

TargetsLists Cam2CamEgoMotionManager::targetsList;

TargetsLists& Cam2CamEgoMotionManager::getTargetLists() {
    return targetsList;
}

TargetsLists& Cam2CamEgoMotionManager::setTargetLists() {
    targetsList.clear();

    if (!CameraInfo::exists(CameraInfo::e_FORWARD)) {
        return targetsList;
    }
    for (int i=1; i < CoordSys::NUM_OF_CAMS; i++) {
        Targets targets = {CoordSys::FORWARD, (CoordSys)i};
        if (CameraInfo::exists(coordsToInstance((CoordSys)i))) {
            targetsList.push_back(targets);
        }
    }
    return targetsList;
}

Cam2CamEgoMotionManager::Cam2CamEgoMotionManager(Targets targets) :
        ExtrinsicCalibratorManager(targets), _calibrator(targets) {
    Cam2CamOutput::instance().toItrkHeaders();
}

void Cam2CamEgoMotionManager::init() {
    _calibrator.init();
}

bool Cam2CamEgoMotionManager::prepSources() {

    WorldModel::Exps exposures[2] =
        {WorldModel::Exps::T0, WorldModel::Exps::T1};
    const unsigned char nExposure = 2;
    const unsigned char nCoordSys = 2;
    assert(_targets.size() == nCoordSys);
    CalibUtils::mat44 transformations[nCoordSys][nExposure];
    WorldModel::globalTick toTicks[nCoordSys] = {0};
    WorldModel::globalTick fromTicks[nCoordSys] = {0};
    bool visionFound[nCoordSys] = {false};
    for (unsigned char iExposure = 0; iExposure < nExposure; iExposure++)
    {
        visionFound[0] = visionFound[1] = false;
        const MEtypes::ptr_vector<WorldModel::EgoMotion::EmData>* EmDatas = 
                            WorldModel::EgoMotion::getUmVisionMeasurements(exposures[iExposure]);
        for (unsigned int i = 0; i < EmDatas->size(); i++)
        {
            WorldModel::EgoMotion::EmData emState = EmDatas->at(i);
#ifdef CALIB_DEBUG
            std::cout << "Cam2CamCalib: valid = " << emState.valid() << "\n";
            std::cout << "Cam2CamCalib: confidence = " << emState.confidence() << "\n";
#endif
            for (unsigned char iCoordSys = 0; iCoordSys < nCoordSys; iCoordSys++)
            {
                if(!visionFound[iCoordSys] && emState.coordinateSystem() == coordsToMECoords(_targets[iCoordSys]))
                {
                    const float EPSILON = 1e-5;
                    if (!emState.valid() || emState.confidence() + EPSILON < 0.7)
                    {
                        return false;
                    }
                    toTicks[iCoordSys] = emState.toGlobalTick();
                    fromTicks[iCoordSys] = emState.fromGlobalTick();
                    visionFound[iCoordSys] = true;
                    Float::MEmath::Mat<4, 4, double>::toMat(emState.transformation(), transformations[iCoordSys][iExposure]);
                    break;
                }
            }
        }
        if (toTicks[0] != toTicks[1])
        {
            return false;
        }
        if (fromTicks[0] != fromTicks[1])
        {
            return false;
        }
        if (!visionFound[0] || !visionFound[1]) // one of the coordinate Systems is missing this exposure
        {
            return false;
        }
    }
    _sources.T0.source = transformations[0][0];
    _sources.T0.target = transformations[1][0];
    _sources.T1.source = transformations[0][1];
    _sources.T1.target = transformations[1][1];

#ifdef CALIB_DEBUG
    /* In debug, we check if a C2C_debugInDir has been defined as a stickyVal. If it has,
    we check whether there are emSource.dat and emTarget.dat in this dir and
    replace use EM in these files instead of the computed EM.
    Also, if C2C_debugOutDir exists, we save the EM sent to the calibrator in the files:
    emSourceUsed.dat and emTargetUsed.dat.
    */
    static const std::string debugInDir = Debug::Args::instance().getStickyValue("-sC2C_debugInDir", "").c_str();
    if (!debugInDir.empty())
    {
        static const CalibUtils::tformPairVec emPairsFromFile = CalibUtils::loadEgoMotion(debugInDir);
        static size_t cam2camDebugFrameIndex = 0;
        if (cam2camDebugFrameIndex < emPairsFromFile.size())
        {
            _sources.T0.source = emPairsFromFile[cam2camDebugFrameIndex].first;
            _sources.T0.target = emPairsFromFile[cam2camDebugFrameIndex++].second;
            _sources.T1.source = emPairsFromFile[cam2camDebugFrameIndex].first;
            _sources.T1.target = emPairsFromFile[cam2camDebugFrameIndex++].second;
        }
        else
        {
            assert(false && "File input finished");
        }
    }
    static const std::string debugOutDir = Debug::Args::instance().getStickyValue("-sC2C_debugOutDir", "").c_str();
    if (!debugOutDir.empty())
    {
        static std::string sourceFilename = debugOutDir + "/emSourceUsed.dat";
        static std::string targetFilename = debugOutDir + "/emTargetUsed.dat";
        static bool outputDeleted = false;
        if (!outputDeleted)
        {
            std::remove(sourceFilename.c_str());
            std::remove(targetFilename.c_str());
            outputDeleted = true;
        }
        CalibUtils::MatArray<4, 4, double> forEmSave;
        forEmSave.data.resize(2);
        forEmSave.data[0] = _sources.T0.source;
        forEmSave.data[1] = _sources.T1.source;
        forEmSave.append(sourceFilename);
        forEmSave.data[0] = _sources.T0.target;
        forEmSave.data[1] = _sources.T1.target;
        forEmSave.append(targetFilename);
    }
#endif
    return true;
}

RunConfig Cam2CamEgoMotionManager::shouldRun() {
    if (VehicleInfo::movablePartsUnlockedValid() && VehicleInfo::movablePartsUnlocked()) {
        return RunConfig::DONT_RUN;
    }
    return RunConfig::RUN;
}

void Cam2CamEgoMotionManager::run() {
//  for T0:
    _calibrator.run(_sources.T0);
//  for T1: // TODO: for now we only have one exposure
    //_calibrator.run(_sources.T1);
}

void Cam2CamEgoMotionManager::calcResult(void) {
    calcResultHelper(_calibrator);
}

State Cam2CamEgoMotionManager::calcState(int &degradeCause) {
  State state = _calibrator.getState();
  Cam2CamOutput::instance().toItrkOutput(state, _calibrator.getCurrentCalibration(),
                                         _calibrator.getStateInfo());
  Cam2CamOutput::instance().toItrkInternalState(_calibrator.getCurrentCalibration(),
                                                _calibrator.getInternalStateInfo());
  return state;
}

void Cam2CamEgoMotionManager::dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
    ExtrinsicCalibratorManager::dumpData(writer);
    _calibrator.dumpData(writer);
    _sources.dumpData(writer);
    int i=1;
    std::stringstream ss;
    for(Targets targets : getTargetLists()) {
        ss.str("");
        ss << i;
        writer.setData("targetList_" + ss.str(), reinterpret_cast<const int *>(&targets[0]), targets.size());
    }
#endif
}

void Cam2CamEgoMotionManager::fillModelIF(Fix::MEimage::Sync<OnlineCalibrationModelIF> &OcModelIF)
{
    OnlineCalibrationModelIF *m = &OcModelIF.editable();

    //Came2CamEM
    m->data.C2C_Num_of_Cams = getTargetLists().size();
    for (int i=0; i< m->data.C2C_Num_of_Cams; i++)
    {
        m->data.C2C_SensorID[i] = 0; //TODO
        m->data.C2C_camName[i] = coordsToInstance(getTargetLists()[i][1]);
        m->data.C2C_state[i] = _calibrationResults.state;
        m->data.C2C_stateDegradeCause[i] = _calibrationResults.stateDegradeCause;

        switch(m->data.C2C_state[i]) {
        case State::GOOD:
            m->data.FS_C2C_Calibration_Out_Of_Range[i] = 1;
            break;
        case State::UNVALIDATED:
            m->data.FS_C2C_Calibration_Out_Of_Range[i] = 25;
            break;
        case State::SUSPECTED:
            m->data.FS_C2C_Calibration_Out_Of_Range[i] = 50;
            break;
        case State::BAD:
            m->data.FS_C2C_Calibration_Out_Of_Range[i] = 99;
            break;
        default:
            m->data.FS_C2C_Calibration_Out_Of_Range[i] = 0;
            break;
        }

        m->data.C2C_Tx[i] = _calibrationResults.calibration.getT()[0];
        m->data.C2C_Ty[i] = _calibrationResults.calibration.getT()[1];
        m->data.C2C_Tz[i] = _calibrationResults.calibration.getT()[2];
        m->data.C2C_R11[i] = _calibrationResults.calibration.getR()[0];
        m->data.C2C_R12[i] = _calibrationResults.calibration.getR()[1];
        m->data.C2C_R13[i] = _calibrationResults.calibration.getR()[2];
        m->data.C2C_R21[i] = _calibrationResults.calibration.getR()[3];
        m->data.C2C_R22[i] = _calibrationResults.calibration.getR()[4];
        m->data.C2C_R23[i] = _calibrationResults.calibration.getR()[5];
        m->data.C2C_R31[i] = _calibrationResults.calibration.getR()[6];
        m->data.C2C_R32[i] = _calibrationResults.calibration.getR()[7];
        m->data.C2C_R33[i] = _calibrationResults.calibration.getR()[8];

        Cam2CamOutput::instance().toItrkModelIF(m, i);
    }

    OcModelIF.update();
}

bool Cam2CamEgoMotionManager::verifyProperties()
{
    return _calibrator.verifyProperties();
}

}
}
