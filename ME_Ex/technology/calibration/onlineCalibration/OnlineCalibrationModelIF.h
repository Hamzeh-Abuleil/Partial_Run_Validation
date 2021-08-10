#ifndef __ONLINE_CALIBRATION_MODEL_IF_H_
#define __ONLINE_CALIBRATION_MODEL_IF_H_

#include "technology/calibration/onlineCalibration/OnlineCalibrationDefs.h"

namespace OnlineCalibration {

struct OnlineCalibrationModelIFData {

    OnlineCalibrationModelIFData()
    {
      std::memset(this, 0, sizeof(*this));

      C2W_state = State::SUSPECTED;
      for (unsigned int i=0; i<NUM_CAM2CAM; ++i) {
           C2C_state[i]             = State::UNVALIDATED;
           C2C_R11[i]               = 1;
           C2C_R22[i]               = 1;
           C2C_R33[i]               = 1;
      }
    }

    //C2C
    int C2C_Num_of_Cams;
    unsigned int C2C_SensorID[NUM_CAM2CAM];
    CameraInfo::CameraInstance C2C_camName[NUM_CAM2CAM];
    State C2C_state[NUM_CAM2CAM];
    int C2C_stateDegradeCause[NUM_CAM2CAM];
    int FS_C2C_Calibration_Out_Of_Range[NUM_CAM2CAM];
    float C2C_Tx[NUM_CAM2CAM];
    float C2C_Ty[NUM_CAM2CAM];
    float C2C_Tz[NUM_CAM2CAM];
    float C2C_R11[NUM_CAM2CAM];
    float C2C_R12[NUM_CAM2CAM];
    float C2C_R13[NUM_CAM2CAM];
    float C2C_R21[NUM_CAM2CAM];
    float C2C_R22[NUM_CAM2CAM];
    float C2C_R23[NUM_CAM2CAM];
    float C2C_R31[NUM_CAM2CAM];
    float C2C_R32[NUM_CAM2CAM];
    float C2C_R33[NUM_CAM2CAM];

    //C2W
    int C2W_yaw;
    int C2W_pitch;
    float C2W_roll;
    float C2W_cameraHeight;
    float C2W_stateConf;
    State C2W_state;
    int C2W_stateDegradeCause;
    int FS_C2W_Calibration_Out_Of_Range;
    float C2W_R[9];
    float C2W_T[3];
    float C2W_HighState_R[9];
    float C2W_HighState_T[3];

    //SPC
    CalibrationStatus SPC_Status;
    int SPC_Progress;
    SPCError SPC_Error;
    int SPC_Session_Number;
    bool SPC_Frame_Valid;
    int SPC_Invalid_Signal;
    int SPC_Invalid_Reason;
    int SPC_Baseline_Yaw;
    int SPC_Baseline_Pitch;
    float SPC_Baseline_Height;
    float SPC_Baseline_Roll;
};

struct OnlineCalibrationModelIF {

    OnlineCalibrationModelIF() : ASIL_CRC(0), data() {
    }

    //CRC
    unsigned short ASIL_CRC;

    OnlineCalibrationModelIFData data;
};

struct EOnlineCalibrationModelIF {
    EOnlineCalibrationModelIF() {
        std::memset(this, 0, sizeof(*this));
        available = false;
    }
    bool available;

    //CRC
    unsigned short ASIL_CRC;

    OnlineCalibrationModelIFData data;
};

}
#endif // __ONLINE_CALIBRATION_MODEL_IF_H_
