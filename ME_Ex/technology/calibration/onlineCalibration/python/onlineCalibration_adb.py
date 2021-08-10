from adbPyDefs import *

def onlineCalibration_agenda(adb):

    adb.push_attrs({'module':'OC'})

    C2C_INIT_DEPS = [] #TODO - fill
    C2W_INIT_DEPS = ['INIT_DISTORTION_CORRECTION_ALGO',  'camera_Properties']

    adb.init(name='OnlineCalibration_init',
             res=ARMC,
             input=C2C_INIT_DEPS + C2W_INIT_DEPS,
             output=['OnlineCalibration_INIT_DONE'],
             technology='OnlineCalibration',
             module='init');

    C2C_RUN_DEPS = ['WM_EGO_MOTION_UM_T0_DONE', 'WM_EGO_MOTION_UM_T1_DONE']
    C2W_RUN_DEPS = ['WM_ROAD_MODEL', 'updateEgoMotionResults_EXP_0']

    adb.goal(name='OnlineCalibration_run',
             res=ARMC,
             input=C2C_RUN_DEPS + C2W_RUN_DEPS,
             output=['OnlineCalibration_RUN_DONE'],
             technology='OnlineCalibration',
             data_output='COMMON_LEVEL',
             module='OC');

    OC_MODEL_IF_INPUT = ['OnlineCalibration_RUN_DONE', adb.deps['General']['REGULAR']['POST_REPORTER'][3]]
    adb.goal(name="OnlineCalibration_fillModelIF",
             res=ARMC,
             input=OC_MODEL_IF_INPUT,
             output=['OnlineCalibration_MODEL'],
             technology='OnlineCalibration',
             data_output='COMMON_LEVEL',
             module='OC');

    adb.goal(name="OnlineCalibration_UpdateDriverProfileInput",
             res=ARMC,
             input=[],
             output=['OnlineCalibration_INPUT_UPDATE'],
             technology='OnlineCalibration',
             data_output='COMMON_LEVEL',
             module='OC');

    adb.pop_attrs()
