from adbPyDefs import *
from IDebugger_adb import *


def wmbc_agenda(adb, calcFailSafe = True, doingVcl = True, doingSLI = True, doingOnlineCalibration = False):
    init_in = ['applicationData_Properties', 'camera_Properties', 'INIT_DISTORTION_CORRECTION_ALGO']
    wmbc_in = ['WM_ROAD_MODEL', 'updateEgoMotionResults_EXP_0', 'EGOMOTION_EVAL_PITCH_YAW']
    if (calcFailSafe):
        wmbc_in += ['updateEffectiveExposureC1_DONE']

    adb.push_attrs({'module':'WMBC'})
    adb.init(name='WMBC',            technology='TAC', module='WMBC', res=ARMC, input=init_in, output=['INIT_WMBC'])
    adb.init(name='WMBCImages',            technology='TAC', module='WMBC', res=ARMC, input=['INIT_PREP', 'INIT_WMBC'])
    adb.goal(name='slcFindTargets',         technology='TAC', module='WMBC', res=ARMC, input=[adb.deps['WMBC']['REGULAR']['SLOW'][0],adb.deps['WMBC']['REGULAR']['RAW'][0]], output=['WMBC_TARGETS'])
    adb.goal(name='runWMBC',         technology='TAC', module='WMBC', res=ARMC, input=wmbc_in, output=['RUN_WMBC'])



    # mimicking autofix dependencies (due to IF override and/or setting camera properties)
    # if (doingVcl):
    #     adb.goal(name='wmbcFillModelIF', input=['vclMatching','vclCliquesPredictKalman' ]) #reverse dependecy (failsafes)
    wmbcFillModelIF_input =['RUN_WMBC', adb.deps['WMBC']['REGULAR']['POST_REPORTER'][3]]
    if calcFailSafe:
        wmbcFillModelIF_input += ['CALIB_FAILSAFES'] # reverse
        if doingSLI:
            wmbcFillModelIF_input +=['TSR_OUT_OF_CALIB_FAILSAFES'] # reverse
        adb.goal(name='endFailSafes', technology='failsafes', module='failsafes', input=['WMBC_MODEL'])
    if doingOnlineCalibration:
        wmbcFillModelIF_input += ['OnlineCalibration_MODEL']

    #adb.goal(name='wmbcFillModelIF',         technology='TAC', module='WMBC', res=ARMC, tasks=tech_out_wmbc_task, input=['RUN_WMBC', adb.deps['WMBC']['REGULAR']['POST_REPORTER'][3]], output=['WMBC_MODEL']) 
    adb.goal(name='wmbcFillModelIF',         technology='TAC', module='WMBC', res=ARMC, data_output=HIGH_LEVEL, input=wmbcFillModelIF_input, output=['WMBC_MODEL'])

    wmbcPrepareInputEndFrame_input = ['WMBC_MODEL']
    if calcFailSafe:
        wmbcPrepareInputEndFrame_input += ['END_FAILSAFES']
    adb.goal(name='wmbcPrepareInputEndFrame', technology='TAC', module='WMBC', res=ARMC, input=wmbcPrepareInputEndFrame_input)


    IDebugger_user(adb, 'slcFindTargets')
    adb.pop_attrs()
