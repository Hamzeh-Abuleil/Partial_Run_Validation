/*
 * Cam2CamOutput.cpp
 *
 *  Created on: Dec 10, 2019
 *      Author: sarap
 */

#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamOutput.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"

namespace OnlineCalibration {

namespace Cam2Cam {

Cam2CamOutput& Cam2CamOutput::instance()
{
    static Cam2CamOutput& single = *new Cam2CamOutput;
    return single;
}


Cam2CamOutput::Cam2CamOutput()
{
    for (int i=0; i<e_ITRK_NUM_TYPES; ++i)
    {
        _itrkHandle[i] = nullptr;
    }
}

// ------------------------------ itrk      ------------------------------------------
void Cam2CamOutput::toItrkHeaders()
{
    if (!itrkWriter::isItrkActive())
    {
        return;
    }

    if (_itrkHandle[ItrkType::e_OUTPUT])
    {
        return;
    }

    MEtl::string headers[e_ITRK_NUM_TYPES];

    headers[e_OUTPUT]                    = "OCC2C Output Rxx Rxy Rxz Ryx Ryy Ryz Rzx Rzy Rzz tx ty tz conf std state";
    headers[e_MODEL_IF]                  = "OCC2C ModelIF C2C_Num_of_Cams C2C_SensorID C2C_camName "
                                           "C2C_state C2C_stateDegradeCause FS_C2C_Calibration_Out_Of_Range "
                                           "C2C_Tx C2C_Ty C2C_Tz C2C_R11 C2C_R12 C2C_R13 C2C_R21 C2C_R22 C2C_R23 "
                                           "C2C_R31 C2C_R32 C2C_R33";
    Cam2CamInternalStateInfo justForHeaders;
    MEtl::string internal_state_headers;
    justForHeaders.getHeaders(internal_state_headers);
    headers[e_CALIBRATOR_INTERNAL_STATE] = "OCC2C InternalState" + internal_state_headers;

    for (int i = 0; i < e_ITRK_NUM_TYPES; ++i)
    {
        _itrkHandle[i] = itrkWriter::registerItrkFormat(headers[i].c_str());
    }
}

void Cam2CamOutput::toItrkInternalState(const ExtrinsicCalibration &ec, const Cam2CamInternalStateInfo &isi) const
{
    if (!itrkWriter::isItrkActive())
    {
        return;
    }

    itrkWriter::writeRecordArr(_itrkHandle[ItrkType::e_CALIBRATOR_INTERNAL_STATE],
                               coordsToInstance(ec.getTargets()[1]), isi.getValues(), isi.getNumOfValues());
}
void Cam2CamOutput::toItrkOutput(State state, const ExtrinsicCalibration &ec, const StateInfo &si) const
{
    if (!itrkWriter::isItrkActive())
    {
        return;
    }

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_OUTPUT],
                            /* OCC2W Output     */
                            /* 04 camPort       */ coordsToInstance(ec.getTargets()[1]),
                            /* 05 Rxx           */ ec.getR()[0],
                            /* 06 Rxy           */ ec.getR()[1],
                            /* 07 Rxz           */ ec.getR()[2],
                            /* 08 Ryx           */ ec.getR()[3],
                            /* 09 Ryy           */ ec.getR()[4],
                            /* 10 Ryz           */ ec.getR()[5],
                            /* 11 Rzx           */ ec.getR()[6],
                            /* 12 Rzy           */ ec.getR()[7],
                            /* 13 Rzz           */ ec.getR()[8],
                            /* 14 tx            */ ec.getT()[0],
                            /* 15 ty            */ ec.getT()[1],
                            /* 16 tz            */ ec.getT()[2],
                            /* 17 conf          */ si.getConfidence(),
                            /* 18 std           */ si.getStd(),
                            /* 19 state         */ (float)state);
}

void Cam2CamOutput::toItrkModelIF(const OnlineCalibrationModelIF *m, int index) const
{
    if (!itrkWriter::isItrkActive())
    {
        return;
    }

    itrkWriter::writeRecord(_itrkHandle[ItrkType::e_MODEL_IF],
                            /* OCC2C ModelIF                      */
                            /* 04 camPort                         */ m->data.C2C_camName[index],
                            /* 05 C2C_Num_of_Cams                 */ m->data.C2C_Num_of_Cams,
                            /* 06 C2C_SensorID                    */ m->data.C2C_SensorID[index],
                            /* 07 C2C_camName                     */ m->data.C2C_camName[index],
                            /* 08 C2C_state                       */ m->data.C2C_state[index],
                            /* 09 C2C_stateDegradeCause           */ m->data.C2C_stateDegradeCause[index],
                            /* 10 FS_C2C_Calibration_Out_Of_Range */ m->data.FS_C2C_Calibration_Out_Of_Range[index],
                            /* 11 C2C_Tx                          */ m->data.C2C_Tx[index],
                            /* 12 C2C_Ty                          */ m->data.C2C_Ty[index],
                            /* 13 C2C_Tz                          */ m->data.C2C_Tz[index],
                            /* 14 C2C_R11                         */ m->data.C2C_R11[index],
                            /* 15 C2C_R12                         */ m->data.C2C_R12[index],
                            /* 16 C2C_R13                         */ m->data.C2C_R13[index],
                            /* 17 C2C_R21                         */ m->data.C2C_R21[index],
                            /* 18 C2C_R22                         */ m->data.C2C_R22[index],
                            /* 19 C2C_R23                         */ m->data.C2C_R23[index],
                            /* 20 C2C_R31                         */ m->data.C2C_R31[index],
                            /* 21 C2C_R32                         */ m->data.C2C_R32[index],
                            /* 22 C2C_R33                         */ m->data.C2C_R33[index]
                           );
}

} /* namespace Cam2Cam */

} /* namespace OnlineCalibration */
