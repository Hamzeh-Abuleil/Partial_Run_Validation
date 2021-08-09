/*
 * ExtrinsicCalibrationProperties.h
 *
 *  Created on: Nov 21, 2019
 *      Author: sarap
 */

#ifndef _ONLINECALIBRATION_EXTRINSICCALIBRATIONPROPERTIES_H_
#define _ONLINECALIBRATION_EXTRINSICCALIBRATIONPROPERTIES_H_

#include "functionality/calibration/Properties.h"
#include "functionality/calibration/SeriesProperties.h"
#include "utilities/clipextIO/clipextIO.h"
#include "functionality/calibration/verifiers/properties/PropertyVerifier.h"

namespace OnlineCalibration {

class CalibrationProperties : public Properties {

public:
    CalibrationProperties(const char* section=NULL);
    ~CalibrationProperties() {}

    virtual void init(const char* section);
    void dumpData(ClipextIO::ClipextWriter& writer);
    const float* const getR() const { return _camR()._seriesArray;}
    const float* const getT() const { return _camT()._seriesArray;}

private:
    RWProperT< Series<float, 9>, PropertyVerifier> _camR;
    RWProperT< Series<float, 3>, PropertyVerifier> _camT;
};

class CalibrationBaselineProperties : public Properties {

public:
    CalibrationBaselineProperties(const char* section=NULL);
    ~CalibrationBaselineProperties() {}

    virtual void init(const char* section);
    void dumpData(ClipextIO::ClipextWriter& writer);

private:
    RWProperT< Series<float, 9>, PropertyVerifier> _baselineCamR;
    RWProperT< Series<float, 3>, PropertyVerifier> _baselineCamT;
};

class Cam2WorldProperties :public Properties {

public:
    Cam2WorldProperties(const char* section=NULL);
    ~Cam2WorldProperties() {}

    virtual void init(const char* section);
    void dumpData(ClipextIO::ClipextWriter& writer);

private:
    RWProperT< float, PropertyVerifier> _yaw;
    RWProperT< float, PropertyVerifier> _pitch;
    RWProperT< float, PropertyVerifier> _roll;
    RWProperT< float, PropertyVerifier> _camH;
};

class Cam2WorldBaseLineProperties :public Properties {

public:
    Cam2WorldBaseLineProperties(const char* section=NULL);
    ~Cam2WorldBaseLineProperties() {}

    virtual void init(const char* section);
    void dumpData(ClipextIO::ClipextWriter& writer);

    bool getSpcMode() const { return _SpcMode();}
    bool getSlowMode() const { return _SlowMode();}

private:
    ProperT< float, PropertyVerifier> _baselineYaw;
    ProperT< float, PropertyVerifier> _baselinePitch;
    ProperT< float, PropertyVerifier> _baselineRoll;
    ProperT< float, PropertyVerifier> _baselineCamH;
    ProperT<bool> _SpcMode;
    ProperT<bool> _SlowMode;
};

}

#endif /* _ONLINECALIBRATION_EXTRINSICCALIBRATIONPROPERTIES_H_ */
