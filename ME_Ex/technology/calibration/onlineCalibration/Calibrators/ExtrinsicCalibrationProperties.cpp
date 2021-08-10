/*
 * ExtrinsicCalibrationparams.cpp
 *
 *  Created on: Nov 21, 2019
 *      Author: sarap
 */

#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibrationProperties.h"
#include "functionality/calibration/propertiesManager.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wData.h"
#include "functionality/calibration/PropertiesValidators.h"

const float IDENTITY_MAT_3[] = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
const float ZERO_VEC_3[] = {0., 0., 0.};

extern bool LoadProperDies(Properties& p, const char* path, const char* sec = NULL);

namespace OnlineCalibration {

CalibrationProperties::CalibrationProperties(const char* section) :
      Properties(section == NULL? MEtl::string() : MEtl::string(section)),
      _camR (_me, Series<float, 9>(IDENTITY_MAT_3), "camR", "rotation matrix", Property::DEFAULT_ME_FLAGS),
      _camT (_me, Series<float, 3>(ZERO_VEC_3), "camT", "translation matrix", Property::DEFAULT_ME_FLAGS)
{
}

void CalibrationProperties::init(const char* section)
{
    MEtl::string verifiersSectionName = "calibration_" + MEtl::string(section);
    this->setName(verifiersSectionName);//for verifiers data base
    LoadProperDies(*this, "etc/camera.conf", section);
}

void CalibrationProperties::dumpData(ClipextIO::ClipextWriter& writer)
{
    writer.setData("camR", _camR()._seriesArray, 3*3);
    writer.setData("camT", _camT()._seriesArray, 3);
}

CalibrationBaselineProperties::CalibrationBaselineProperties(const char* section) :
      Properties(section == NULL? MEtl::string() : MEtl::string(section)),
      _baselineCamR (_me, Series<float, 9>(IDENTITY_MAT_3), "baselineCamR", "baseline rotation matrix", Property::DEFAULT_ME_FLAGS),
      _baselineCamT (_me, Series<float, 3>(ZERO_VEC_3), "baselineCamT", "baseline translation matrix", Property::DEFAULT_ME_FLAGS)
{
}

void CalibrationBaselineProperties::init(const char* section)
{
    LoadProperDies(*this, "etc/camera.conf", section);
}

void CalibrationBaselineProperties::dumpData(ClipextIO::ClipextWriter& writer)
{
    writer.setData("baselineR", _baselineCamR()._seriesArray, 3*3);
    writer.setData("baselineT", _baselineCamT()._seriesArray, 3);
}


Cam2WorldProperties::Cam2WorldProperties(const char* section) :
      Properties(section == NULL? MEtl::string() : MEtl::string(section)),
      _yaw (_me, 0, "yaw", "yaw", Property::DEFAULT_ME_FLAGS),
      _pitch (_me,0, "pitch", "pitch", Property::DEFAULT_ME_FLAGS),
      _roll (_me, 0, "roll", "roll", Property::DEFAULT_ME_FLAGS),
      _camH (_me, 0, "camH", "camera height", Property::DEFAULT_ME_FLAGS)
{
}

void Cam2WorldProperties::init(const char* section)
{
    MEtl::string verifiersSectionName = "calibration_" + MEtl::string(section);
    this->setName(verifiersSectionName);//for verifiers data base
    LoadProperDies(*this, "etc/camera.conf", section);
}

void Cam2WorldProperties::dumpData(ClipextIO::ClipextWriter& writer)
{
    writer.setData("yaw", &_yaw());
    writer.setData("pitch", &_pitch());
    writer.setData("roll", &_roll());
    writer.setData("camH", &_camH());
}

Cam2WorldBaseLineProperties::Cam2WorldBaseLineProperties(const char* section) :
      Properties(section == NULL? MEtl::string() : MEtl::string(section)),
      _baselineYaw (_me, 0, "baselineYaw", "baseline yaw", Property::DEFAULT_ME_FLAGS),
      _baselinePitch (_me, 0, "baselinePitch", "baseline pitch", Property::DEFAULT_ME_FLAGS),
      _baselineRoll (_me, 0, "baselineRoll", "baseline roll", Property::DEFAULT_ME_FLAGS),
      _baselineCamH (_me, 0, "baselineCamH", "baseline camera height", Property::DEFAULT_ME_FLAGS),
      _SpcMode (_me, false, "baselineSpcMode", "baseline spc mode", Property::DEFAULT_ME_FLAGS, nullptr, TrueFalseValidator(this)),
      _SlowMode (_me, false, "baselineSlowMode", "baseline slow mode", Property::DEFAULT_ME_FLAGS, nullptr, TrueFalseValidator(this))
{
}

void Cam2WorldBaseLineProperties::init(const char* section)
{
    LoadProperDies(*this, "etc/camera.conf", section);
}

void Cam2WorldBaseLineProperties::dumpData(ClipextIO::ClipextWriter& writer)
{
    writer.setData("baselineYaw", &_baselineYaw());
    writer.setData("baselinePitch", &_baselinePitch());
    writer.setData("baselineRoll", &_baselineRoll());
    writer.setData("baselineCamH", &_baselineCamH());
}

}
