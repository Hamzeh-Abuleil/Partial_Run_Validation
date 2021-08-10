/**
 * \file calibTypes.h
 * \brief Typedef's used in the CalibUtils namespace.
 * 
 * \author Amit Hochman
 * \date Dec 13, 2018
 */
#pragma once
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "basicTypes/geo/geo2D/Pixel.h"
#include <vector>
namespace CalibUtils
{
typedef Float::MEmath::Mat<3, 3, double> mat33;
typedef Float::MEmath::Mat<3, 3, float> mat33f;
typedef Float::MEmath::Mat<4, 4, double> mat44;
typedef Float::MEmath::Mat<4, 4, float> mat44f;
typedef Float::MEmath::Mat<9, 9, double> mat99;
typedef Float::MEmath::Vec<3, double> vec3;
typedef Float::MEmath::Vec<3, float> vec3f;
typedef Float::MEmath::Vec<9, double> vec9;
typedef Float::MEmath::Mat<2, 2, float> mat22f;
typedef Float::MEmath::Mat<2, 2, double> mat22;
typedef Float::MEmath::Vec<2, float> vec2f;
typedef std::vector<mat44> tformVec;
typedef std::vector<std::pair<mat44, mat44>> tformPairVec;
typedef MEtypes::OldPixel<float, MEtypes::DistortionModel::DM_ORIGINAL, MEtypes::RealCamInstance::RCI_FORWARD, MEtypes::PixLevel::PIXLEVEL_M2> PixelLm2_f;
typedef MEtypes::OldPixel<int, MEtypes::DistortionModel::DM_ORIGINAL, MEtypes::RealCamInstance::RCI_FORWARD, MEtypes::PixLevel::PIXLEVEL_M2> PixelLm2;

Float::MEmath::Mat<3, 3, double> matf2d(Float::MEmath::Mat<3, 3, float> matf);
Float::MEmath::Mat<3, 3, float> matd2f(Float::MEmath::Mat<3, 3, double> matd);
Float::MEmath::Mat<4, 4, float> matd2f(Float::MEmath::Mat<4, 4, double> matd);
Float::MEmath::Vec<3, float> vecd2f(Float::MEmath::Vec<3, double> vecd);

const auto id3 = Float::MEmath::identity<3, double>;
const auto zVec3 = Float::MEmath::zeros<3, double>;
const auto id3f = Float::MEmath::identity<3, float>;
const auto zVec3f = Float::MEmath::zeros<3, float>;
const auto id2f = Float::MEmath::identity<2, float>;
const auto zVec2f = Float::MEmath::zeros<2, float>;
}
