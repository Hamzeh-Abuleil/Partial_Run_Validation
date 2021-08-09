/*
 * test_Cam2Cam.h
 *
 *  Created on: Jun 03, 2020
 *      Author: sarap
 */

#ifndef ME_DEVELOP_TECHNOLOGY_WORLDMODEL_EGOMOTION_MANAGER_STANDALONE_TEST_MANAGER_H_
#define ME_DEVELOP_TECHNOLOGY_WORLDMODEL_EGOMOTION_MANAGER_STANDALONE_TEST_MANAGER_H_

#include "utilities/googleTests/googletest/gtest/include/gtest/gtest.h"
#include "../Cam2CamEgoMotionManager.h"
#include "technology/mobilib/float/common/MEmath/mat.h"

namespace OnlineCalibration {
namespace Cam2Cam {

class Cam2CamTest : public testing::Test
{
public:

  Float::MEmath::Vec<3, float> vec;
  Float::MEmath::Mat<3, 3, float> Mat3;
  Cam2Cam::Cam2CamEgoMotionManager manager;

  Cam2CamTest ();
  ~Cam2CamTest();

  };

}}



#endif /* ME_DEVELOP_TECHNOLOGY_WORLDMODEL_EGOMOTION_MANAGER_STANDALONE_TEST_MANAGER_H_ */
