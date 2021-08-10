/*
 * test_Cam2Cam.cpp
 *
 *  Created on: Jun 03, 2020
 *      Author: sarap
 */

#include "test_Cam2Cam.h"

//note - the order of the functions in the tests is important!!

namespace OnlineCalibration {
namespace Cam2Cam {

Cam2CamTest::Cam2CamTest() : vec(Float::MEmath::zeros<3, float>()), Mat3(Float::MEmath::identity<3, float>()),
                             manager({CoordSys::FORWARD, CoordSys::NARROW}) {
    manager.init();
}

Cam2CamTest::~Cam2CamTest() {
}

TEST_F(Cam2CamTest, TestCam2CamShouldRun){
    EXPECT_EQ(RunConfig::RUN, manager.shouldRun());
}

TEST_F(Cam2CamTest, TestCam2CamPrepSources){
    EXPECT_FALSE(manager.prepSources());
}

TEST_F(Cam2CamTest, TestCam2CamR){
    manager.calcResult();
    EXPECT_EQ(Mat3, (Float::MEmath::Mat<3,3,float>(manager.getCurrentCalibration().calibration.getR())));
}

TEST_F(Cam2CamTest, TestCam2CamT){
    manager.calcResult();
    EXPECT_EQ(vec, (Float::MEmath::Vec<3,float>(manager.getCurrentCalibration().calibration.getT())));
}

TEST_F(Cam2CamTest, TestCam2CamState){
    manager.calcResult();
    EXPECT_EQ(State::SUSPECTED, manager.getCurrentCalibration().state);
}
}
}
