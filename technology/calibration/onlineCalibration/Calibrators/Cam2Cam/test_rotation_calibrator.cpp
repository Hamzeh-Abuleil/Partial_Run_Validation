/**
 * \file test_rotation_calibrator.cpp
 * \brief Unit test for cam2cam calibration
 * 
 * \author Amit Hochman
 * \date Dec 13, 2018
 */
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>

#include "matio.h"
//#include "calibMatUtils.h"
#include "egomotionCalibration_API.h"

int main(int argc, char *argv[])
{
    std::cout << std::setprecision(16);
    std::vector<Float::MEmath::Mat<4, 4, double>>
        emMain, emCam;
    std::string filenameIn, filenameOut;
    if (argc < 2)
    {
        std::cout << "Using default file names.\n";
        std::string sDir = "/homes/amitho/nonrepos/matlab/egomotionCalib/";
        filenameOut = sDir + "out.hdf";
        filenameIn = sDir + "test.hdf";
    }
    else if (argc != 3)
    {
        std::cout << "Usage: test_rotation_calibrator <input_filename.hdf> <output_filename.hdf>\n";
        return 0;
    }
    else
    {
        filenameIn = argv[1];
        filenameOut = argv[2];
    }

    std::cout << "Opening Egomotion data file: " << filenameIn << "... ";
    bool ok = CalibUtils::loadHdf5(filenameIn, "emMain", emMain);
    if (!ok)
    {
        std::cout << "Failed. Aborting.\n";
        return 1;
    }
    else
    {
        std::cout << "done.\n";
    }
    ok = CalibUtils::loadHdf5(filenameIn, "emCam", emCam);
    if (!ok)
    {
        std::cout << "Failed. Aborting.\n";
        return 1;
    }

    std::vector<Float::MEmath::Mat<3, 3, double>> R(emMain.size());
    std::vector<Float::MEmath::Mat<3, 1, double>> t(emMain.size());
    std::cout << "Computing calibration... ";
    arma::wall_clock timer;
    timer.tic();
    // e_FORWARD and e_REAR are the default values, so we don't really need to
    // do the init below if those are the cameras.
    EgomotionCalibration::init(CameraInfo::CameraInstance::e_FORWARD, CameraInfo::CameraInstance::e_REAR);
    Float::MEmath::Vec<3, double> currT;
    for (unsigned int i = 0; i < emMain.size(); i++)
    {
        //CalibUtils::printMat(emMain[i], "emMain = \n");
        //CalibUtils::printMat(emCam[i], "emCam = \n");
        EgomotionCalibration::updateCalibrator(emMain[i], emCam[i]);
        EgomotionCalibration::computeCalibration();
        EgomotionCalibration::getRotation(R[i]);
        EgomotionCalibration::getTranslation(currT);
        t[i] = Float::MEmath::Mat<3,1>(currT);
        //CalibUtils::printMat(C[i], "C = \n");
    }
    std::cout << "Done.\nThat took: " << timer.toc() << " seconds.\n";
    std::cout << "Writing results to " << filenameOut << "... ";
    CalibUtils::saveHdf5(filenameOut, "rotation", R);
    CalibUtils::saveHdf5(filenameOut, "translation", t, "u");
    std::cout << "done.\n";
    return 0;
}