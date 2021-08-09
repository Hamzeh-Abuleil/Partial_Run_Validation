/**
 * \file convolver1d.cpp
 * \brief ConvolverOneDim
 *
 * \author Uri London
 * \date Jan 27, 2020
 */

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/convolver1d.h"

namespace OnlineCalibration {
  namespace Cam2World {


  ConvolverOneDim::ConvolverOneDim() : _result(0.f) {
    _kernelType = (KernelType)Debug::Args::instance().getStickyValue("-sOCC2W-kernelType", e_KERNEL_EXP);
    _initWidth = Debug::Args::instance().getStickyValue("-sOCC2W-kernelWidth", 128);
    // ASSERT(_width > EPS);
    std::fill(std::begin(_kernel), std::end(_kernel), 0.f);
    _valueBuff.alloc(CONVOLVE_WIN_SIZE);
    reset();
    setKernel(_kernelType);
  }

  void ConvolverOneDim::reset() {
    _result = 0.f;
    _valueBuff.clear();
    _width = _initWidth;
  }

  void ConvolverOneDim::setKernel(const KernelType kernelType) {
    float (ConvolverOneDim::*kernel)(unsigned int) = nullptr;
    switch (kernelType) {
      case e_KERNEL_EXP:
        kernel = &ConvolverOneDim::kernelExp;
        break;
      default:
        kernel = &ConvolverOneDim::kernelExp;
    }

    for (unsigned int i=0; i<CONVOLVE_WIN_SIZE; ++i) {
      _kernel[i] = (this->*kernel)(i);
    }
  }

  float ConvolverOneDim::update(float x) {
    _valueBuff.push_back(x);

    unsigned int size = _valueBuff.size();
    _result = 0.f;

    for (unsigned int i=0; i<size; ++i) {
      _result += _kernel[i]*_valueBuff[-i];
    }

    return _result;
  }

  void ConvolverOneDim::updateWidth(float width){
    _width = width;
    setKernel(_kernelType);
  }

  float ConvolverOneDim::kernelExp(unsigned int i) {
    float invW = 1.f/_width;
    float N = (me_expf(invW)-1.f)/(me_expf(invW*CONVOLVE_WIN_SIZE)-1.f);
    return N*me_expf(invW*(CONVOLVE_WIN_SIZE-1-i));
  }

  } // Cam2World
} // OnlineCalibration
