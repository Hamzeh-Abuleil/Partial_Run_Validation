#ifndef __OC_CAM2CAM_EM_SOURCES_H
#define __OC_CAM2CAM_EM_SOURCES_H
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "utilities/clipextIO/clipextIO.h"

namespace OnlineCalibration {
namespace Cam2Cam {

struct EmPair {
    Float::MEmath::Mat<4, 4, double> source;
    Float::MEmath::Mat<4, 4, double> target;
};

struct Cam2CamEgoMotionSources {
    void dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
        writer.setData("TO_source", &T0.source, 4*4);
        writer.setData("TO_target", &T0.target, 4*4);
        writer.setData("T1_source", &T1.source, 4*4);
        writer.setData("T1_target", &T1.target, 4*4);
#endif
    }
    EmPair T0;
    EmPair T1;
};

}
}

#endif
