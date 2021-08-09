#ifndef STATIONLESS_TARGET_FINDER_DEBUGGER_H_
#define STATIONLESS_TARGET_FINDER_DEBUGGER_H_

#ifndef MEwin
namespace WMBC {class StationlessTargetFinder_Debugger;}
#else
// #include "technology/mobilib/fix/common/MEXimage/imageToWin.h"

namespace IDBG {
// class IDebugger;
// class IDebuggerDataGeneric;
  struct IDbgWindow;
}


namespace Fix {
  namespace MEimage {
    class Coordinate;
    class Win;
    class ColorPixel;
    class Size;
    template < typename T > class tImage;
    template < class T > class Sync;
  }
}

namespace SaddlePoints {
  namespace Types {
    struct SPInfo;
  }
}

namespace MEtypes {
  template<class T> class ptr_vector;
}

namespace WMBC {
  class StationlessTargetFinder;

  class StationlessTargetFinder_Debugger {
  public:
    enum Stage {e_INVALID_STAGE = -1,
                e_INIT,
                e_START,
                e_RAW_SP,
                e_POLES_CANDS,
                e_POLES_CHOSEN,
                e_POLES_CLEAR,
                e_FINAL_TARGETS,
                e_END,
                e_TEST,
                e_NUM_STAGES
    };

    enum Layer {e_L_IMAGE,
                e_L_RAW_SP,
                e_L_POLES_CANDS,
                e_L_POLES_CHOSEN,
                e_L_POLES_CLEAR,
                e_L_FINAL_TARGETS,
                e_NUM_LAYERS
    };

    StationlessTargetFinder_Debugger(StationlessTargetFinder* stationlessTargetFinder);
    void updateStage(Stage);
    void setInputSp(MEtypes::ptr_vector<SaddlePoints::Types::SPInfo> &sp);

  private:
    void init();
    void start();
    void drawRawSp();
    void drawCandsSp();
    void drawPolesChosenSp();
    void drawTargetCorners();
    void drawFinalTargets();
    void end();
    
    void test();

    void drawSp(MEtypes::ptr_vector<SaddlePoints::Types::SPInfo> &spVec, Layer layer, Fix::MEimage::ColorPixel color);
    void drawSpSub(MEtypes::ptr_vector<SaddlePoints::Types::SPInfo> &spVec, Layer layer, Fix::MEimage::ColorPixel color);
    void drawSpSubDist(MEtypes::ptr_vector<SaddlePoints::Types::SPInfo> &spVec, Layer layer, Fix::MEimage::ColorPixel color);

    static IDBG::IDbgWindow* createMainWin(std::string winName, Fix::MEimage::Size & sizeL1, const Fix::MEimage::Coordinate &pos, bool writePPMs);

    StationlessTargetFinder* _targetFinder;
    MEtypes::ptr_vector<SaddlePoints::Types::SPInfo> *_inSp;
    const Fix::MEimage::Sync < Fix::MEimage::tImage<unsigned char> > *_img;
    int _left;
    int _bottom;
    int _leftRect;
    int _bottomRect;
  };

} // WMBC

#endif // MEwin
#endif // STATIONLESS_TARGET_FINDER_DEBUGGER_H_
