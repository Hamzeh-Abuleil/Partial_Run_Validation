#ifndef _WMBC_SLC_DEBUGGER_H_
#define _WMBC_SLC_DEBUGGER_H_

#ifndef MEwin
namespace WMBC {class FOEFinder_Stationless_Debugger;}
#else

namespace IDBG {
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

namespace Fix {
  namespace MEstd {
    template<class T> class ptr_vector;
  }
}

namespace WMBC {
  class FOEFinder_Stationless;

  class FOEFinder_Stationless_Debugger {
  public:
    enum Stage {e_INVALID_STAGE = -1,
                e_INIT,
                e_START,
                e_SEARCH_WIN,
                e_TARGETS,
                e_END,
                e_TEST,
                e_NUM_STAGES
    };

    enum Layer {e_L_IMAGE,
                e_L_SEARCH_AREA,
                e_L_SP,
                e_L_SEARCH_WIN,
                e_L_TARGETS,
                e_NUM_LAYERS
    };

    FOEFinder_Stationless_Debugger(FOEFinder_Stationless* foeFinder_Stationless);
    void updateStage(Stage);
    
  private:
    void init();
    void start();
    void end();

    void drawSearchWin();
    void drawTargets();
    void test();

    static IDBG::IDbgWindow* createMainWin(std::string winName, Fix::MEimage::Size & sizeL1, const Fix::MEimage::Coordinate &pos, bool writePPMs);

    FOEFinder_Stationless* _foeFinder;
    const Fix::MEimage::Sync < Fix::MEimage::tImage<unsigned char> > *_img;
    int _left;
    int _bottom;
    int _leftRect;
    int _bottomRect;
  };

} // WMBC

#endif // MEwin
#endif // _WMBC_SLC_DEBUGGER_H_
