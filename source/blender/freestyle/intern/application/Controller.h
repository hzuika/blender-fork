/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup freestyle
 * \brief The spinal tap of the system.
 */

#include <string>

#include "../geometry/FastGrid.h"
#include "../scene_graph/SceneHash.h"
#include "../system/Precision.h"
#include "../system/TimeUtils.h"
#include "../view_map/FEdgeXDetector.h"
#include "../view_map/ViewMapBuilder.h"

#ifdef WITH_CXX_GUARDEDALLOC
#  include "MEM_guardedalloc.h"
#endif

namespace Freestyle {

class AppCanvas;
class AppView;
class Interpreter;
class NodeGroup;
class ProgressBar;
class RenderMonitor;
class SShape;
class ViewEdge;
class ViewMap;

class Controller {
 public:
  Controller();
  ~Controller();

  void setView(AppView *iView);
  void setRenderMonitor(RenderMonitor *iRenderMonitor);
  void setPassDiffuse(float *buf, int width, int height);
  void setPassZ(float *buf, int width, int height);
  void setContext(bContext *C);

  // soc
  void init_options();

  int LoadMesh(Render *re, ViewLayer *view_layer, Depsgraph *depsgraph);
  int Load3DSFile(const char *iFileName);
  void CloseFile();
  void ComputeViewMap();
  int DrawStrokes();
  void ResetRenderCount();
  Render *RenderStrokes(Render *re, bool render);
  void SwapStyleModules(unsigned i1, unsigned i2);
  void InsertStyleModule(unsigned index, const char *iFileName);
  void InsertStyleModule(unsigned index, const char *iName, const char *iBuffer);
  void InsertStyleModule(unsigned index, const char *iName, struct Text *iText);
  void RemoveStyleModule(unsigned index);
  void Clear();
  void ClearRootNode();
  void DeleteWingedEdge();
  void DeleteViewMap(bool freeCache = false);
  void toggleLayer(unsigned index, bool iDisplay);
  void setModified(unsigned index, bool iMod);
  void resetModified(bool iMod = false);
  void updateCausalStyleModules(unsigned index);

  ViewEdge *SelectViewEdge(real x, real y);
  FEdge *SelectFEdge(real x, real y);

  void setVisibilityAlgo(int algo);

  void setViewMapCache(bool iBool);
  void setQuantitativeInvisibility(bool iBool);  // if true, we compute quantitativeInvisibility
  void setFaceSmoothness(bool iBool);
  bool getFaceSmoothness() const;

  void setComputeRidgesAndValleysFlag(bool b);
  bool getComputeRidgesAndValleysFlag() const;
  void setComputeSuggestiveContoursFlag(bool b);
  bool getComputeSuggestiveContoursFlag() const;
  void setComputeMaterialBoundariesFlag(bool b);
  bool getComputeMaterialBoundariesFlag() const;

  void setCreaseAngle(float angle)
  {
    _creaseAngle = angle;
  }
  float getCreaseAngle() const
  {
    return _creaseAngle;
  }
  void setSphereRadius(float s)
  {
    _sphereRadius = s;
  }
  float getSphereRadius() const
  {
    return _sphereRadius;
  }
  void setSuggestiveContourKrDerivativeEpsilon(float dkr)
  {
    _suggestiveContourKrDerivativeEpsilon = dkr;
  }
  float getSuggestiveContourKrDerivativeEpsilon() const
  {
    return _suggestiveContourKrDerivativeEpsilon;
  }

  bool hitViewMapCache();

 public:
  // Viewmap data structure
  ViewMap *_ViewMap;

  // Canvas
  AppCanvas *_Canvas;

 private:
  // Main Window:
  // AppMainWindow *_pMainWindow;

  // List of models currently loaded
  vector<string> _ListOfModels;

  // Current directories
  // ConfigIO* _current_dirs;

  // View
  // 3D
  AppView *_pView;

  RenderMonitor *_pRenderMonitor;

  // Model
  // Drawing Structure
  NodeGroup *_RootNode;

  // Winged-Edge structure
  WingedEdge *_winged_edge;

  // debug
  // NodeUser<ViewMap> *_ViewMapNode; // FIXME

  // Chronometer:
  Chronometer _Chrono;

  // Progress Bar
  ProgressBar *_ProgressBar;

  // edges tesselation nature
  int _edgeTesselationNature;

  FastGrid _Grid;
  // HashGrid _Grid;

  BBox<Vec3r> _Scene3dBBox;
  unsigned int _SceneNumFaces;

  real _EPSILON;
  real _bboxDiag;

  int _render_count;

  // AppStyleWindow *_pStyleWindow;
  // AppOptionsWindow *_pOptionsWindow;
  // AppDensityCurvesWindow *_pDensityCurvesWindow;

  ViewMapBuilder::visibility_algo _VisibilityAlgo;

  // Script Interpreter
  Interpreter *_inter;

  string _help_index;
  string _browser_cmd;

  bool _EnableViewMapCache;
  bool _EnableQI;
  bool _EnableFaceSmoothness;
  bool _ComputeRidges;
  bool _ComputeSuggestive;
  bool _ComputeMaterialBoundaries;
  float _creaseAngle;
  float _sphereRadius;
  float _suggestiveContourKrDerivativeEpsilon;

  FEdgeXDetector edgeDetector;

  SceneHash sceneHashFunc;
  real prevSceneHash;

#ifdef WITH_CXX_GUARDEDALLOC
  MEM_CXX_CLASS_ALLOC_FUNCS("Freestyle:Controller")
#endif
};

extern Controller *g_pController;

} /* namespace Freestyle */
