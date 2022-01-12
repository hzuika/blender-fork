/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/** \file
 * \ingroup freestyle
 */

extern "C" {
#include <Python.h>
}

#include <cfloat>
#include <fstream>
#include <string>

#include "AppCanvas.h"
#include "AppConfig.h"
#include "AppView.h"
#include "Controller.h"

#include "../image/Image.h"

#include "../scene_graph/NodeDrawingStyle.h"
#include "../scene_graph/NodeShape.h"
#include "../scene_graph/NodeTransform.h"
#include "../scene_graph/NodeViewLayer.h"
#include "../scene_graph/ScenePrettyPrinter.h"
#include "../scene_graph/VertexRep.h"

#include "../stroke/PSStrokeRenderer.h"
#include "../stroke/StrokeTesselator.h"
#include "../stroke/StyleModule.h"
#include "../stroke/TextStrokeRenderer.h"

#include "../system/PythonInterpreter.h"
#include "../system/StringUtils.h"

#include "../view_map/SteerableViewMap.h"
#include "../view_map/ViewMap.h"
#include "../view_map/ViewMapTesselator.h"

#include "../winged_edge/Curvature.h"
#include "../winged_edge/WEdge.h"
#include "../winged_edge/WXEdgeBuilder.h"
#include "../winged_edge/WingedEdgeBuilder.h"

#include "../blender_interface/BlenderFileLoader.h"
#include "../blender_interface/BlenderStrokeRenderer.h"
#include "../blender_interface/BlenderStyleModule.h"

#include "BKE_global.h"
#include "BLI_path_util.h"
#include "BLI_utildefines.h"

#include "DNA_freestyle_types.h"

#include "FRS_freestyle.h"

namespace Freestyle {

Controller::Controller()
{
  const string sep(Config::DIR_SEP);

  _RootNode = new NodeGroup;
  _RootNode->addRef();

  _winged_edge = nullptr;

  _pView = nullptr;
  _pRenderMonitor = nullptr;

  _edgeTesselationNature = (Nature::SILHOUETTE | Nature::BORDER | Nature::CREASE);

  _ProgressBar = new ProgressBar;
  _SceneNumFaces = 0;

  _EPSILON = 1.0e-6;
  _bboxDiag = 0;

  _ViewMap = nullptr;

  _Canvas = nullptr;

  _VisibilityAlgo = ViewMapBuilder::ray_casting_culled_adaptive_cumulative;

  _Canvas = new AppCanvas;

  _inter = new PythonInterpreter();
  _EnableViewMapCache = false;
  _EnableQI = true;
  _EnableFaceSmoothness = false;
  _ComputeRidges = true;
  _ComputeSteerableViewMap = false;
  _ComputeSuggestive = true;
  _ComputeMaterialBoundaries = true;
  _sphereRadius = 1.0;
  _creaseAngle = 134.43;
  prevSceneHash = -1.0;

  init_options();
}

Controller::~Controller()
{
  if (nullptr != _RootNode) {
    int ref = _RootNode->destroy();
    if (0 == ref) {
      delete _RootNode;
    }
  }

  if (_winged_edge) {
    delete _winged_edge;
    _winged_edge = nullptr;
  }

  if (nullptr != _ViewMap) {
    delete _ViewMap;
    _ViewMap = nullptr;
  }

  if (nullptr != _Canvas) {
    delete _Canvas;
    _Canvas = nullptr;
  }

  if (_inter) {
    delete _inter;
    _inter = nullptr;
  }

  if (_ProgressBar) {
    delete _ProgressBar;
    _ProgressBar = nullptr;
  }
}

void Controller::setView(AppView *iView)
{
  if (nullptr == iView) {
    return;
  }

  _pView = iView;
  _Canvas->setViewer(_pView);
}

void Controller::setRenderMonitor(RenderMonitor *iRenderMonitor)
{
  _pRenderMonitor = iRenderMonitor;
}

void Controller::setPassDiffuse(float *buf, int width, int height)
{
  AppCanvas *app_canvas = dynamic_cast<AppCanvas *>(_Canvas);
  BLI_assert(app_canvas != nullptr);
  app_canvas->setPassDiffuse(buf, width, height);
}

void Controller::setPassZ(float *buf, int width, int height)
{
  AppCanvas *app_canvas = dynamic_cast<AppCanvas *>(_Canvas);
  BLI_assert(app_canvas != nullptr);
  app_canvas->setPassZ(buf, width, height);
}

void Controller::setContext(bContext *C)
{
  PythonInterpreter *py_inter = dynamic_cast<PythonInterpreter *>(_inter);
  py_inter->setContext(C);
}

bool Controller::hitViewMapCache()
{
  if (!_EnableViewMapCache) {
    return false;
  }
  if (sceneHashFunc.match()) {
    return (nullptr != _ViewMap);
  }
  sceneHashFunc.store();
  return false;
}

int Controller::LoadMesh(Render *re, ViewLayer *view_layer, Depsgraph *depsgraph)
{
  BlenderFileLoader loader(re, view_layer, depsgraph);

  loader.setRenderMonitor(_pRenderMonitor);

  _Chrono.start();

  NodeGroup *blenderScene = loader.Load();

  if (blenderScene == nullptr) {
    if (G.debug & G_DEBUG_FREESTYLE) {
      cout << "Cannot load scene" << endl;
    }
    return 1;
  }

  if (blenderScene->numberOfChildren() < 1) {
    if (G.debug & G_DEBUG_FREESTYLE) {
      cout << "Empty scene" << endl;
    }
    blenderScene->destroy();
    delete blenderScene;
    return 1;
  }

  real duration = _Chrono.stop();
  if (G.debug & G_DEBUG_FREESTYLE) {
    cout << "Scene loaded" << endl;
    printf("Mesh cleaning    : %lf\n", duration);
    printf("View map cache   : %s\n", _EnableViewMapCache ? "enabled" : "disabled");
  }
  _SceneNumFaces += loader.numFacesRead();

  _RootNode->AddChild(blenderScene);
  _RootNode->UpdateBBox();  // FIXME: Correct that by making a Renderer to compute the bbox

  _pView->setModel(_RootNode);
  //_pView->FitBBox();

  if (_pRenderMonitor->testBreak()) {
    return 0;
  }

  if (_EnableViewMapCache) {

    NodeCamera *cam;
    if (g_freestyle.proj[3][3] != 0.0) {
      cam = new NodeOrthographicCamera;
    }
    else {
      cam = new NodePerspectiveCamera;
    }
    double proj[16];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        proj[i * 4 + j] = g_freestyle.proj[i][j];
      }
    }
    cam->setProjectionMatrix(proj);
    _RootNode->AddChild(cam);
    _RootNode->AddChild(new NodeViewLayer(*re->scene, *view_layer));

    sceneHashFunc.reset();
    // blenderScene->accept(sceneHashFunc);
    _RootNode->accept(sceneHashFunc);
    if (G.debug & G_DEBUG_FREESTYLE) {
      cout << "Scene hash       : " << sceneHashFunc.toString() << endl;
    }
    if (hitViewMapCache()) {
      ClearRootNode();
      return 0;
    }

    delete _ViewMap;
    _ViewMap = nullptr;
  }

  _Chrono.start();

  WXEdgeBuilder wx_builder;
  wx_builder.setRenderMonitor(_pRenderMonitor);
  blenderScene->accept(wx_builder);
  _winged_edge = wx_builder.getWingedEdge();

  duration = _Chrono.stop();
  if (G.debug & G_DEBUG_FREESTYLE) {
    printf("WEdge building   : %lf\n", duration);
  }

  _ListOfModels.emplace_back("Blender_models");

  _Scene3dBBox = _RootNode->bbox();

  _bboxDiag = (_RootNode->bbox().getMax() - _RootNode->bbox().getMin()).norm();
  if (G.debug & G_DEBUG_FREESTYLE) {
    cout << "Triangles nb     : " << _SceneNumFaces << " imported, " << _winged_edge->getNumFaces()
         << " retained" << endl;
    cout << "Bounding Box     : " << _bboxDiag << endl;
  }

  ClearRootNode();

  _SceneNumFaces = _winged_edge->getNumFaces();
  if (_SceneNumFaces == 0) {
    DeleteWingedEdge();
    return 1;
  }

  return 0;
}

void Controller::CloseFile()
{
  WShape::setCurrentId(0);
  _ListOfModels.clear();

  // We deallocate the memory:
  ClearRootNode();
  DeleteWingedEdge();
  DeleteViewMap();

  // clears the canvas
  _Canvas->Clear();

  // soc: reset passes
  setPassDiffuse(nullptr, 0, 0);
  setPassZ(nullptr, 0, 0);
}

void Controller::ClearRootNode()
{
  _pView->DetachModel();
  if (nullptr != _RootNode) {
    int ref = _RootNode->destroy();
    if (0 == ref) {
      _RootNode->addRef();
    }
    _RootNode->clearBBox();
  }
}

void Controller::DeleteWingedEdge()
{
  if (_winged_edge) {
    delete _winged_edge;
    _winged_edge = nullptr;
  }

  // clears the grid
  _Grid.clear();
  _Scene3dBBox.clear();
  _SceneNumFaces = 0;
}

void Controller::DeleteViewMap(bool freeCache)
{
  if (nullptr != _ViewMap) {
    if (freeCache || !_EnableViewMapCache) {
      delete _ViewMap;
      _ViewMap = nullptr;
      prevSceneHash = -1.0;
    }
    else {
      _ViewMap->Clean();
    }
  }
}

void Controller::ComputeViewMap()
{
  if (_ListOfModels.empty()) {
    return;
  }

  DeleteViewMap(true);

  // retrieve the 3D viewpoint and transformations information
  //----------------------------------------------------------
  // Save the viewpoint context at the view level in order
  // to be able to restore it later:

  // Restore the context of view:
  // we need to perform all these operations while the
  // 3D context is on.
  Vec3f vp(UNPACK3(g_freestyle.viewpoint));

  real mv[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      mv[i][j] = g_freestyle.mv[i][j];
    }
  }

  real proj[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      proj[i][j] = g_freestyle.proj[i][j];
    }
  }

  int viewport[4];
  for (int i = 0; i < 4; i++) {
    viewport[i] = g_freestyle.viewport[i];
  }

  // Flag the WXEdge structure for silhouette edge detection:
  //----------------------------------------------------------

  if (G.debug & G_DEBUG_FREESTYLE) {
    cout << "\n===  Detecting silhouette edges  ===" << endl;
  }
  _Chrono.start();

  edgeDetector.setViewpoint(vp);
  edgeDetector.enableOrthographicProjection(proj[3][3] != 0.0);
  edgeDetector.enableRidgesAndValleysFlag(_ComputeRidges);
  edgeDetector.enableSuggestiveContours(_ComputeSuggestive);
  edgeDetector.enableMaterialBoundaries(_ComputeMaterialBoundaries);
  edgeDetector.enableFaceSmoothness(_EnableFaceSmoothness);
  edgeDetector.setCreaseAngle(_creaseAngle);
  edgeDetector.setSphereRadius(_sphereRadius);
  edgeDetector.setSuggestiveContourKrDerivativeEpsilon(_suggestiveContourKrDerivativeEpsilon);
  edgeDetector.setRenderMonitor(_pRenderMonitor);
  edgeDetector.processShapes(*_winged_edge);

  real duration = _Chrono.stop();
  if (G.debug & G_DEBUG_FREESTYLE) {
    printf("Feature lines    : %lf\n", duration);
  }

  if (_pRenderMonitor->testBreak()) {
    return;
  }

  // Builds the view map structure from the flagged WSEdge structure:
  //----------------------------------------------------------
  ViewMapBuilder vmBuilder;
  vmBuilder.setEnableQI(_EnableQI);
  vmBuilder.setViewpoint(vp);
  vmBuilder.setTransform(
      mv, proj, viewport, _pView->GetFocalLength(), _pView->GetAspect(), _pView->GetFovyRadian());
  vmBuilder.setFrustum(_pView->znear(), _pView->zfar());
  vmBuilder.setGrid(&_Grid);
  vmBuilder.setRenderMonitor(_pRenderMonitor);

  if (G.debug & G_DEBUG_FREESTYLE) {
    cout << "\n===  Building the view map  ===" << endl;
  }
  _Chrono.start();
  // Build View Map
  _ViewMap = vmBuilder.BuildViewMap(
      *_winged_edge, _VisibilityAlgo, _EPSILON, _Scene3dBBox, _SceneNumFaces);
  _ViewMap->setScene3dBBox(_Scene3dBBox);

  if (G.debug & G_DEBUG_FREESTYLE) {
    printf("ViewMap edge count : %i\n", _ViewMap->viewedges_size());
  }

  duration = _Chrono.stop();
  if (G.debug & G_DEBUG_FREESTYLE) {
    printf("ViewMap building : %lf\n", duration);
  }

  // Draw the steerable density map:
  //--------------------------------
  if (_ComputeSteerableViewMap) {
    ComputeSteerableViewMap();
  }
  // Reset Style modules modification flags
  resetModified(true);

  DeleteWingedEdge();
}

void Controller::ComputeSteerableViewMap()
{
}

void Controller::saveSteerableViewMapImages()
{
  SteerableViewMap *svm = _Canvas->getSteerableViewMap();
  if (!svm) {
    cerr << "the Steerable ViewMap has not been computed yet" << endl;
    return;
  }
  svm->saveSteerableViewMap();
}

void Controller::setVisibilityAlgo(int algo)
{
  switch (algo) {
    case FREESTYLE_ALGO_CULLED_ADAPTIVE_CUMULATIVE:
      _VisibilityAlgo = ViewMapBuilder::ray_casting_culled_adaptive_cumulative;
      break;
    case FREESTYLE_ALGO_ADAPTIVE_CUMULATIVE:
      _VisibilityAlgo = ViewMapBuilder::ray_casting_adaptive_cumulative;
      break;
  }
}

int Controller::getVisibilityAlgo()
{
  switch (_VisibilityAlgo) {
    case ViewMapBuilder::ray_casting_culled_adaptive_cumulative:
      return FREESTYLE_ALGO_CULLED_ADAPTIVE_CUMULATIVE;
    case ViewMapBuilder::ray_casting_adaptive_cumulative:
      return FREESTYLE_ALGO_ADAPTIVE_CUMULATIVE;
  }
}

void Controller::setViewMapCache(bool iBool)
{
  _EnableViewMapCache = iBool;
}

bool Controller::getViewMapCache() const
{
  return _EnableViewMapCache;
}

void Controller::setQuantitativeInvisibility(bool iBool)
{
  _EnableQI = iBool;
}

bool Controller::getQuantitativeInvisibility() const
{
  return _EnableQI;
}

void Controller::setFaceSmoothness(bool iBool)
{
  _EnableFaceSmoothness = iBool;
}

bool Controller::getFaceSmoothness() const
{
  return _EnableFaceSmoothness;
}

void Controller::setComputeRidgesAndValleysFlag(bool b)
{
  _ComputeRidges = b;
}

bool Controller::getComputeRidgesAndValleysFlag() const
{
  return _ComputeRidges;
}

void Controller::setComputeSuggestiveContoursFlag(bool b)
{
  _ComputeSuggestive = b;
}

bool Controller::getComputeSuggestiveContoursFlag() const
{
  return _ComputeSuggestive;
}

void Controller::setComputeMaterialBoundariesFlag(bool b)
{
  _ComputeMaterialBoundaries = b;
}

bool Controller::getComputeMaterialBoundariesFlag() const
{
  return _ComputeMaterialBoundaries;
}

void Controller::setComputeSteerableViewMapFlag(bool iBool)
{
  _ComputeSteerableViewMap = iBool;
}

bool Controller::getComputeSteerableViewMapFlag() const
{
  return _ComputeSteerableViewMap;
}

int Controller::DrawStrokes()
{
  if (_ViewMap == nullptr) {
    return 0;
  }

  if (G.debug & G_DEBUG_FREESTYLE) {
    cout << "\n===  Stroke drawing  ===" << endl;
  }
  _Chrono.start();
  _Canvas->Draw();
  real d = _Chrono.stop();
  int strokeCount = _Canvas->getStrokeCount();
  if (G.debug & G_DEBUG_FREESTYLE) {
    cout << "Strokes generation  : " << d << endl;
    cout << "Stroke count  : " << strokeCount << endl;
  }
  resetModified();
  DeleteViewMap();
  return strokeCount;
}

void Controller::ResetRenderCount()
{
  _render_count = 0;
}

Render *Controller::RenderStrokes(Render *re, bool render)
{
  int totmesh = 0;
  _Chrono.start();
  BlenderStrokeRenderer *blenderRenderer = new BlenderStrokeRenderer(re, ++_render_count);
  if (render) {
    _Canvas->Render(blenderRenderer);
    totmesh = blenderRenderer->GenerateScene();
  }
  real d = _Chrono.stop();
  if (G.debug & G_DEBUG_FREESTYLE) {
    cout << "Temporary scene generation: " << d << endl;
  }
  _Chrono.start();
  Render *freestyle_render = blenderRenderer->RenderScene(re, render);
  d = _Chrono.stop();
  if (G.debug & G_DEBUG_FREESTYLE) {
    cout << "Stroke rendering  : " << d << endl;

    uintptr_t mem_in_use = MEM_get_memory_in_use();
    uintptr_t peak_memory = MEM_get_peak_memory();

    float megs_used_memory = (mem_in_use) / (1024.0 * 1024.0);
    float megs_peak_memory = (peak_memory) / (1024.0 * 1024.0);

    printf("%d objs, mem %.2fM (peak %.2fM)\n", totmesh, megs_used_memory, megs_peak_memory);
  }
  delete blenderRenderer;

  return freestyle_render;
}

void Controller::InsertStyleModule(unsigned index, const char *iFileName)
{
  if (!BLI_path_extension_check(iFileName, ".py")) {
    cerr << "Error: Cannot load \"" << string(iFileName) << "\", unknown extension" << endl;
    return;
  }

  StyleModule *sm = new StyleModule(iFileName, _inter);
  _Canvas->InsertStyleModule(index, sm);
}

void Controller::InsertStyleModule(unsigned index, const char *iName, const char *iBuffer)
{
  StyleModule *sm = new BufferedStyleModule(iBuffer, iName, _inter);
  _Canvas->InsertStyleModule(index, sm);
}

void Controller::InsertStyleModule(unsigned index, const char *iName, struct Text *iText)
{
  StyleModule *sm = new BlenderStyleModule(iText, iName, _inter);
  _Canvas->InsertStyleModule(index, sm);
}

void Controller::AddStyleModule(const char * /*iFileName*/)
{
  //_pStyleWindow->Add(iFileName);
}

void Controller::RemoveStyleModule(unsigned index)
{
  _Canvas->RemoveStyleModule(index);
}

void Controller::Clear()
{
  _Canvas->Clear();
}

void Controller::ReloadStyleModule(unsigned index, const char *iFileName)
{
  StyleModule *sm = new StyleModule(iFileName, _inter);
  _Canvas->ReplaceStyleModule(index, sm);
}

void Controller::SwapStyleModules(unsigned i1, unsigned i2)
{
  _Canvas->SwapStyleModules(i1, i2);
}

void Controller::toggleLayer(unsigned index, bool iDisplay)
{
  _Canvas->setVisible(index, iDisplay);
}

void Controller::setModified(unsigned index, bool iMod)
{
  //_pStyleWindow->setModified(index, iMod);
  _Canvas->setModified(index, iMod);
  updateCausalStyleModules(index + 1);
}

void Controller::updateCausalStyleModules(unsigned index)
{
  vector<unsigned> vec;
  _Canvas->causalStyleModules(vec, index);
  for (vector<unsigned>::const_iterator it = vec.begin(); it != vec.end(); it++) {
    //_pStyleWindow->setModified(*it, true);
    _Canvas->setModified(*it, true);
  }
}

void Controller::resetModified(bool iMod)
{
  //_pStyleWindow->resetModified(iMod);
  _Canvas->resetModified(iMod);
}

NodeGroup *Controller::BuildRep(vector<ViewEdge *>::iterator vedges_begin,
                                vector<ViewEdge *>::iterator vedges_end)
{
  ViewMapTesselator2D tesselator2D;
  FrsMaterial mat;
  mat.setDiffuse(1, 1, 0.3, 1);
  tesselator2D.setFrsMaterial(mat);

  return (tesselator2D.Tesselate(vedges_begin, vedges_end));
}

void Controller::toggleEdgeTesselationNature(Nature::EdgeNature iNature)
{
  _edgeTesselationNature ^= (iNature);
  ComputeViewMap();
}

void Controller::resetInterpreter()
{
  if (_inter) {
    _inter->reset();
  }
}

void Controller::displayDensityCurves(int x, int y)
{
  SteerableViewMap *svm = _Canvas->getSteerableViewMap();
  if (!svm) {
    return;
  }

  unsigned int i, j;
  using densityCurve = vector<Vec3r>;
  vector<densityCurve> curves(svm->getNumberOfOrientations() + 1);
  vector<densityCurve> curvesDirection(svm->getNumberOfPyramidLevels());

  // collect the curves values
  unsigned nbCurves = svm->getNumberOfOrientations() + 1;
  unsigned nbPoints = svm->getNumberOfPyramidLevels();
  if (!nbPoints) {
    return;
  }

  // build the density/nbLevels curves for each orientation
  for (i = 0; i < nbCurves; ++i) {
    for (j = 0; j < nbPoints; ++j) {
      curves[i].push_back(Vec3r(j, svm->readSteerableViewMapPixel(i, j, x, y), 0));
    }
  }
  // build the density/nbOrientations curves for each level
  for (i = 0; i < nbPoints; ++i) {
    for (j = 0; j < nbCurves; ++j) {
      curvesDirection[i].push_back(Vec3r(j, svm->readSteerableViewMapPixel(j, i, x, y), 0));
    }
  }

}

void Controller::init_options()
{
  // from AppOptionsWindow.cpp
  // Default init options

  Config::Path *cpath = Config::Path::getInstance();

  // Directories
  TextureManager::Options::setPatternsPath(cpath->getPatternsPath());
  TextureManager::Options::setBrushesPath(cpath->getModelsPath());

  // ViewMap Format
  setComputeSteerableViewMapFlag(false);

  // Visibility
  setQuantitativeInvisibility(true);

  // soc: initialize canvas
  _Canvas->init();

  // soc: initialize passes
  setPassDiffuse(nullptr, 0, 0);
  setPassZ(nullptr, 0, 0);
}

} /* namespace Freestyle */
