/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup freestyle
 * \brief Class to build silhouette edges from a Winged-Edge structure
 */

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "FRS_freestyle.h"

#include "BoxGrid.h"
#include "CulledOccluderSource.h"
#include "HeuristicGridDensityProviderFactory.h"
#include "OccluderSource.h"
#include "SphericalGrid.h"
#include "ViewMapBuilder.h"

#include "../geometry/GeomUtils.h"
#include "../geometry/GridHelpers.h"

#include "../winged_edge/WFillGrid.h"

#include "BKE_global.h"

namespace Freestyle {

// XXX Grmll... G is used as template's typename parameter :/
static const Global &_global = G;

using namespace std;

template<typename G, typename I>
static void findOccludee(FEdge *fe,
                         G & /*grid*/,
                         I &occluders,
                         real epsilon,
                         WFace **oaWFace,
                         Vec3r &u,
                         Vec3r &A,
                         Vec3r &origin,
                         Vec3r &edgeDir,
                         vector<WVertex *> &faceVertices)
{
  WFace *face = nullptr;
  if (fe->isSmooth()) {
    FEdgeSmooth *fes = dynamic_cast<FEdgeSmooth *>(fe);
    face = (WFace *)fes->face();
  }
  WFace *oface;
  bool skipFace;

  WVertex::incoming_edge_iterator ie;

  *oaWFace = nullptr;
  if (((fe)->getNature() & Nature::SILHOUETTE) || ((fe)->getNature() & Nature::BORDER)) {
    // we cast a ray from A in the same direction but looking behind
    Vec3r v(-u[0], -u[1], -u[2]);
    bool noIntersection = true;
    real mint = FLT_MAX;

    for (occluders.initAfterTarget(); occluders.validAfterTarget(); occluders.nextOccludee()) {
      oface = occluders.getWFace();
      Polygon3r *p = occluders.getCameraSpacePolygon();
      real d = -((p->getVertices())[0] * p->getNormal());
      real t, t_u, t_v;

      if (nullptr != face) {
        skipFace = false;

        if (face == oface) {
          continue;
        }

        if (faceVertices.empty()) {
          continue;
        }

        for (vector<WVertex *>::iterator fv = faceVertices.begin(), fvend = faceVertices.end();
             fv != fvend;
             ++fv) {
          if ((*fv)->isBoundary()) {
            continue;
          }
          WVertex::incoming_edge_iterator iebegin = (*fv)->incoming_edges_begin();
          WVertex::incoming_edge_iterator ieend = (*fv)->incoming_edges_end();
          for (ie = iebegin; ie != ieend; ++ie) {
            if ((*ie) == nullptr) {
              continue;
            }

            WFace *sface = (*ie)->GetbFace();
            if (sface == oface) {
              skipFace = true;
              break;
            }
          }
          if (skipFace) {
            break;
          }
        }
        if (skipFace) {
          continue;
        }
      }
      else {
        // check whether the edge and the polygon plane are coincident:
        //-------------------------------------------------------------
        // first let us compute the plane equation.
        if (GeomUtils::COINCIDENT ==
            GeomUtils::intersectRayPlane(origin, edgeDir, p->getNormal(), d, t, epsilon)) {
          continue;
        }
      }

      if (p->rayIntersect(A, v, t, t_u, t_v)) {
        if (fabs(v * p->getNormal()) > 0.0001) {
          if ((t > 0.0) /* && (t<1.0) */) {
            if (t < mint) {
              *oaWFace = oface;
              mint = t;
              noIntersection = false;
              fe->setOccludeeIntersection(Vec3r(A + t * v));
            }
          }
        }

        occluders.reportDepth(A, v, t);
      }
    }

    if (noIntersection) {
      *oaWFace = nullptr;
    }
  }
}

template<typename G, typename I>
static void findOccludee(FEdge *fe, G &grid, real epsilon, ViewEdge * /*ve*/, WFace **oaFace)
{
  Vec3r A;
  Vec3r edgeDir;
  Vec3r origin;
  A = Vec3r(((fe)->vertexA()->point3D() + (fe)->vertexB()->point3D()) / 2.0);
  edgeDir = Vec3r((fe)->vertexB()->point3D() - (fe)->vertexA()->point3D());
  edgeDir.normalize();
  origin = Vec3r((fe)->vertexA()->point3D());
  Vec3r u;
  if (grid.orthographicProjection()) {
    u = Vec3r(0.0, 0.0, grid.viewpoint().z() - A.z());
  }
  else {
    u = Vec3r(grid.viewpoint() - A);
  }
  u.normalize();

  vector<WVertex *> faceVertices;

  WFace *face = nullptr;
  if (fe->isSmooth()) {
    FEdgeSmooth *fes = dynamic_cast<FEdgeSmooth *>(fe);
    face = (WFace *)fes->face();
  }

  if (face) {
    face->RetrieveVertexList(faceVertices);
  }

  I occluders(grid, A, epsilon);
  findOccludee<G, I>(fe, grid, occluders, epsilon, oaFace, u, A, origin, edgeDir, faceVertices);
}

// computeVisibility takes a pointer to foundOccluders, instead of using a reference,
// so that computeVeryFastVisibility can skip the AddOccluders step with minimal overhead.
template<typename G, typename I>
static int computeVisibility(ViewMap *viewMap,
                             FEdge *fe,
                             G &grid,
                             real epsilon,
                             ViewEdge * /*ve*/,
                             WFace **oaWFace,
                             set<ViewShape *> *foundOccluders)
{
  int qi = 0;

  Vec3r center;
  Vec3r edgeDir;
  Vec3r origin;

  center = fe->center3d();
  edgeDir = Vec3r(fe->vertexB()->point3D() - fe->vertexA()->point3D());
  edgeDir.normalize();
  origin = Vec3r(fe->vertexA()->point3D());

  Vec3r vp;
  if (grid.orthographicProjection()) {
    vp = Vec3r(center.x(), center.y(), grid.viewpoint().z());
  }
  else {
    vp = Vec3r(grid.viewpoint());
  }
  Vec3r u(vp - center);
  real raylength = u.norm();
  u.normalize();

  WFace *face = nullptr;
  if (fe->isSmooth()) {
    FEdgeSmooth *fes = dynamic_cast<FEdgeSmooth *>(fe);
    face = (WFace *)fes->face();
  }
  vector<WVertex *> faceVertices;
  WVertex::incoming_edge_iterator ie;

  WFace *oface;
  bool skipFace;

  if (face) {
    face->RetrieveVertexList(faceVertices);
  }

  I occluders(grid, center, epsilon);

  for (occluders.initBeforeTarget(); occluders.validBeforeTarget(); occluders.nextOccluder()) {
    // If we're dealing with an exact silhouette, check whether we must take care of this occluder
    // of not. (Indeed, we don't consider the occluders that share at least one vertex with the
    // face containing this edge).
    //-----------
    oface = occluders.getWFace();
    Polygon3r *p = occluders.getCameraSpacePolygon();
    real t, t_u, t_v;

    real d = -((p->getVertices())[0] * p->getNormal());

    if (face) {
      skipFace = false;

      if (face == oface) {
        continue;
      }

      for (vector<WVertex *>::iterator fv = faceVertices.begin(), fvend = faceVertices.end();
           fv != fvend;
           ++fv) {
        if ((*fv)->isBoundary()) {
          continue;
        }

        WVertex::incoming_edge_iterator iebegin = (*fv)->incoming_edges_begin();
        WVertex::incoming_edge_iterator ieend = (*fv)->incoming_edges_end();
        for (ie = iebegin; ie != ieend; ++ie) {
          if ((*ie) == nullptr) {
            continue;
          }

          WFace *sface = (*ie)->GetbFace();
          // WFace *sfacea = (*ie)->GetaFace();
          // if ((sface == oface) || (sfacea == oface))
          if (sface == oface) {
            skipFace = true;
            break;
          }
        }
        if (skipFace) {
          break;
        }
      }
      if (skipFace) {
        continue;
      }
    }
    else {
      // check whether the edge and the polygon plane are coincident:
      //-------------------------------------------------------------
      // first let us compute the plane equation.
      if (GeomUtils::COINCIDENT ==
          GeomUtils::intersectRayPlane(origin, edgeDir, p->getNormal(), d, t, epsilon)) {
        continue;
      }
    }

    if (p->rayIntersect(center, u, t, t_u, t_v)) {
      if (fabs(u * p->getNormal()) > 0.0001) {
        if ((t > 0.0) && (t < raylength)) {
          if (foundOccluders != nullptr) {
            ViewShape *vshape = viewMap->viewShape(oface->GetVertex(0)->shape()->GetId());
            foundOccluders->insert(vshape);
          }
          ++qi;

          if (!grid.enableQI()) {
            break;
          }
        }

        occluders.reportDepth(center, u, t);
      }
    }
  }

  // Find occludee
  findOccludee<G, I>(
      fe, grid, occluders, epsilon, oaWFace, u, center, origin, edgeDir, faceVertices);

  return qi;
}

// computeCumulativeVisibility returns the lowest x such that the majority of FEdges have QI <= x
//
// This was probably the original intention of the "normal" algorithm on which
// computeDetailedVisibility is based. But because the "normal" algorithm chooses the most popular
// QI, without considering any other values, a ViewEdge with FEdges having QIs of 0, 21, 22, 23, 24
// and 25 will end up having a total QI of 0, even though most of the FEdges are heavily occluded.
// computeCumulativeVisibility will treat this case as a QI of 22 because 3 out of 6 occluders have
// QI <= 22.

template<typename G, typename I>
static void computeCumulativeVisibility(ViewMap *ioViewMap,
                                        G &grid,
                                        real epsilon,
                                        RenderMonitor *iRenderMonitor)
{
  vector<ViewEdge *> &vedges = ioViewMap->ViewEdges();

  FEdge *fe, *festart;
  int nSamples = 0;
  vector<WFace *> wFaces;
  WFace *wFace = nullptr;
  unsigned count = 0;
  unsigned count_step = (unsigned)ceil(0.01f * vedges.size());
  unsigned tmpQI = 0;
  unsigned qiClasses[256];
  unsigned maxIndex, maxCard;
  unsigned qiMajority;
  for (vector<ViewEdge *>::iterator ve = vedges.begin(), veend = vedges.end(); ve != veend; ve++) {
    if (iRenderMonitor) {
      if (iRenderMonitor->testBreak()) {
        break;
      }
      if (count % count_step == 0) {
        stringstream ss;
        ss << "Freestyle: Visibility computations " << (100 * count / vedges.size()) << "%";
        iRenderMonitor->setInfo(ss.str());
        iRenderMonitor->progress((float)count / vedges.size());
      }
      count++;
    }
    // Find an edge to test
    if (!(*ve)->isInImage()) {
      // This view edge has been proscenium culled
      (*ve)->setQI(255);
      (*ve)->setaShape(nullptr);
      continue;
    }

    // Test edge
    festart = (*ve)->fedgeA();
    fe = (*ve)->fedgeA();
    qiMajority = 0;
    do {
      if (fe != nullptr && fe->isInImage()) {
        qiMajority++;
      }
      fe = fe->nextEdge();
    } while (fe && fe != festart);

    if (qiMajority == 0) {
      // There are no occludable FEdges on this ViewEdge
      // This should be impossible.
      if (_global.debug & G_DEBUG_FREESTYLE) {
        cout << "View Edge in viewport without occludable FEdges: " << (*ve)->getId() << endl;
      }
      // We can recover from this error:
      // Treat this edge as fully visible with no occludee
      (*ve)->setQI(0);
      (*ve)->setaShape(nullptr);
      continue;
    }

    ++qiMajority;
    qiMajority >>= 1;

    tmpQI = 0;
    maxIndex = 0;
    maxCard = 0;
    nSamples = 0;
    memset(qiClasses, 0, 256 * sizeof(*qiClasses));
    set<ViewShape *> foundOccluders;

    fe = (*ve)->fedgeA();
    do {
      if (!fe || !fe->isInImage()) {
        fe = fe->nextEdge();
        continue;
      }
      if (maxCard < qiMajority) {
        // ARB: change &wFace to wFace and use reference in called function
        tmpQI = computeVisibility<G, I>(
            ioViewMap, fe, grid, epsilon, *ve, &wFace, &foundOccluders);

        // ARB: This is an error condition, not an alert condition.
        // Some sort of recovery or abort is necessary.
        if (tmpQI >= 256) {
          cerr << "Warning: too many occluding levels" << endl;
          // ARB: Wild guess: instead of aborting or corrupting memory, treat as tmpQI == 255
          tmpQI = 255;
        }

        if (++qiClasses[tmpQI] > maxCard) {
          maxCard = qiClasses[tmpQI];
          maxIndex = tmpQI;
        }
      }
      else {
        // ARB: FindOccludee is redundant if ComputeRayCastingVisibility has been called
        // ARB: change &wFace to wFace and use reference in called function
        findOccludee<G, I>(fe, grid, epsilon, *ve, &wFace);
      }

      // Store test results
      if (wFace) {
        vector<Vec3r> vertices;
        for (int i = 0, numEdges = wFace->numberOfEdges(); i < numEdges; ++i) {
          vertices.emplace_back(wFace->GetVertex(i)->GetVertex());
        }
        Polygon3r poly(vertices, wFace->GetNormal());
        poly.userdata = (void *)wFace;
        fe->setaFace(poly);
        wFaces.push_back(wFace);
        fe->setOccludeeEmpty(false);
      }
      else {
        fe->setOccludeeEmpty(true);
      }

      ++nSamples;
      fe = fe->nextEdge();
    } while ((maxCard < qiMajority) && (fe) && (fe != festart));

    // ViewEdge
    // qi --
    // Find the minimum value that is >= the majority of the QI
    for (unsigned count = 0, i = 0; i < 256; ++i) {
      count += qiClasses[i];
      if (count >= qiMajority) {
        (*ve)->setQI(i);
        break;
      }
    }
    // occluders --
    // I would rather not have to go through the effort of creating this set and then copying out
    // its contents. Is there a reason why ViewEdge::_Occluders cannot be converted to a set<>?
    for (set<ViewShape *>::iterator o = foundOccluders.begin(), oend = foundOccluders.end();
         o != oend;
         ++o) {
      (*ve)->AddOccluder((*o));
    }
    (void)maxIndex;
    // occludee --
    if (!wFaces.empty()) {
      if (wFaces.size() <= (float)nSamples / 2.0f) {
        (*ve)->setaShape(nullptr);
      }
      else {
        ViewShape *vshape = ioViewMap->viewShape(
            (*wFaces.begin())->GetVertex(0)->shape()->GetId());
        (*ve)->setaShape(vshape);
      }
    }

    wFaces.clear();
  }
  if (iRenderMonitor && !vedges.empty()) {
    stringstream ss;
    ss << "Freestyle: Visibility computations " << (100 * count / vedges.size()) << "%";
    iRenderMonitor->setInfo(ss.str());
    iRenderMonitor->progress((float)count / vedges.size());
  }
}

ViewMap *ViewMapBuilder::BuildViewMap(WingedEdge &we,
                                      visibility_algo iAlgo,
                                      real epsilon,
                                      const BBox<Vec3r> &bbox,
                                      unsigned int sceneNumFaces)
{
  _ViewMap = new ViewMap;
  _currentId = 1;
  _currentFId = 0;
  _currentSVertexId = 0;

  // Builds initial view edges
  computeInitialViewEdges(we);

  // Detects cusps
  computeCusps(_ViewMap);

  // Compute intersections
  ComputeIntersections(_ViewMap, sweep_line, epsilon);

  // Compute visibility
  ComputeEdgesVisibility(_ViewMap, we, bbox, sceneNumFaces, iAlgo, epsilon);

  return _ViewMap;
}

static inline real distance2D(const Vec3r &point, const real origin[2])
{
  return ::hypot((point[0] - origin[0]), (point[1] - origin[1]));
}

static inline bool crossesProscenium(real proscenium[4], FEdge *fe)
{
  Vec2r min(proscenium[0], proscenium[2]);
  Vec2r max(proscenium[1], proscenium[3]);
  Vec2r A(fe->vertexA()->getProjectedX(), fe->vertexA()->getProjectedY());
  Vec2r B(fe->vertexB()->getProjectedX(), fe->vertexB()->getProjectedY());

  return GeomUtils::intersect2dSeg2dArea(min, max, A, B);
}

static inline bool insideProscenium(const real proscenium[4], const Vec3r &point)
{
  return !(point[0] < proscenium[0] || point[0] > proscenium[1] || point[1] < proscenium[2] ||
           point[1] > proscenium[3]);
}

void ViewMapBuilder::computeInitialViewEdges(WingedEdge &we)
{
  vector<WShape *> wshapes = we.getWShapes();
  SShape *psShape;

  for (vector<WShape *>::const_iterator it = wshapes.begin(); it != wshapes.end(); it++) {
    if (_pRenderMonitor && _pRenderMonitor->testBreak()) {
      break;
    }

    // create the embedding
    psShape = new SShape;
    psShape->setId((*it)->GetId());
    psShape->setName((*it)->getName());
    psShape->setLibraryPath((*it)->getLibraryPath());
    psShape->setFrsMaterials((*it)->frs_materials());  // FIXME

    // create the view shape
    ViewShape *vshape = new ViewShape(psShape);
    // add this view shape to the view map:
    _ViewMap->AddViewShape(vshape);

    // we want to number the view edges in a unique way for the while scene.
    _pViewEdgeBuilder->setCurrentViewId(_currentId);
    // we want to number the feature edges in a unique way for the while scene.
    _pViewEdgeBuilder->setCurrentFId(_currentFId);
    // we want to number the SVertex in a unique way for the while scene.
    _pViewEdgeBuilder->setCurrentSVertexId(_currentFId);
    _pViewEdgeBuilder->BuildViewEdges(dynamic_cast<WXShape *>(*it),
                                      vshape,
                                      _ViewMap->ViewEdges(),
                                      _ViewMap->ViewVertices(),
                                      _ViewMap->FEdges(),
                                      _ViewMap->SVertices());

    _currentId = _pViewEdgeBuilder->currentViewId() + 1;
    _currentFId = _pViewEdgeBuilder->currentFId() + 1;
    _currentSVertexId = _pViewEdgeBuilder->currentSVertexId() + 1;

    psShape->ComputeBBox();
  }
}

void ViewMapBuilder::computeCusps(ViewMap *ioViewMap)
{
  vector<ViewEdge *> newVEdges;
  ViewMap::viewedges_container &vedges = ioViewMap->ViewEdges();
  ViewMap::viewedges_container::iterator ve = vedges.begin(), veend = vedges.end();
  for (; ve != veend; ++ve) {
    if (_pRenderMonitor && _pRenderMonitor->testBreak()) {
      break;
    }
    if ((!((*ve)->getNature() & Nature::SILHOUETTE)) || (!((*ve)->fedgeA()->isSmooth()))) {
      continue;
    }
    FEdge *fe = (*ve)->fedgeA();
    FEdge *fefirst = fe;
    bool first = true;
    bool positive = true;
    do {
      FEdgeSmooth *fes = dynamic_cast<FEdgeSmooth *>(fe);
      Vec3r A((fes)->vertexA()->point3d());
      Vec3r B((fes)->vertexB()->point3d());
      Vec3r AB(B - A);
      AB.normalize();
      Vec3r m((A + B) / 2.0);
      Vec3r crossP(AB ^ (fes)->normal());
      crossP.normalize();
      Vec3r viewvector;
      if (_orthographicProjection) {
        viewvector = Vec3r(0.0, 0.0, m.z() - _viewpoint.z());
      }
      else {
        viewvector = Vec3r(m - _viewpoint);
      }
      viewvector.normalize();
      if (first) {
        if (((crossP) * (viewvector)) > 0) {
          positive = true;
        }
        else {
          positive = false;
        }
        first = false;
      }
      // If we're in a positive part, we need a stronger negative value to change
      NonTVertex *cusp = nullptr;
      if (positive) {
        if (((crossP) * (viewvector)) < -0.1) {
          // state changes
          positive = false;
          // creates and insert cusp
          cusp = dynamic_cast<NonTVertex *>(
              ioViewMap->InsertViewVertex(fes->vertexA(), newVEdges));
          if (cusp) {
            cusp->setNature(cusp->getNature() | Nature::CUSP);
          }
        }
      }
      else {
        // If we're in a negative part, we need a stronger negative value to change
        if (((crossP) * (viewvector)) > 0.1) {
          positive = true;
          cusp = dynamic_cast<NonTVertex *>(
              ioViewMap->InsertViewVertex(fes->vertexA(), newVEdges));
          if (cusp) {
            cusp->setNature(cusp->getNature() | Nature::CUSP);
          }
        }
      }
      fe = fe->nextEdge();
    } while (fe && fe != fefirst);
  }
  for (ve = newVEdges.begin(), veend = newVEdges.end(); ve != veend; ++ve) {
    (*ve)->viewShape()->AddEdge(*ve);
    vedges.push_back(*ve);
  }
}

void ViewMapBuilder::ComputeCumulativeVisibility(ViewMap *ioViewMap,
                                                 WingedEdge &we,
                                                 const BBox<Vec3r> &bbox,
                                                 real epsilon,
                                                 bool cull,
                                                 GridDensityProviderFactory &factory)
{
  AutoPtr<GridHelpers::Transform> transform;
  AutoPtr<OccluderSource> source;

  if (_orthographicProjection) {
    transform = std::make_unique<BoxGrid::Transform>();
  }
  else {
    transform = std::make_unique<SphericalGrid::Transform>();
  }

  if (cull) {
    source = std::make_unique<CulledOccluderSource>(*transform, we, *ioViewMap, true);
  }
  else {
    source = std::make_unique<OccluderSource>(*transform, we);
  }

  AutoPtr<GridDensityProvider> density(factory.newGridDensityProvider(*source, bbox, *transform));

  if (_orthographicProjection) {
    BoxGrid grid(*source, *density, ioViewMap, _viewpoint, _EnableQI);
    computeCumulativeVisibility<BoxGrid, BoxGrid::Iterator>(
        ioViewMap, grid, epsilon, _pRenderMonitor);
  }
  else {
    SphericalGrid grid(*source, *density, ioViewMap, _viewpoint, _EnableQI);
    computeCumulativeVisibility<SphericalGrid, SphericalGrid::Iterator>(
        ioViewMap, grid, epsilon, _pRenderMonitor);
  }
}

void ViewMapBuilder::ComputeEdgesVisibility(ViewMap *ioViewMap,
                                            WingedEdge &we,
                                            const BBox<Vec3r> &bbox,
                                            unsigned int sceneNumFaces,
                                            visibility_algo iAlgo,
                                            real epsilon)
{
  switch (iAlgo) {
    case ray_casting_culled_adaptive_cumulative:
      if (_global.debug & G_DEBUG_FREESTYLE) {
        cout << "Using culled adaptive grid with heuristic density and cumulative QI calculation"
             << endl;
      }
      try {
        HeuristicGridDensityProviderFactory factory(0.5f, sceneNumFaces);
        ComputeCumulativeVisibility(ioViewMap, we, bbox, epsilon, true, factory);
      }
      catch (...) {
        throw;
      }
      break;
    case ray_casting_adaptive_cumulative:
      if (_global.debug & G_DEBUG_FREESTYLE) {
        cout << "Using unculled adaptive grid with heuristic density and cumulative QI calculation"
             << endl;
      }
      try {
        HeuristicGridDensityProviderFactory factory(0.5f, sceneNumFaces);
        ComputeCumulativeVisibility(ioViewMap, we, bbox, epsilon, false, factory);
      }
      catch (...) {
        throw;
      }
      break;
    default:
      break;
  }
}

static const unsigned gProgressBarMaxSteps = 10;
static const unsigned gProgressBarMinSize = 2000;

void ViewMapBuilder::ComputeIntersections(ViewMap *ioViewMap,
                                          intersection_algo iAlgo,
                                          real epsilon)
{
  switch (iAlgo) {
    case sweep_line:
      ComputeSweepLineIntersections(ioViewMap, epsilon);
      break;
    default:
      break;
  }
}

struct less_SVertex2D {
  real epsilon;

  less_SVertex2D(real eps)
  {
    epsilon = eps;
  }

  bool operator()(SVertex *x, SVertex *y)
  {
    Vec3r A = x->point2D();
    Vec3r B = y->point2D();
    for (unsigned int i = 0; i < 3; i++) {
      if (fabs(A[i] - B[i]) < epsilon) {
        continue;
      }
      if (A[i] < B[i]) {
        return true;
      }
      if (A[i] > B[i]) {
        return false;
      }
    }
    return false;
  }
};

using segment = Segment<FEdge *, Vec3r>;
using intersection = Intersection<segment>;

struct less_Intersection {
  segment *edge;

  less_Intersection(segment *iEdge)
  {
    edge = iEdge;
  }

  bool operator()(intersection *x, intersection *y)
  {
    real tx = x->getParameter(edge);
    real ty = y->getParameter(edge);
    if (tx > ty) {
      return true;
    }
    return false;
  }
};

struct silhouette_binary_rule : public binary_rule<segment, segment> {
  bool operator()(segment &s1, segment &s2) override
  {
    FEdge *f1 = s1.edge();
    FEdge *f2 = s2.edge();

    if ((!(((f1)->getNature() & Nature::SILHOUETTE) || ((f1)->getNature() & Nature::BORDER))) &&
        (!(((f2)->getNature() & Nature::SILHOUETTE) || ((f2)->getNature() & Nature::BORDER)))) {
      return false;
    }

    return true;
  }
};

void ViewMapBuilder::ComputeSweepLineIntersections(ViewMap *ioViewMap, real epsilon)
{
  vector<SVertex *> &svertices = ioViewMap->SVertices();
  bool progressBarDisplay = false;
  unsigned sVerticesSize = svertices.size();
  unsigned fEdgesSize = ioViewMap->FEdges().size();

  unsigned progressBarStep = 0;

  if (_pProgressBar != nullptr && fEdgesSize > gProgressBarMinSize) {
    unsigned progressBarSteps = min(gProgressBarMaxSteps, sVerticesSize);
    progressBarStep = sVerticesSize / progressBarSteps;
    _pProgressBar->reset();
    _pProgressBar->setLabelText("Computing Sweep Line Intersections");
    _pProgressBar->setTotalSteps(progressBarSteps);
    _pProgressBar->setProgress(0);
    progressBarDisplay = true;
  }

  unsigned counter = progressBarStep;

  sort(svertices.begin(), svertices.end(), less_SVertex2D(epsilon));

  SweepLine<FEdge *, Vec3r> SL;

  vector<FEdge *> &ioEdges = ioViewMap->FEdges();

  vector<segment *> segments;

  vector<FEdge *>::iterator fe, fend;

  for (fe = ioEdges.begin(), fend = ioEdges.end(); fe != fend; fe++) {
    segment *s = new segment((*fe), (*fe)->vertexA()->point2D(), (*fe)->vertexB()->point2D());
    (*fe)->userdata = s;
    segments.push_back(s);
  }

  vector<segment *> vsegments;
  for (vector<SVertex *>::iterator sv = svertices.begin(), svend = svertices.end(); sv != svend;
       sv++) {
    if (_pRenderMonitor && _pRenderMonitor->testBreak()) {
      break;
    }

    const vector<FEdge *> &vedges = (*sv)->fedges();

    for (vector<FEdge *>::const_iterator sve = vedges.begin(), sveend = vedges.end();
         sve != sveend;
         sve++) {
      vsegments.push_back((segment *)((*sve)->userdata));
    }

    Vec3r evt((*sv)->point2D());
    silhouette_binary_rule sbr;
    SL.process(evt, vsegments, sbr, epsilon);

    if (progressBarDisplay) {
      counter--;
      if (counter <= 0) {
        counter = progressBarStep;
        _pProgressBar->setProgress(_pProgressBar->getProgress() + 1);
      }
    }
    vsegments.clear();
  }

  if (_pRenderMonitor && _pRenderMonitor->testBreak()) {
    // delete segments
    if (!segments.empty()) {
      vector<segment *>::iterator s, send;
      for (s = segments.begin(), send = segments.end(); s != send; s++) {
        delete *s;
      }
    }
    return;
  }

  // reset userdata:
  for (fe = ioEdges.begin(), fend = ioEdges.end(); fe != fend; fe++) {
    (*fe)->userdata = nullptr;
  }

  // list containing the new edges resulting from splitting operations.
  vector<FEdge *> newEdges;

  // retrieve the intersected edges:
  vector<segment *> &iedges = SL.intersectedEdges();
  // retrieve the intersections:
  vector<intersection *> &intersections = SL.intersections();

  int id = 0;
  // create a view vertex for each intersection and linked this one with the intersection object
  vector<intersection *>::iterator i, iend;
  for (i = intersections.begin(), iend = intersections.end(); i != iend; i++) {
    FEdge *fA = (*i)->EdgeA->edge();
    FEdge *fB = (*i)->EdgeB->edge();

    Vec3r A1 = fA->vertexA()->point3D();
    Vec3r A2 = fA->vertexB()->point3D();
    Vec3r B1 = fB->vertexA()->point3D();
    Vec3r B2 = fB->vertexB()->point3D();

    Vec3r a1 = fA->vertexA()->point2D();
    Vec3r a2 = fA->vertexB()->point2D();
    Vec3r b1 = fB->vertexA()->point2D();
    Vec3r b2 = fB->vertexB()->point2D();

    real ta = (*i)->tA;
    real tb = (*i)->tB;

    if ((ta < -epsilon) || (ta > 1 + epsilon)) {
      cerr << "Warning: 2D intersection out of range for edge " << fA->vertexA()->getId() << " - "
           << fA->vertexB()->getId() << endl;
    }

    if ((tb < -epsilon) || (tb > 1 + epsilon)) {
      cerr << "Warning: 2D intersection out of range for edge " << fB->vertexA()->getId() << " - "
           << fB->vertexB()->getId() << endl;
    }

    real Ta = SilhouetteGeomEngine::ImageToWorldParameter(fA, ta);
    real Tb = SilhouetteGeomEngine::ImageToWorldParameter(fB, tb);

    if ((Ta < -epsilon) || (Ta > 1 + epsilon)) {
      cerr << "Warning: 3D intersection out of range for edge " << fA->vertexA()->getId() << " - "
           << fA->vertexB()->getId() << endl;
    }

    if ((Tb < -epsilon) || (Tb > 1 + epsilon)) {
      cerr << "Warning: 3D intersection out of range for edge " << fB->vertexA()->getId() << " - "
           << fB->vertexB()->getId() << endl;
    }

    TVertex *tvertex = ioViewMap->CreateTVertex(Vec3r(A1 + Ta * (A2 - A1)),
                                                Vec3r(a1 + ta * (a2 - a1)),
                                                fA,
                                                Vec3r(B1 + Tb * (B2 - B1)),
                                                Vec3r(b1 + tb * (b2 - b1)),
                                                fB,
                                                id);

    (*i)->userdata = tvertex;
    ++id;
  }

  progressBarStep = 0;

  if (progressBarDisplay) {
    unsigned iEdgesSize = iedges.size();
    unsigned progressBarSteps = min(gProgressBarMaxSteps, iEdgesSize);
    progressBarStep = iEdgesSize / progressBarSteps;
    _pProgressBar->reset();
    _pProgressBar->setLabelText("Splitting intersected edges");
    _pProgressBar->setTotalSteps(progressBarSteps);
    _pProgressBar->setProgress(0);
  }

  counter = progressBarStep;

  vector<TVertex *> edgeVVertices;
  vector<ViewEdge *> newVEdges;
  vector<segment *>::iterator s, send;
  for (s = iedges.begin(), send = iedges.end(); s != send; s++) {
    edgeVVertices.clear();
    newEdges.clear();
    newVEdges.clear();

    FEdge *fedge = (*s)->edge();
    ViewEdge *vEdge = fedge->viewedge();
    ViewShape *shape = vEdge->viewShape();

    vector<intersection *> &eIntersections = (*s)->intersections();
    // we first need to sort these intersections from farther to closer to A
    sort(eIntersections.begin(), eIntersections.end(), less_Intersection(*s));
    for (i = eIntersections.begin(), iend = eIntersections.end(); i != iend; i++) {
      edgeVVertices.push_back((TVertex *)(*i)->userdata);
    }

    shape->SplitEdge(fedge, edgeVVertices, ioViewMap->FEdges(), ioViewMap->ViewEdges());

    if (progressBarDisplay) {
      counter--;
      if (counter <= 0) {
        counter = progressBarStep;
        _pProgressBar->setProgress(_pProgressBar->getProgress() + 1);
      }
    }
  }

  // reset userdata:
  for (fe = ioEdges.begin(), fend = ioEdges.end(); fe != fend; fe++) {
    (*fe)->userdata = nullptr;
  }

  // delete segments
  if (!segments.empty()) {
    for (s = segments.begin(), send = segments.end(); s != send; s++) {
      delete *s;
    }
  }
}

} /* namespace Freestyle */
