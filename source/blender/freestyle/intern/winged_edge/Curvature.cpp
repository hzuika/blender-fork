/* SPDX-License-Identifier: GPL-2.0-or-later
 * The Original Code is:
 *     GTS - Library for the manipulation of triangulated surfaces
 *     Copyright 1999 Stephane Popinet
 * and:
 *     OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *     Copyright 2000-2003 Bruno Levy <levy@loria.fr> */

/** \file
 * \ingroup freestyle
 * \brief GTS - Library for the manipulation of triangulated surfaces
 * \brief OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 */

#include <cassert>
#include <cstdlib>  // for malloc and free
#include <set>
#include <stack>

#include "Curvature.h"
#include "WEdge.h"

#include "../geometry/normal_cycle.h"

#include "BLI_math.h"

namespace Freestyle {

namespace OGF {

// precondition1: P is inside the sphere
// precondition2: P,V points to the outside of the sphere (i.e. OP.V > 0)
static bool sphere_clip_vector(const Vec3r &O, real r, const Vec3r &P, Vec3r &V)
{
  Vec3r W = P - O;
  real a = V.squareNorm();
  real b = 2.0 * V * W;
  real c = W.squareNorm() - r * r;
  real delta = b * b - 4 * a * c;
  if (delta < 0) {
    // Should not happen, but happens sometimes (numerical precision)
    return true;
  }
  real t = -b + ::sqrt(delta) / (2.0 * a);
  if (t < 0.0) {
    // Should not happen, but happens sometimes (numerical precision)
    return true;
  }
  if (t >= 1.0) {
    // Inside the sphere
    return false;
  }

  V[0] = (t * V.x());
  V[1] = (t * V.y());
  V[2] = (t * V.z());

  return true;
}

// TODO: check optimizations:
// use marking ? (measure *timings* ...)
void compute_curvature_tensor(WVertex *start, real radius, NormalCycle &nc)
{
  // in case we have a non-manifold vertex, skip it...
  if (start->isBoundary()) {
    return;
  }

  std::set<WVertex *> vertices;
  const Vec3r &O = start->GetVertex();
  std::stack<WVertex *> S;
  S.push(start);
  vertices.insert(start);
  while (!S.empty()) {
    WVertex *v = S.top();
    S.pop();
    if (v->isBoundary()) {
      continue;
    }
    const Vec3r &P = v->GetVertex();
    WVertex::incoming_edge_iterator woeit = v->incoming_edges_begin();
    WVertex::incoming_edge_iterator woeitend = v->incoming_edges_end();
    for (; woeit != woeitend; ++woeit) {
      WOEdge *h = *woeit;
      if ((v == start) || h->GetVec() * (O - P) > 0.0) {
        Vec3r V(-1 * h->GetVec());
        bool isect = sphere_clip_vector(O, radius, P, V);
        assert(h->GetOwner()->GetNumberOfOEdges() ==
               2);  // Because otherwise v->isBoundary() would be true
        nc.accumulate_dihedral_angle(V, h->GetAngle());

        if (!isect) {
          WVertex *w = h->GetaVertex();
          if (vertices.find(w) == vertices.end()) {
            vertices.insert(w);
            S.push(w);
          }
        }
      }
    }
  }
}

void compute_curvature_tensor_one_ring(WVertex *start, NormalCycle &nc)
{
  // in case we have a non-manifold vertex, skip it...
  if (start->isBoundary()) {
    return;
  }

  WVertex::incoming_edge_iterator woeit = start->incoming_edges_begin();
  WVertex::incoming_edge_iterator woeitend = start->incoming_edges_end();
  for (; woeit != woeitend; ++woeit) {
    WOEdge *h = (*woeit)->twin();
    nc.accumulate_dihedral_angle(h->GetVec(), h->GetAngle());
    WOEdge *hprev = h->getPrevOnFace();
    nc.accumulate_dihedral_angle(hprev->GetVec(), hprev->GetAngle());
  }
}

}  // namespace OGF

} /* namespace Freestyle */
