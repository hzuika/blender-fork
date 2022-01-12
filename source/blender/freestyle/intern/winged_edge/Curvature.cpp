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
 *
 * The Original Code is:
 *     GTS - Library for the manipulation of triangulated surfaces
 *     Copyright (C) 1999 Stephane Popinet
 * and:
 *     OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *     Copyright (C) 2000-2003 Bruno Levy
 *     Contact: Bruno Levy <levy@loria.fr>
 *         ISA Project
 *         LORIA, INRIA Lorraine,
 *         Campus Scientifique, BP 239
 *         54506 VANDOEUVRE LES NANCY CEDEX
 *         FRANCE
 */

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

static bool angle_obtuse(WVertex *v, WFace *f)
{
  WOEdge *e;
  f->getOppositeEdge(v, e);

  Vec3r vec1(e->GetaVertex()->GetVertex() - v->GetVertex());
  Vec3r vec2(e->GetbVertex()->GetVertex() - v->GetVertex());
  return ((vec1 * vec2) < 0);
}

// FIXME
// WVvertex is useless but kept for history reasons
static bool triangle_obtuse(WVertex *UNUSED(v), WFace *f)
{
  bool b = false;
  for (int i = 0; i < 3; i++) {
    b = b || ((f->getEdgeList()[i]->GetVec() * f->getEdgeList()[(i + 1) % 3]->GetVec()) < 0);
  }
  return b;
}

static real cotan(WVertex *vo, WVertex *v1, WVertex *v2)
{
  /* cf. Appendix B of [Meyer et al 2002] */
  real udotv, denom;

  Vec3r u(v1->GetVertex() - vo->GetVertex());
  Vec3r v(v2->GetVertex() - vo->GetVertex());

  udotv = u * v;
  denom = sqrt(u.squareNorm() * v.squareNorm() - udotv * udotv);

  /* denom can be zero if u==v.  Returning 0 is acceptable, based on the callers of this function
   * below. */
  if (denom == 0.0) {
    return 0.0;
  }
  return (udotv / denom);
}

static real angle_from_cotan(WVertex *vo, WVertex *v1, WVertex *v2)
{
  /* cf. Appendix B and the caption of Table 1 from [Meyer et al 2002] */
  real udotv, denom;

  Vec3r u(v1->GetVertex() - vo->GetVertex());
  Vec3r v(v2->GetVertex() - vo->GetVertex());

  udotv = u * v;
  denom = sqrt(u.squareNorm() * v.squareNorm() - udotv * udotv);

  /* NOTE(Ray Jones): I assume this is what they mean by using #atan2. */

  /* tan = denom/udotv = y/x (see man page for atan2) */
  return (fabs(atan2(denom, udotv)));
}

void gts_vertex_principal_curvatures(real Kh, real Kg, real *K1, real *K2)
{
  real temp = Kh * Kh - Kg;

  if (!K1 || !K2) {
    return;
  }

  if (temp < 0.0) {
    temp = 0.0;
  }
  temp = sqrt(temp);
  *K1 = Kh + temp;
  *K2 = Kh - temp;
}

/* from Maple */
static void linsolve(real m11, real m12, real b1, real m21, real m22, real b2, real *x1, real *x2)
{
  real temp;

  temp = 1.0 / (m21 * m12 - m11 * m22);
  *x1 = (m12 * b2 - m22 * b1) * temp;
  *x2 = (m11 * b2 - m21 * b1) * temp;
}

/* from Maple - largest eigenvector of [a b; b c] */
static void eigenvector(real a, real b, real c, Vec3r e)
{
  if (b == 0.0) {
    e[0] = 0.0;
  }
  else {
    e[0] = -(c - a - sqrt(c * c - 2 * a * c + a * a + 4 * b * b)) / (2 * b);
  }
  e[1] = 1.0;
  e[2] = 0.0;
}

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
