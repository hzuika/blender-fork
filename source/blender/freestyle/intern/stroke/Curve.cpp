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
 * \brief Class to define a container for curves
 */

#include <cstdio> /* printf */

#include "Curve.h"
#include "CurveAdvancedIterators.h"
#include "CurveIterators.h"

#include "BKE_global.h"
#include "BLI_utildefines.h"

namespace Freestyle {

/**********************************/
/*                                */
/*                                */
/*             CurvePoint         */
/*                                */
/*                                */
/**********************************/

CurvePoint::CurvePoint()
{
  __A = nullptr;
  __B = nullptr;
  _t2d = 0;
}

CurvePoint::CurvePoint(SVertex *iA, SVertex *iB, float t)
{
  __A = iA;
  __B = iB;
  _t2d = t;
  if ((iA == nullptr) && (t == 1.0f)) {
    _Point2d = __B->point2d();
    _Point3d = __B->point3d();
  }
  else if ((iB == nullptr) && (t == 0.0f)) {
    _Point2d = __A->point2d();
    _Point3d = __A->point3d();
  }
  else {
    _Point2d = __A->point2d() + _t2d * (__B->point2d() - __A->point2d());
    _Point3d = __A->point3d() + _t2d * (__B->point3d() - __A->point3d());
  }
}

CurvePoint::CurvePoint(CurvePoint *iA, CurvePoint *iB, float t3)
{
  __A = nullptr;
  __B = nullptr;
  float t1 = iA->t2d();
  float t2 = iB->t2d();
  if ((iA->A() == iB->A()) && (iA->B() == iB->B()) && (iA->A() != nullptr) &&
      (iA->B() != nullptr) && (iB->A() != nullptr) && (iB->B() != nullptr)) {
    __A = iA->A();
    __B = iB->B();
    _t2d = t1 + t2 * t3 - t1 * t3;
  }
  else if ((iA->B() == nullptr) && (iB->B() == nullptr)) {
    __A = iA->A();
    __B = iB->A();
    _t2d = t3;
  }
  else if ((iA->t2d() == 0) && (iB->t2d() == 0)) {
    __A = iA->A();
    __B = iB->A();
    _t2d = t3;
  }
  else if (iA->A() == iB->A()) {
  iA_A_eq_iB_A:
    if (iA->t2d() == 0) {
      __A = iB->A();
      __B = iB->B();
      _t2d = t3;
    }
    else if (iB->t2d() == 0) {
      __A = iA->A();
      __B = iA->B();
      _t2d = t3;
    }
  }
  else if (iA->B() == iB->B()) {
  iA_B_eq_iB_B:
    if (iA->t2d() == 1) {
      __A = iB->A();
      __B = iB->B();
      _t2d = t3;
    }
    else if (iB->t2d() == 1) {
      __A = iA->A();
      __B = iA->B();
      _t2d = t3;
    }
  }
  else if (iA->B() == iB->A()) {
  iA_B_eq_iB_A:
    if ((iA->t2d() != 1.0f) && (iB->t2d() == 0.0f)) {
      __A = iA->A();
      __B = iA->B();
      _t2d = t1 + t3 - t1 * t3;
    }
    else if ((iA->t2d() == 1.0f) && (iB->t2d() != 0.0f)) {
      __A = iB->A();
      __B = iB->B();
      _t2d = t2 * t3;
    }
    else if ((iA->getPoint2D() - iB->getPoint2D()).norm() < 1.0e-6) {
      __A = iB->A();
      __B = iB->B();
      _t2d = t2 * t3;
    }
  }
  else if (iA->A() != nullptr && iB->A() != nullptr &&
           (iA->A()->point3d() - iB->A()->point3d()).norm() < 1.0e-6) {
    goto iA_A_eq_iB_A;
  }
  else if (iA->B() != nullptr && iB->B() != nullptr &&
           (iA->B()->point3d() - iB->B()->point3d()).norm() < 1.0e-6) {
    goto iA_B_eq_iB_B;
  }
  else if (iA->B() != nullptr && iB->A() != nullptr &&
           (iA->B()->point3d() - iB->A()->point3d()).norm() < 1.0e-6) {
    goto iA_B_eq_iB_A;
  }

  if (!__A || !__B) {
    if (G.debug & G_DEBUG_FREESTYLE) {
      printf(
          "iA A 0x%p p (%f, %f)\n", iA->A(), iA->A()->getPoint2D().x(), iA->A()->getPoint2D().y());
      printf(
          "iA B 0x%p p (%f, %f)\n", iA->B(), iA->B()->getPoint2D().x(), iA->B()->getPoint2D().y());
      printf(
          "iB A 0x%p p (%f, %f)\n", iB->A(), iB->A()->getPoint2D().x(), iB->A()->getPoint2D().y());
      printf(
          "iB B 0x%p p (%f, %f)\n", iB->B(), iB->B()->getPoint2D().x(), iB->B()->getPoint2D().y());
      printf("iA t2d %f p (%f, %f)\n", iA->t2d(), iA->getPoint2D().x(), iA->getPoint2D().y());
      printf("iB t2d %f p (%f, %f)\n", iB->t2d(), iB->getPoint2D().x(), iB->getPoint2D().y());
    }
    cerr << "Fatal error in CurvePoint::CurvePoint(CurvePoint *iA, CurvePoint *iB, float t3)"
         << endl;
  }
  BLI_assert(__A != nullptr && __B != nullptr);

  _Point2d = iA->point2d() + t3 * (iB->point2d() - iA->point2d());
  _Point3d = __A->point3d() + _t2d * (__B->point3d() - __A->point3d());
}

CurvePoint::CurvePoint(const CurvePoint &iBrother)
{
  __A = iBrother.__A;
  __B = iBrother.__B;
  _t2d = iBrother._t2d;
  _Point2d = iBrother._Point2d;
  _Point3d = iBrother._Point3d;
}

CurvePoint &CurvePoint::operator=(const CurvePoint &iBrother)
{
  __A = iBrother.__A;
  __B = iBrother.__B;
  _t2d = iBrother._t2d;
  _Point2d = iBrother._Point2d;
  _Point3d = iBrother._Point3d;
  return *this;
}

FEdge *CurvePoint::fedge()
{
  if (getNature() & Nature::T_VERTEX) {
    return nullptr;
  }
  return __A->fedge();
}

FEdge *CurvePoint::getFEdge(Interface0D &inter)
{
  CurvePoint *iVertexB = dynamic_cast<CurvePoint *>(&inter);
  if (!iVertexB) {
    cerr << "Warning: CurvePoint::getFEdge() failed to cast the given 0D element to CurvePoint."
         << endl;
    return nullptr;
  }
  if (((__A == iVertexB->__A) && (__B == iVertexB->__B)) ||
      ((__A == iVertexB->__B) && (__B == iVertexB->__A))) {
    return __A->getFEdge(*__B);
  }
  if (__B == nullptr) {
    if (iVertexB->__B == nullptr) {
      return __A->getFEdge(*(iVertexB->__A));
    }
    if (iVertexB->__A == __A) {
      return __A->getFEdge(*(iVertexB->__B));
    }
    if (iVertexB->__B == __A) {
      return __A->getFEdge(*(iVertexB->__A));
    }
  }
  if (iVertexB->__B == nullptr) {
    if (iVertexB->__A == __A) {
      return __B->getFEdge(*(iVertexB->__A));
    }
    if (iVertexB->__A == __B) {
      return __A->getFEdge(*(iVertexB->__A));
    }
  }
  if (__B == iVertexB->__A) {
    if ((_t2d != 1) && (iVertexB->_t2d == 0)) {
      return __A->getFEdge(*__B);
    }
    if ((_t2d == 1) && (iVertexB->_t2d != 0)) {
      return iVertexB->__A->getFEdge(*(iVertexB->__B));
    }
  }
  if (__B == iVertexB->__B) {
    if ((_t2d != 1) && (iVertexB->_t2d == 1)) {
      return __A->getFEdge(*__B);
    }
    if ((_t2d == 1) && (iVertexB->_t2d != 1)) {
      return iVertexB->__A->getFEdge(*(iVertexB->__B));
    }
  }
  if (__A == iVertexB->__A) {
    if ((_t2d == 0) && (iVertexB->_t2d != 0)) {
      return iVertexB->__A->getFEdge(*(iVertexB->__B));
    }
    if ((_t2d != 0) && (iVertexB->_t2d == 0)) {
      return __A->getFEdge(*__B);
    }
  }
  if (__A == iVertexB->__B) {
    if ((_t2d == 0) && (iVertexB->_t2d != 1)) {
      return iVertexB->__A->getFEdge(*(iVertexB->__B));
    }
    if ((_t2d != 0) && (iVertexB->_t2d == 1)) {
      return __A->getFEdge(*__B);
    }
  }
  cerr << "Warning: CurvePoint::getFEdge() failed." << endl;

  return nullptr;
}

Vec3r CurvePoint::normal() const
{
  if (__B == nullptr) {
    return __A->normal();
  }
  if (__A == nullptr) {
    return __B->normal();
  }
  Vec3r Na = __A->normal();
  if (Exception::getException()) {
    Na = Vec3r(0, 0, 0);
  }
  Vec3r Nb = __B->normal();
  if (Exception::getException()) {
    Nb = Vec3r(0, 0, 0);
  }
  // compute t3d:
  real t3d = SilhouetteGeomEngine::ImageToWorldParameter(__A->getFEdge(*__B), _t2d);
  return ((1 - t3d) * Na + t3d * Nb);
}

const SShape *CurvePoint::shape() const
{
  if (__A == nullptr) {
    return __B->shape();
  }
  return __A->shape();
}

occluder_container::const_iterator CurvePoint::occluders_begin() const
{
  if (__A == nullptr) {
    return __B->occluders_begin();
  }
  if (__B == nullptr) {
    return __A->occluders_begin();
  }
  return __A->getFEdge(*__B)->occluders_begin();
}

occluder_container::const_iterator CurvePoint::occluders_end() const
{
  if (__A == nullptr) {
    return __B->occluders_end();
  }
  if (__B == nullptr) {
    return __A->occluders_end();
  }
  return __A->getFEdge(*__B)->occluders_end();
}

bool CurvePoint::occluders_empty() const
{
  if (__A == nullptr) {
    return __B->occluders_empty();
  }
  if (__B == nullptr) {
    return __A->occluders_empty();
  }
  return __A->getFEdge(*__B)->occluders_empty();
}

int CurvePoint::occluders_size() const
{
  if (__A == nullptr) {
    return __B->occluders_size();
  }
  if (__B == nullptr) {
    return __A->occluders_size();
  }
  return __A->getFEdge(*__B)->occluders_size();
}

const SShape *CurvePoint::occluded_shape() const
{
  if (__A == nullptr) {
    return __B->occluded_shape();
  }
  if (__B == nullptr) {
    return __A->occluded_shape();
  }
  return __A->getFEdge(*__B)->occluded_shape();
}

const Polygon3r &CurvePoint::occludee() const
{
  if (__A == nullptr) {
    return __B->occludee();
  }
  if (__B == nullptr) {
    return __A->occludee();
  }
  return __A->getFEdge(*__B)->occludee();
}

bool CurvePoint::occludee_empty() const
{
  if (__A == nullptr) {
    return __B->occludee_empty();
  }
  if (__B == nullptr) {
    return __A->occludee_empty();
  }
  return __A->getFEdge(*__B)->occludee_empty();
}

real CurvePoint::z_discontinuity() const
{
  if (__A == nullptr) {
    return __B->z_discontinuity();
  }
  if (__B == nullptr) {
    return __A->z_discontinuity();
  }
  if (__A->getFEdge(*__B) == nullptr) {
    return 0.0;
  }

  return __A->getFEdge(*__B)->z_discontinuity();
}

/**********************************/
/*                                */
/*                                */
/*             Curve              */
/*                                */
/*                                */
/**********************************/

/* for  functions */

Curve::~Curve()
{
  if (!_Vertices.empty()) {
    for (vertex_container::iterator it = _Vertices.begin(), itend = _Vertices.end(); it != itend;
         ++it) {
      delete (*it);
    }
    _Vertices.clear();
  }
}

/** iterators access */
Curve::point_iterator Curve::points_begin(float step)
{
  vertex_container::iterator second = _Vertices.begin();
  ++second;
  return point_iterator(
      _Vertices.begin(), second, _Vertices.begin(), _Vertices.end(), _nSegments, step, 0.0f, 0.0f);
}

Curve::const_point_iterator Curve::points_begin(float step) const
{
  vertex_container::const_iterator second = _Vertices.begin();
  ++second;
  return const_point_iterator(
      _Vertices.begin(), second, _Vertices.begin(), _Vertices.end(), _nSegments, step, 0.0f, 0.0f);
}

Curve::point_iterator Curve::points_end(float step)
{
  return point_iterator(_Vertices.end(),
                        _Vertices.end(),
                        _Vertices.begin(),
                        _Vertices.end(),
                        _nSegments,
                        step,
                        1.0f,
                        _Length);
}

Curve::const_point_iterator Curve::points_end(float step) const
{
  return const_point_iterator(_Vertices.end(),
                              _Vertices.end(),
                              _Vertices.begin(),
                              _Vertices.end(),
                              _nSegments,
                              step,
                              1.0f,
                              _Length);
}

// Adavnced Iterators access
Curve::point_iterator Curve::vertices_begin()
{
  return points_begin(0);
}

Curve::const_point_iterator Curve::vertices_begin() const
{
  return points_begin(0);
}

Curve::point_iterator Curve::vertices_end()
{
  return points_end(0);
}

Curve::const_point_iterator Curve::vertices_end() const
{
  return points_end(0);
}

// specialized iterators access
CurveInternal::CurvePointIterator Curve::curvePointsBegin(float t)
{
  vertex_container::iterator second = _Vertices.begin();
  ++second;
  return CurveInternal::CurvePointIterator(_Vertices.begin(),
                                           second,
                                           _Vertices.begin(),
                                           _Vertices.end(),
                                           0,
                                           _nSegments,
                                           _Length,
                                           t,
                                           0.0f,
                                           0.0f);
}

CurveInternal::CurvePointIterator Curve::curvePointsEnd(float t)
{
  vertex_container::iterator last = _Vertices.end();
  --last;
  return CurveInternal::CurvePointIterator(last,
                                           _Vertices.end(),
                                           _Vertices.begin(),
                                           _Vertices.end(),
                                           _nSegments,
                                           _nSegments,
                                           _Length,
                                           t,
                                           0.0f,
                                           _Length);
}

CurveInternal::CurvePointIterator Curve::curveVerticesBegin()
{
  return curvePointsBegin(0);
}

CurveInternal::CurvePointIterator Curve::curveVerticesEnd()
{
  return curvePointsEnd(0);
}

Interface0DIterator Curve::pointsBegin(float t)
{
  vertex_container::iterator second = _Vertices.begin();
  ++second;
  Interface0DIterator ret(new CurveInternal::CurvePointIterator(_Vertices.begin(),
                                                                second,
                                                                _Vertices.begin(),
                                                                _Vertices.end(),
                                                                0,
                                                                _nSegments,
                                                                _Length,
                                                                t,
                                                                0.0f,
                                                                0.0f));
  return ret;
}

Interface0DIterator Curve::pointsEnd(float t)
{
  vertex_container::iterator last = _Vertices.end();
  --last;
  Interface0DIterator ret(new CurveInternal::CurvePointIterator(last,
                                                                _Vertices.end(),
                                                                _Vertices.begin(),
                                                                _Vertices.end(),
                                                                _nSegments,
                                                                _nSegments,
                                                                _Length,
                                                                t,
                                                                0.0f,
                                                                _Length));
  return ret;
}

Interface0DIterator Curve::verticesBegin()
{
  return pointsBegin(0);
}

Interface0DIterator Curve::verticesEnd()
{
  return pointsEnd(0);
}

} /* namespace Freestyle */
