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
 * \brief Class to define a cell grid surrounding the projected image of a scene
 */

#include <algorithm>

#include "OccluderSource.h"

#include "BKE_global.h"

namespace Freestyle {

OccluderSource::OccluderSource(const GridHelpers::Transform &t, WingedEdge &we)
    : wingedEdge(we), valid(false), transform(t)
{
  begin();
}

OccluderSource::~OccluderSource() = default;

void OccluderSource::buildCachedPolygon()
{
  vector<Vec3r> vertices(GridHelpers::enumerateVertices((*currentFace)->getEdgeList()));
  // This doesn't work, because our functor's polymorphism won't survive the copy:
  // so we have to do:
  for (vector<Vec3r>::iterator i = vertices.begin(); i != vertices.end(); ++i) {
    (*i) = transform(*i);
  }
  cachedPolygon = Polygon3r(vertices, transform((*currentFace)->GetNormal()));
}

void OccluderSource::begin()
{
  vector<WShape *> &wshapes = wingedEdge.getWShapes();
  currentShape = wshapes.begin();
  shapesEnd = wshapes.end();
  valid = false;
  if (currentShape != shapesEnd) {
    vector<WFace *> &wFaces = (*currentShape)->GetFaceList();
    currentFace = wFaces.begin();
    facesEnd = wFaces.end();

    if (currentFace != facesEnd) {
      buildCachedPolygon();
      valid = true;
    }
  }
}

bool OccluderSource::next()
{
  if (valid) {
    ++currentFace;
    while (currentFace == facesEnd) {
      ++currentShape;
      if (currentShape == shapesEnd) {
        valid = false;
        return false;
      }

      vector<WFace *> &wFaces = (*currentShape)->GetFaceList();
      currentFace = wFaces.begin();
      facesEnd = wFaces.end();
    }
    buildCachedPolygon();
    return true;
  }
  return false;
}

bool OccluderSource::isValid()
{
  // Or:
  return valid;
}

WFace *OccluderSource::getWFace()
{
  return valid ? *currentFace : nullptr;
}

Polygon3r OccluderSource::getCameraSpacePolygon()
{
  return Polygon3r(GridHelpers::enumerateVertices((*currentFace)->getEdgeList()),
                   (*currentFace)->GetNormal());
}

Polygon3r &OccluderSource::getGridSpacePolygon()
{
  return cachedPolygon;
}

} /* namespace Freestyle */
