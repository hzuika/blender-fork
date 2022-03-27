/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup freestyle
 * \brief Base class to define a cell grid surrounding the bounding box of the scene
 */

#include <stdexcept>

#include "BBox.h"
#include "Grid.h"

#include "BLI_utildefines.h"

namespace Freestyle {

// Grid Visitors
/////////////////
bool firstIntersectionGridVisitor::stop()
{
  if (occluder_) {
    return true;
  }
  return false;
}

// Grid
/////////////////
void Grid::clear()
{
  if (!_occluders.empty()) {
    for (OccludersSet::iterator it = _occluders.begin(); it != _occluders.end(); it++) {
      delete (*it);
    }
    _occluders.clear();
  }

  _size = Vec3r(0, 0, 0);
  _cell_size = Vec3r(0, 0, 0);
  _orig = Vec3r(0, 0, 0);
  _cells_nb = Vec3u(0, 0, 0);
  //_ray_occluders.clear();
}

void Grid::configure(const Vec3r &orig, const Vec3r &size, unsigned nb)
{
  _orig = orig;
  Vec3r tmpSize = size;
  // Compute the volume of the desired grid
  real grid_vol = size[0] * size[1] * size[2];

  if (grid_vol == 0) {
    double min = DBL_MAX;
    int index = 0;
    int nzeros = 0;
    for (int i = 0; i < 3; ++i) {
      if (size[i] == 0) {
        ++nzeros;
        index = i;
      }
      if ((size[i] != 0) && (min > size[i])) {
        min = size[i];
      }
    }
    if (nzeros > 1) {
      throw std::runtime_error("Warning: the 3D grid has more than one null dimension");
    }
    tmpSize[index] = min;
    _orig[index] = _orig[index] - min / 2;
  }
  // Compute the desired volume of a single cell
  real cell_vol = grid_vol / nb;
  // The edge of such a cubic cell is cubic root of cellVolume
  real edge = pow(cell_vol, 1.0 / 3.0);

  // We compute the number of cells par edge such as we cover at least the whole box.
  unsigned i;
  for (i = 0; i < 3; i++) {
    _cells_nb[i] = (unsigned)floor(tmpSize[i] / edge) + 1;
  }

  _size = tmpSize;

  for (i = 0; i < 3; i++) {
    _cell_size[i] = _size[i] / _cells_nb[i];
  }
}

void Grid::insertOccluder(Polygon3r *occluder)
{
  const vector<Vec3r> vertices = occluder->getVertices();
  if (vertices.empty()) {
    return;
  }

  // add this occluder to the grid's occluders list
  addOccluder(occluder);

  // find the bbox associated to this polygon
  Vec3r min, max;
  occluder->getBBox(min, max);

  // Retrieve the cell x, y, z coordinates associated with these min and max
  Vec3u imax, imin;
  getCellCoordinates(max, imax);
  getCellCoordinates(min, imin);

  // We are now going to fill in the cells overlapping with the polygon bbox.
  // If the polygon is a triangle (most of cases), we also check for each of these cells if it is
  // overlapping with the triangle in order to only fill in the ones really overlapping the
  // triangle.

  unsigned i, x, y, z;
  vector<Vec3r>::const_iterator it;
  Vec3u coord;

  if (vertices.size() == 3) {  // Triangle case
    Vec3r triverts[3];
    i = 0;
    for (it = vertices.begin(); it != vertices.end(); it++) {
      triverts[i] = Vec3r(*it);
      i++;
    }

    Vec3r boxmin, boxmax;

    for (z = imin[2]; z <= imax[2]; z++) {
      for (y = imin[1]; y <= imax[1]; y++) {
        for (x = imin[0]; x <= imax[0]; x++) {
          coord[0] = x;
          coord[1] = y;
          coord[2] = z;
          // We retrieve the box coordinates of the current cell
          getCellBox(coord, boxmin, boxmax);
          // We check whether the triangle and the box ovewrlap:
          Vec3r boxcenter((boxmin + boxmax) / 2.0);
          Vec3r boxhalfsize(_cell_size / 2.0);
          if (GeomUtils::overlapTriangleBox(boxcenter, boxhalfsize, triverts)) {
            // We must then create the Cell and add it to the cells list if it does not exist yet.
            // We must then add the occluder to the occluders list of this cell.
            Cell *cell = getCell(coord);
            if (!cell) {
              cell = new Cell(boxmin);
              fillCell(coord, *cell);
            }
            cell->addOccluder(occluder);
          }
        }
      }
    }
  }
  else {  // The polygon is not a triangle, we add all the cells overlapping the polygon bbox.
    for (z = imin[2]; z <= imax[2]; z++) {
      for (y = imin[1]; y <= imax[1]; y++) {
        for (x = imin[0]; x <= imax[0]; x++) {
          coord[0] = x;
          coord[1] = y;
          coord[2] = z;
          Cell *cell = getCell(coord);
          if (!cell) {
            Vec3r orig;
            getCellOrigin(coord, orig);
            cell = new Cell(orig);
            fillCell(coord, *cell);
          }
          cell->addOccluder(occluder);
        }
      }
    }
  }
}

} /* namespace Freestyle */
