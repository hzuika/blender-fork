/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup freestyle
 * \brief Base class to define a cell grid surrounding the bounding box of the scene
 */

#include <cstring>  // for memset
#include <float.h>
#include <stdint.h>  // For POINTER_FROM_UINT, i.e. uintptr_t.
#include <vector>

#include "Geom.h"
#include "GeomUtils.h"
#include "Polygon.h"

#include "BLI_utildefines.h"

#ifdef WITH_CXX_GUARDEDALLOC
#  include "MEM_guardedalloc.h"
#endif

using namespace std;

namespace Freestyle {

using namespace Geometry;

typedef vector<Polygon3r *> OccludersSet;

//
// Class to define cells used by the regular grid
//
///////////////////////////////////////////////////////////////////////////////

class Cell {
 public:
  Cell(Vec3r &orig)
  {
    _orig = orig;
  }

  virtual ~Cell()
  {
  }

  inline void addOccluder(Polygon3r *o)
  {
    if (o) {
      _occluders.push_back(o);
    }
  }

  inline const Vec3r &getOrigin()
  {
    return _orig;
  }

  inline OccludersSet &getOccluders()
  {
    return _occluders;
  }

 private:
  Vec3r _orig;
  OccludersSet _occluders;

#ifdef WITH_CXX_GUARDEDALLOC
  MEM_CXX_CLASS_ALLOC_FUNCS("Freestyle:Cell")
#endif
};

class GridVisitor {
 public:
  virtual ~GridVisitor(){};  // soc

  virtual void discoverCell(Cell * /*cell*/)
  {
  }

  virtual void finishCell(Cell * /*cell*/)
  {
  }

  virtual bool stop()
  {
    return false;
  }

#ifdef WITH_CXX_GUARDEDALLOC
  MEM_CXX_CLASS_ALLOC_FUNCS("Freestyle:GridVisitor")
#endif
};

/** Gathers all the occluders belonging to the cells traversed by the ray */
class allOccludersGridVisitor : public GridVisitor {
 public:
  allOccludersGridVisitor(OccludersSet &occluders) : GridVisitor(), occluders_(occluders)
  {
  }

  OccludersSet &occluders()
  {
    return occluders_;
  }

  void clear()
  {
    occluders_.clear();
  }

 private:
  OccludersSet &occluders_;
};

/** Finds the first intersection and breaks.
 *  The occluder and the intersection information are stored and accessible.
 */
class firstIntersectionGridVisitor : public GridVisitor {
  // soc - changed order to remove warnings
 public:
  double u_, v_, t_;

 private:
  Polygon3r *occluder_;
  Vec3r ray_org_, ray_dir_, cell_size_;
  Cell *current_cell_;

 public:
  firstIntersectionGridVisitor(const Vec3r &ray_org, const Vec3r &ray_dir, const Vec3r &cell_size)
      : GridVisitor(),
        u_(0),
        v_(0),
        t_(DBL_MAX),
        occluder_(0),
        ray_org_(ray_org),
        ray_dir_(ray_dir),
        cell_size_(cell_size),
        current_cell_(0)
  {
  }

  virtual ~firstIntersectionGridVisitor()
  {
  }

  virtual void discoverCell(Cell *cell)
  {
    current_cell_ = cell;
  }

  virtual bool stop();

  Polygon3r *occluder()
  {
    return occluder_;
  }
};

//
// Class to define a regular grid used for ray casting computations
//
///////////////////////////////////////////////////////////////////////////////

class Grid {
 public:
  /** Builds a Grid. Must be followed by a call to configure() */
  Grid()
  {
  }

  virtual ~Grid()
  {
    clear();
  }

  /** clears the grid
   *  Deletes all the cells, clears the hashtable, resets size, size of cell, number of cells.
   */
  virtual void clear();

  /** Sets the different parameters of the grid
   *    orig
   *      The grid origin
   *    size
   *      The grid's dimensions
   *    nb
   *      The number of cells of the grid
   */
  virtual void configure(const Vec3r &orig, const Vec3r &size, unsigned nb);

  /** returns a vector of integer containing the coordinates of the cell containing the point
   * passed as argument
   *    p
   *      The point for which we're looking the cell
   */
  inline void getCellCoordinates(const Vec3r &p, Vec3u &res)
  {
    int tmp;
    for (int i = 0; i < 3; i++) {
      tmp = (int)((p[i] - _orig[i]) / _cell_size[i]);
      if (tmp < 0) {
        res[i] = 0;
      }
      else if ((unsigned int)tmp >= _cells_nb[i]) {
        res[i] = _cells_nb[i] - 1;
      }
      else {
        res[i] = tmp;
      }
    }
  }

  /** Fills the case corresponding to coord with the cell */
  virtual void fillCell(const Vec3u &coord, Cell &cell) = 0;

  /** returns the cell whose coordinates are passed as argument */
  virtual Cell *getCell(const Vec3u &coord) = 0;

  /** returns the cell containing the point passed as argument.
   *  If the cell is empty (contains no occluder),  NULL is returned:
   *    p
   *      The point for which we're looking the cell
   */
  inline Cell *getCell(const Vec3r &p)
  {
    Vec3u coord;
    getCellCoordinates(p, coord);
    return getCell(coord);
  }

  /** Retrieves the x,y,z coordinates of the origin of the cell whose coordinates (i,j,k)
   *  is passed as argument:
   *    cell_coord
   *      i,j,k integer coordinates for the cell
   *    orig
   *      x,y,x vector to be filled in with the cell origin's coordinates
   */
  inline void getCellOrigin(const Vec3u &cell_coord, Vec3r &orig)
  {
    for (unsigned int i = 0; i < 3; i++) {
      orig[i] = _orig[i] + cell_coord[i] * _cell_size[i];
    }
  }

  /** Retrieves the box corresponding to the cell whose coordinates are passed as argument:
   *    cell_coord
   *      i,j,k integer coordinates for the cell
   *    min_out
   *      The min x,y,x vector of the box. Filled in by the method.
   *    max_out
   *      The max x,y,z coordinates of the box. Filled in by the method.
   */
  inline void getCellBox(const Vec3u &cell_coord, Vec3r &min_out, Vec3r &max_out)
  {
    getCellOrigin(cell_coord, min_out);
    max_out = min_out + _cell_size;
  }

  /** inserts a convex polygon occluder
   *  This method is quite coarse insofar as it adds all cells intersecting the polygon bounding
   * box convex_poly The list of 3D points constituting a convex polygon
   */
  void insertOccluder(Polygon3r *occluder);

  /** Adds an occluder to the list of occluders */
  void addOccluder(Polygon3r *occluder)
  {
    _occluders.push_back(occluder);
  }

  /** Accessors */
  inline const Vec3r &getOrigin() const
  {
    return _orig;
  }

  inline Vec3r gridSize() const
  {
    return _size;
  }

  inline Vec3r getCellSize() const
  {
    return _cell_size;
  }

  // ARB profiling only:
  inline OccludersSet *getOccluders()
  {
    return &_occluders;
  }

  void displayDebug()
  {
    cerr << "Cells nb     : " << _cells_nb << endl;
    cerr << "Cell size    : " << _cell_size << endl;
    cerr << "Origin       : " << _orig << endl;
    cerr << "Occluders nb : " << _occluders.size() << endl;
  }

 protected:
  unsigned int _timestamp;

  Vec3u _cells_nb;   // number of cells for x,y,z axis
  Vec3r _cell_size;  // cell x,y,z dimensions
  Vec3r _size;       // grid x,y,x dimensions
  Vec3r _orig;       // grid origin

  Vec3r _ray_dir;       // direction vector for the ray
  Vec3u _current_cell;  // The current cell being processed (designated by its 3 coordinates)
  Vec3r _pt;    // Points corresponding to the incoming and outgoing intersections of one cell with
                // the ray
  real _t_end;  // To know when we are at the end of the ray
  real _t;

  // OccludersSet _ray_occluders; // Set storing the occluders contained in the cells traversed by
  // a ray
  OccludersSet _occluders;  // List of all occluders inserted in the grid

#ifdef WITH_CXX_GUARDEDALLOC
  MEM_CXX_CLASS_ALLOC_FUNCS("Freestyle:Grid")
#endif
};

//
// Class to walk through occluders in grid without building intermediate data structures
//
///////////////////////////////////////////////////////////////////////////////

class VirtualOccludersSet {
 public:
  VirtualOccludersSet(Grid &_grid) : grid(_grid){};
  Polygon3r *begin();
  Polygon3r *next();
  Polygon3r *next(bool stopOnNewCell);

 private:
  Polygon3r *firstOccluderFromNextCell();
  Grid &grid;
  OccludersSet::iterator it, end;

#ifdef WITH_CXX_GUARDEDALLOC
  MEM_CXX_CLASS_ALLOC_FUNCS("Freestyle:VirtualOccludersSet")
#endif
};

} /* namespace Freestyle */
