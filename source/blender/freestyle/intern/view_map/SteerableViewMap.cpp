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
 * \brief Convenient access to the steerable ViewMap to which any element of the ViewMap belongs
 * to.
 */

#include <sstream>

#include "Silhouette.h"
#include "SteerableViewMap.h"

#include "../geometry/Geom.h"

#include "../image/Image.h"
#include "../image/ImagePyramid.h"

#include "BKE_global.h"
#include "BLI_math.h"

#include "IMB_imbuf.h"
#include "IMB_imbuf_types.h"

namespace Freestyle {

using namespace Geometry;

SteerableViewMap::SteerableViewMap(unsigned int nbOrientations)
{
  _nbOrientations = nbOrientations;
  _bound = cos(M_PI / (float)_nbOrientations);
  for (unsigned int i = 0; i < _nbOrientations; ++i) {
    _directions.emplace_back(cos((float)i * M_PI / (float)_nbOrientations),
                             sin((float)i * M_PI / (float)_nbOrientations));
  }
  Build();
}

void SteerableViewMap::Build()
{
  _imagesPyramids =
      new ImagePyramid *[_nbOrientations + 1];  // one more map to store the complete visible VM
  memset((_imagesPyramids), 0, (_nbOrientations + 1) * sizeof(ImagePyramid *));
}

SteerableViewMap::SteerableViewMap(const SteerableViewMap &iBrother)
{
  _nbOrientations = iBrother._nbOrientations;
  unsigned int i;
  _bound = iBrother._bound;
  _directions = iBrother._directions;
  _mapping = iBrother._mapping;
  _imagesPyramids =
      new ImagePyramid *[_nbOrientations + 1];  // one more map to store the complete visible VM
  for (i = 0; i <= _nbOrientations; ++i) {
    _imagesPyramids[i] = new GaussianPyramid(
        *(dynamic_cast<GaussianPyramid *>(iBrother._imagesPyramids[i])));
  }
}

SteerableViewMap::~SteerableViewMap()
{
  Clear();
}

void SteerableViewMap::Clear()
{
  unsigned int i;
  if (_imagesPyramids) {
    for (i = 0; i <= _nbOrientations; ++i) {
      if (_imagesPyramids[i]) {
        delete (_imagesPyramids)[i];
      }
    }
    delete[] _imagesPyramids;
    _imagesPyramids = nullptr;
  }
  if (!_mapping.empty()) {
    for (map<unsigned int, double *>::iterator m = _mapping.begin(), mend = _mapping.end();
         m != mend;
         ++m) {
      delete[](*m).second;
    }
    _mapping.clear();
  }
}

void SteerableViewMap::Reset()
{
  Clear();
  Build();
}

double SteerableViewMap::ComputeWeight(const Vec2d &dir, unsigned i)
{
  double dotp = fabs(dir * _directions[i]);
  if (dotp < _bound) {
    return 0.0;
  }
  if (dotp > 1.0) {
    dotp = 1.0;
  }

  return cos((float)_nbOrientations / 2.0 * acos(dotp));
}

double *SteerableViewMap::AddFEdge(FEdge *iFEdge)
{
  unsigned i;
  unsigned id = iFEdge->getId().getFirst();
  map<unsigned int, double *>::iterator o = _mapping.find(id);
  if (o != _mapping.end()) {
    return (*o).second;
  }
  double *res = new double[_nbOrientations];
  for (i = 0; i < _nbOrientations; ++i) {
    res[i] = 0.0;
  }
  Vec3r o2d3 = iFEdge->orientation2d();
  Vec2r o2d2(o2d3.x(), o2d3.y());
  real norm = o2d2.norm();
  if (norm < 1.0e-6) {
    return res;
  }
  o2d2 /= norm;

  for (i = 0; i < _nbOrientations; ++i) {
    res[i] = ComputeWeight(o2d2, i);
  }
  _mapping[id] = res;
  return res;
}

unsigned SteerableViewMap::getSVMNumber(Vec2f dir)
{
  // soc unsigned res = 0;
  real norm = dir.norm();
  if (norm < 1.0e-6) {
    return _nbOrientations + 1;
  }
  dir /= norm;
  double maxw = 0.0f;
  unsigned winner = _nbOrientations + 1;
  for (unsigned int i = 0; i < _nbOrientations; ++i) {
    double w = ComputeWeight(dir, i);
    if (w > maxw) {
      maxw = w;
      winner = i;
    }
  }
  return winner;
}

unsigned SteerableViewMap::getSVMNumber(unsigned id)
{
  map<unsigned int, double *>::iterator o = _mapping.find(id);
  if (o != _mapping.end()) {
    double *wvalues = (*o).second;
    double maxw = 0.0;
    unsigned winner = _nbOrientations + 1;
    for (unsigned i = 0; i < _nbOrientations; ++i) {
      double w = wvalues[i];
      if (w > maxw) {
        maxw = w;
        winner = i;
      }
    }
    return winner;
  }
  return _nbOrientations + 1;
}

float SteerableViewMap::readSteerableViewMapPixel(unsigned iOrientation, int iLevel, int x, int y)
{
  ImagePyramid *pyramid = _imagesPyramids[iOrientation];
  if (!pyramid) {
    if (G.debug & G_DEBUG_FREESTYLE) {
      cout << "Warning: this steerable ViewMap level doesn't exist" << endl;
    }
    return 0.0f;
  }
  if ((x < 0) || (x >= pyramid->width()) || (y < 0) || (y >= pyramid->height())) {
    return 0;
  }
  // float v = pyramid->pixel(x, pyramid->height() - 1 - y, iLevel) * 255.0f;
  // We encode both the directionality and the lines counting on 8 bits (because of frame buffer).
  // Thus, we allow until 8 lines to pass through the same pixel, so that we can discretize the
  // Pi/_nbOrientations angle into 32 slices. Therefore, for example, in the vertical direction, a
  // vertical line will have the value 32 on each pixel it passes through.
  float v = pyramid->pixel(x, pyramid->height() - 1 - y, iLevel) / 32.0f;
  return v;
}

float SteerableViewMap::readCompleteViewMapPixel(int iLevel, int x, int y)
{
  return readSteerableViewMapPixel(_nbOrientations, iLevel, x, y);
}

} /* namespace Freestyle */
