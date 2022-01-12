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

#pragma once

/** \file
 * \ingroup freestyle
 * \brief Class to represent a transform node. A Transform node contains one or several children,
 * \brief all affected by the transformation.
 */

#include "NodeGroup.h"

#include "../geometry/Geom.h"

#include "../system/FreestyleConfig.h"

namespace Freestyle {

using namespace Geometry;

class NodeTransform : public NodeGroup {
 public:
  inline NodeTransform() : NodeGroup()
  {
    _Matrix = Matrix44r::identity();
    _Scaled = false;
  }

  virtual ~NodeTransform()
  {
  }

  /** Accept the corresponding visitor */
  virtual void accept(SceneVisitor &v);

  /** Overloads the Node::AddBBox in order to take care about the transformation */
  virtual void AddBBox(const BBox<Vec3r> &iBBox);

  /** Checks whether a matrix contains a scale factor or not.
   *  Returns true if yes.
   *    M
   *      The matrix to check
   */
  bool isScaled(const Matrix44r &M);

  /** accessors */
  inline const Matrix44r &matrix() const
  {
    return _Matrix;
  }

  inline bool scaled() const
  {
    return _Scaled;
  }

 private:
  Matrix44r _Matrix;
  bool _Scaled;
};

} /* namespace Freestyle */
