/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup freestyle
 * \brief Class to represent a transform node. A Transform node contains one or several children,
 * \brief all affected by the transformation.
 */

#include "NodeGroup.h"

#include "../geometry/Geom.h"

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
