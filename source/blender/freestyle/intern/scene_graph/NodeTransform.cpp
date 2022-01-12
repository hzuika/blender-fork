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
 * \brief Class to represent a transform node. A Transform node contains one or several children,
 * \brief all affected by the transformation.
 */

#include "NodeTransform.h"

#include "BLI_math.h"

namespace Freestyle {

void NodeTransform::accept(SceneVisitor &v)
{
  v.visitNodeTransform(*this);

  v.visitNodeTransformBefore(*this);
  for (vector<Node *>::iterator node = _Children.begin(), end = _Children.end(); node != end;
       ++node) {
    (*node)->accept(v);
  }
  v.visitNodeTransformAfter(*this);
}

void NodeTransform::AddBBox(const BBox<Vec3r> &iBBox)
{
  Vec3r oldMin(iBBox.getMin());
  Vec3r oldMax(iBBox.getMax());

  // compute the 8 corners of the bbox
  HVec3r box[8];
  box[0] = HVec3r(iBBox.getMin());
  box[1] = HVec3r(oldMax[0], oldMin[1], oldMin[2]);
  box[2] = HVec3r(oldMax[0], oldMax[1], oldMin[2]);
  box[3] = HVec3r(oldMin[0], oldMax[1], oldMin[2]);
  box[4] = HVec3r(oldMin[0], oldMin[1], oldMax[2]);
  box[5] = HVec3r(oldMax[0], oldMin[1], oldMax[2]);
  box[6] = HVec3r(oldMax[0], oldMax[1], oldMax[2]);
  box[7] = HVec3r(oldMin[0], oldMax[1], oldMax[2]);

  // Computes the transform iBBox
  HVec3r tbox[8];
  unsigned int i;
  for (i = 0; i < 8; i++) {
    tbox[i] = _Matrix * box[i];
  }

  Vec3r newMin(tbox[0]);
  Vec3r newMax(tbox[0]);
  for (i = 0; i < 8; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      if (newMin[j] > tbox[i][j]) {
        newMin[j] = tbox[i][j];
      }
      if (newMax[j] < tbox[i][j]) {
        newMax[j] = tbox[i][j];
      }
    }
  }

  BBox<Vec3r> transformBox(newMin, newMax);

  Node::AddBBox(transformBox);
}

bool NodeTransform::isScaled(const Matrix44r &M)
{
  for (unsigned int j = 0; j < 3; j++) {
    real norm = 0;
    for (unsigned int i = 0; i < 3; i++) {
      norm += M(i, j) * M(i, j);
    }
    if ((norm > 1.01) || (norm < 0.99)) {
      return true;
    }
  }

  return false;
}

} /* namespace Freestyle */
