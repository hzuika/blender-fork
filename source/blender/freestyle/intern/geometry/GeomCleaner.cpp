/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup freestyle
 * \brief Class to define a cleaner of geometry providing a set of useful tools
 */

#include <cstdio>
#include <list>
#include <map>

#include "GeomCleaner.h"

#include "../system/TimeUtils.h"

#include "BKE_global.h"

using namespace std;

namespace Freestyle {

void GeomCleaner::SortIndexedVertexArray(const float *iVertices,
                                         unsigned iVSize,
                                         const unsigned *iIndices,
                                         unsigned iISize,
                                         float **oVertices,
                                         unsigned **oIndices)
{
  // First, we build a list of IndexVertex:
  list<IndexedVertex> indexedVertices;
  unsigned i;
  for (i = 0; i < iVSize; i += 3) {
    indexedVertices.emplace_back(Vec3f(iVertices[i], iVertices[i + 1], iVertices[i + 2]), i / 3);
  }

  // q-sort
  indexedVertices.sort();

  // build the indices mapping array:
  unsigned *mapIndices = new unsigned[iVSize / 3];
  *oVertices = new float[iVSize];
  list<IndexedVertex>::iterator iv;
  unsigned newIndex = 0;
  unsigned vIndex = 0;
  for (iv = indexedVertices.begin(); iv != indexedVertices.end(); iv++) {
    // Build the final results:
    (*oVertices)[vIndex] = iv->x();
    (*oVertices)[vIndex + 1] = iv->y();
    (*oVertices)[vIndex + 2] = iv->z();

    mapIndices[iv->index()] = newIndex;
    newIndex++;
    vIndex += 3;
  }

  // Build the final index array:
  *oIndices = new unsigned[iISize];
  for (i = 0; i < iISize; i++) {
    (*oIndices)[i] = 3 * mapIndices[iIndices[i] / 3];
  }

  delete[] mapIndices;
}

/** Defines a hash table used for searching the Cells */
struct GeomCleanerHasher {
#define _MUL 950706376UL
#define _MOD 2147483647UL
  inline size_t operator()(const Vec3r &p) const
  {
    size_t res = ((unsigned long)(p[0] * _MUL)) % _MOD;
    res = ((res + (unsigned long)(p[1]) * _MUL)) % _MOD;
    return ((res + (unsigned long)(p[2]) * _MUL)) % _MOD;
  }
#undef _MUL
#undef _MOD
};

void GeomCleaner::CleanIndexedVertexArray(const float *iVertices,
                                          unsigned iVSize,
                                          const unsigned *iIndices,
                                          unsigned iISize,
                                          float **oVertices,
                                          unsigned *oVSize,
                                          unsigned **oIndices)
{
  using cleanHashTable = map<Vec3f, unsigned>;
  vector<Vec3f> vertices;
  unsigned i;
  for (i = 0; i < iVSize; i += 3) {
    vertices.emplace_back(iVertices[i], iVertices[i + 1], iVertices[i + 2]);
  }

  cleanHashTable ht;
  vector<unsigned> newIndices;
  vector<Vec3f> newVertices;

  // elimination of needless points
  unsigned currentIndex = 0;
  vector<Vec3f>::const_iterator v = vertices.begin();
  vector<Vec3f>::const_iterator end = vertices.end();
  cleanHashTable::const_iterator found;
  for (; v != end; v++) {
    found = ht.find(*v);
    if (found != ht.end()) {
      // The vertex is already in the new array.
      newIndices.push_back((*found).second);
    }
    else {
      newVertices.push_back(*v);
      newIndices.push_back(currentIndex);
      ht[*v] = currentIndex;
      currentIndex++;
    }
  }

  // creation of oVertices array:
  *oVSize = 3 * newVertices.size();
  *oVertices = new float[*oVSize];
  currentIndex = 0;
  end = newVertices.end();
  for (v = newVertices.begin(); v != end; v++) {
    (*oVertices)[currentIndex++] = (*v)[0];
    (*oVertices)[currentIndex++] = (*v)[1];
    (*oVertices)[currentIndex++] = (*v)[2];
  }

  // map new indices:
  *oIndices = new unsigned[iISize];
  for (i = 0; i < iISize; i++) {
    (*oIndices)[i] = 3 * newIndices[iIndices[i] / 3];
  }
}

} /* namespace Freestyle */
