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
 * \brief Configuration file
 */

#include <algorithm>
#include <string>

#include "../system/Precision.h"

#ifdef WITH_CXX_GUARDEDALLOC
#  include "MEM_guardedalloc.h"
#endif

using namespace std;

namespace Freestyle {

namespace Config {

class Path {
 protected:
  static Path *_pInstance;
  string _ProjectDir;
  string _ModelsPath;
  string _PatternsPath;
  string _BrushesPath;
  string _EnvMapDir;
  string _MapsDir;
  string _HomeDir;

 public:
  Path();
  virtual ~Path();
  static Path *getInstance();

  void setRootDir(const string &iRootDir);
  void setHomeDir(const string &iHomeDir);

  const string &getProjectDir() const
  {
    return _ProjectDir;
  }
  const string &getModelsPath() const
  {
    return _ModelsPath;
  }
  const string &getPatternsPath() const
  {
    return _PatternsPath;
  }
  const string &getBrushesPath() const
  {
    return _BrushesPath;
  }
  const string &getEnvMapDir() const
  {
    return _EnvMapDir;
  }
  const string &getMapsDir() const
  {
    return _MapsDir;
  }
  const string &getHomeDir() const
  {
    return _HomeDir;
  }

  static string getEnvVar(const string &iEnvVarName);

#ifdef WITH_CXX_GUARDEDALLOC
  MEM_CXX_CLASS_ALLOC_FUNCS("Freestyle:Config:Path")
#endif
};

}  // namespace Config

} /* namespace Freestyle */
