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
 */

#include "AppConfig.h"
#include <iostream>

#include "../system/FreestyleConfig.h"
#include "../system/StringUtils.h"

using namespace std;

#include "BKE_appdir.h"

namespace Freestyle::Config {

Path *Path::_pInstance = nullptr;
Path::Path()
{
  // get the root directory
  // soc
  setRootDir(BKE_appdir_folder_id(BLENDER_SYSTEM_SCRIPTS, nullptr));

  _pInstance = this;
}

void Path::setRootDir(const string &iRootDir)
{
  _ProjectDir = iRootDir + string(DIR_SEP) + "freestyle";
  _PatternsPath = _ProjectDir + string(DIR_SEP) + "data" + string(DIR_SEP) + "textures" +
                  string(DIR_SEP) + "variation_patterns" + string(DIR_SEP);
  _BrushesPath = _ProjectDir + string(DIR_SEP) + "data" + string(DIR_SEP) + "textures" +
                 string(DIR_SEP) + "brushes" + string(DIR_SEP);
  _MapsDir = _ProjectDir + string(DIR_SEP) + "data" + string(DIR_SEP) + "maps" + string(DIR_SEP);
}

Path::~Path()
{
  _pInstance = nullptr;
}

Path *Path::getInstance()
{
  return _pInstance;
}

}  // namespace Freestyle::Config
