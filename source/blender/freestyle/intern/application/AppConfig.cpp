/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup freestyle
 */

#include "AppConfig.h"
#include <iostream>

#include "../system/StringUtils.h"

using namespace std;

#include "BKE_appdir.h"

namespace Freestyle::Config {

Path *Path::_pInstance = nullptr;
Path::Path()
{
  _pInstance = this;
}

Path::~Path()
{
  _pInstance = nullptr;
}

Path *Path::getInstance()
{
  return _pInstance;
}

string Path::getEnvVar(const string &iEnvVarName)
{
  string value;
  if (!getenv(iEnvVarName.c_str())) {
    cerr << "Warning: You may want to set the $" << iEnvVarName
         << " environment variable to use Freestyle." << endl
         << "         Otherwise, the current directory will be used instead." << endl;
    value = ".";
  }
  else {
    value = getenv(iEnvVarName.c_str());
  }
  return value;
}

}  // namespace Freestyle::Config
