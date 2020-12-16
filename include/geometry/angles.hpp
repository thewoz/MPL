/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017
 * Created by Leonardo Parisi (leonardo.parisi[at]gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _H_MPL_ANGLE_H_
#define _H_MPL_ANGLE_H_

#include <cstdlib>
#include <cstdio>

#include <cmath>

/*****************************************************************************/
// namespace mpl::geometry
/*****************************************************************************/
namespace mpl::angles {
  
  /*****************************************************************************/
  // namespace utils
  /*****************************************************************************/
  namespace utils {
    
    const double deg2rad = M_PI  / 180.0;
    const double rad2deg = 180.0 /  M_PI;
    
  }
  
  inline double radians(double deg, double min, double sec) { return (deg + min/60.0 + sec/3600.0) * utils::deg2rad; }
  inline double radians(double deg) { return deg * utils::deg2rad; }
  inline double degrees(double rad) { return rad * utils::rad2deg; }
 
} /* namespace mpl::geometry */

#endif /* _H_MPL_ANGLE_H_ */





