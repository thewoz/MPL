/*
 * MIT License
 *
 * Copyright © 2017
 * Created by Leonardo Parisi (leonardo.parisi[at]gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _H_MPL_ANGLE_H_
#define _H_MPL_ANGLE_H_

#include <cstdlib>
#include <cstdio>

#include <cmath>

/*****************************************************************************/
// namespace mpl::geometry
/*****************************************************************************/
namespace mpl::geometry {
  
  /*****************************************************************************/
  // namespace utils
  /*****************************************************************************/
  namespace utils {
    
    const double deg2rad = M_PI  / 180.0;
    const double rad2deg = 180.0 /  M_PI;
    
  }
  
  inline double Radians(double deg, double min, double sec) { return (deg + min/60.0 + sec/3600.0) * utils::deg2rad; }
  inline double Radians(double deg) { return deg * utils::deg2rad; }
  inline double Degrees(double rad) { return rad * utils::rad2deg; }
 
} /* namespace mpl::geometry */

#endif /* _H_MPL_ANGLE_H_ */





