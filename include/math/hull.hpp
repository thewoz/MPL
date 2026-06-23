/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017-2026
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

#ifndef _H_MPL_GEOMETRY_HULL_H_
#define _H_MPL_GEOMETRY_HULL_H_

#include <cstdlib>
#include <cstdio>

#include <algorithm>
#include <vector>

#include <opencv2/opencv.hpp>



//*****************************************************************************/
// namespace mpl::geometry
//*****************************************************************************/
namespace mpl::geometry {

  //*****************************************************************************/
  // cross
  //*****************************************************************************/
  // 2D cross product of the vectors OA and OB, i.e. (A-O) x (B-O).
  // Positive if OAB makes a counter-clockwise turn, negative for clockwise.
  template <typename T>
  inline double cross(const cv::Point_<T> & O, const cv::Point_<T> & A, const cv::Point_<T> & B) {
    return ((double)(A.x) - O.x) * ((double)(B.y) - O.y) - ((double)(A.y) - O.y) * ((double)(B.x) - O.x);
  }

  //*****************************************************************************/
  // Andrew's monotone chain 2D convex hull algorithm
  //*****************************************************************************/
  // Returns the convex hull vertices in counter-clockwise order (each vertex
  // appears once; the polygon is not explicitly closed).
  template <typename T>
  std::vector<cv::Point_<T>> hull(std::vector<cv::Point_<T>> points) {

    size_t n = points.size();

    if(n <= 1) return points;

    std::vector<cv::Point_<T>> H(2*n);

    // Sort points lexicographically (by x, then by y)
    std::sort(points.begin(), points.end(), [](const cv::Point_<T> & a, const cv::Point_<T> & b){
      return a.x < b.x || (a.x == b.x && a.y < b.y);
    });

    size_t k = 0;

    // Build lower hull
    for(size_t i = 0; i < n; ++i) {
      while(k >= 2 && cross(H[k-2], H[k-1], points[i]) <= 0) --k;
      H[k++] = points[i];
    }

    // Build upper hull
    for(size_t i = n-1, t = k+1; i-- > 0; ) {
      while(k >= t && cross(H[k-2], H[k-1], points[i]) <= 0) --k;
      H[k++] = points[i];
    }

    H.resize(k-1);

    return H;

  }

} // namespace mpl::geometry


#endif // _H_MPL_GEOMETRY_HULL_H_





