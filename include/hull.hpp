/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2019
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

#ifndef _H_MPL_HULL_H_
#define _H_MPL_HULL_H_

#include <cstdlib>
#include <cstdio>

#include <algorithm>
#include <vector>

#include <opencv2/opencv.hpp>

#include <mpl/opencv.hpp>


/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // Andrew's monotone chain 2D convex hull algorithm
  /*****************************************************************************/
  // Returns a list of points on the convex hull in counter-clockwise order.
  // Note: the last point in the returned list is the same as the first one.
  template <typename T>
  std::vector<cv::Point_<T>> hull(std::vector<cv::Point_<T>> points) {
    
    size_t n = points.size();
    
    if(n == 1) return points;
    
    std::vector<cv::Point_<T>> H(2*n);
    
    // Sort points lexicographically
    std::sort(points.begin(), points.end());
    
    size_t k = 0;

    // Build lower hull
    for(size_t i = 0; i < n; ++i) {
      while(k >= 2 && cv::cross(H[k-2], H[k-1], points[i]) <= 0) --k;
      H[k++] = points[i];
    }
    
    // Build upper hull
    for(size_t i = n-2, t = k+1; i >= 0; i--) {
      while(k >= t && cv::cross(H[k-2], H[k-1], points[i]) <= 0) --k;
      H[k++] = points[i];
    }
    
    H.resize(k-1);
    
    return H;
    
  }
  
} /* namespace mpl::hull */


#endif /* _H_MPL_HULL_H_ */





