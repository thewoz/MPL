/*
 * MIT License
 *
 * Copyright © 2019
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





