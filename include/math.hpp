/*
 * MIT License
 *
 * Copyright © 2017 COBBS
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

#ifndef _H_MPL_MATH_H_
#define _H_MPL_MATH_H_

#include <cmath>

#include <opencv2/opencv.hpp>

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
template <class T>
inline double norm(const cv::Point_<T> & a) {

  return std::sqrt((a.x*a.x) + (a.y*a.y));

}

template <class T>
inline double norm(const cv::Point3_<T> & a) {

  return std::sqrt((a.x*a.x) + (a.y*a.y) + (a.z*a.z));

}

template <class T>
inline double norm(const cv::Point_<T> & a, const cv::Point_<T> & b) {

  return std::sqrt(((a.x-b.x) * (a.x-b.x)) + ((a.y-b.y) * (a.y-b.y)));

}

template <class T>
inline double norm(const cv::Point3_<T> & a, const cv::Point3_<T> & b) {

  return std::sqrt(((a.x-b.x) * (a.x-b.x)) + ((a.y-b.y) * (a.y-b.y)) + ((a.z-b.z) * (a.z-b.z)));

}

template <class T>
inline double norm(const std::vector<T> & A, const std::vector<T> & B){

  if(A.size() != B.size()) {
    fprintf(stderr, "error in mpl::norm() vector must be of the same size\n");
    abort();
  }

  double result = 0;
  
  for(size_t i=0; i<A.size(); ++i)
    result += norm(A[i], B[i]);

  result /= (double) A.size();

  return result;

}


} /* namespace mpl */


  
#endif /* _H_MPL_MATH_H_ */
