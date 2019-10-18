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


#ifndef _H_MPL_VISION_CONVERT_H_
#define _H_MPL_VISION_CONVERT_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

#include <mpl/opencv.hpp>

/*****************************************************************************/
// namespace mpl::vision::convert
/*****************************************************************************/
namespace mpl::vision::convert {
  
  /*****************************************************************************/
  // homogeneous
  /*****************************************************************************/
  void homogeneous(const std::vector<cv::Point2d> & points, std::vector<cv::Point3d> & pointsH) {

    // Alloco lo spazio
    pointsH.resize(points.size());
    
    // Converto i punti
    for(size_t i=0; i<points.size(); ++i) {
      pointsH[i].x = points[i].x;
      pointsH[i].y = points[i].y;
      pointsH[i].z = 1.0;
    }
      
  }
  
  /*****************************************************************************/
  // homogeneous
  /*****************************************************************************/
  void homogeneous(const std::vector<cv::Point3d> & points, std::vector<cv::Point4d> & pointsH) {

    // Alloco lo spazio
    pointsH.resize(points.size());
    
    // Converto i punti
    for(size_t i=0; i<points.size(); ++i) {
      pointsH[i].x = points[i].x;
      pointsH[i].y = points[i].y;
      pointsH[i].z = points[i].z;
      pointsH[i].w = 1.0;
    }
    
  }
  
} /* namespace mpl::vision::convert */

#endif /* _H_MPL_VISION_CONVERT_H_ */
