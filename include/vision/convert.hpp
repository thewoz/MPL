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


#ifndef _H_MPL_VISION_CONVERT_H_
#define _H_MPL_VISION_CONVERT_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

//#include <mpl/opencv.hpp>

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
  void homogeneous(const std::vector<cv::Point3d> & points, std::vector<point4d_t> & pointsH) {

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
