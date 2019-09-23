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


#ifndef _H_COBBS_VISION_REPROJECTION_H_
#define _H_COBBS_VISION_REPROJECTION_H_

#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>


/*****************************************************************************/
// namespace vision
/*****************************************************************************/
namespace mpl::vision {

  /*****************************************************************************/
  // reproject
  /*****************************************************************************/
  template <typename T>
  inline void reproject(const cv::Point3_<T> & point3D, cv::Point_<T> & point2D, const double * P){
  
    double w =   P[8] * point3D.x + P[9] * point3D.y + P[10] * point3D.z + P[11];
    
    point2D.x = (P[0] * point3D.x + P[1] * point3D.y + P[02] * point3D.z + P[03]) / w;
    point2D.y = (P[4] * point3D.x + P[5] * point3D.y + P[06] * point3D.z + P[07]) / w;

  }
 
  /*****************************************************************************/
  // reproject
  /*****************************************************************************/
//  template <typename T2D, typename T3D>
//  inline void reproject(const std::vector< cv::Point3_<T3D> > & points3D, std::vector< cv::Point_<T2D> > & points2D, const double * P){
//
//    if(points3D.size() != points2D.size()){
//      fprintf(stderr, "ss\n");
//      abort();
//    }
//
//    for(std::size_t i=0; i<points3D.size(); ++i){
//
//      double w =   P[8] * points3D[i].x + P[9] * points3D[i].y + P[10] * points3D[i].z + P[11];
//
//      points2D[i].x = (P[0] * points3D[i].x + P[1] * points3D[i].y + P[02] * points3D[i].z + P[03]) / w;
//      points2D[i].y = (P[4] * points3D[i].x + P[5] * points3D[i].y + P[06] * points3D[i].z + P[07]) / w;
//
//    }
//
//  }
  
  /*****************************************************************************/
  // reproject
  /*****************************************************************************/
  template <typename T>
  inline cv::Point_<T> reproject(const cv::Point3_<T> & point3D, const double * P){
    
    cv::Point_<T> point2D;
    
    reproject(point3D, point2D, P);
    
    return point2D;
    
  }
  
  /*****************************************************************************/
  // reproject
  /*****************************************************************************/
//  template <typename T2D, typename T3D>
//  inline std::vector< cv::Point_<T2D> > reproject(const std::vector< cv::Point3_<T3D> > & point3D, const double * P){
//
//    std::vector< cv::Point_<T2D> > point2D;
//
//    reprojection(point3D, point2D, P);
//
//    return point2D;
//
//  }
  
  
  /*****************************************************************************/
  // reproject
  /*****************************************************************************/
//  template <typename T2D, typename T3D>
//  inline void reproject(const cv::Point3_<T3D> & point3D, cv::Point_<T2D> & point2D, const cv::Mat & projectionMatrix){
//
//    reproject(point3D, point2D, (double *)projectionMatrix.data);
//
//  }
  
  /*****************************************************************************/
  // reproject
  /*****************************************************************************/
//  template <typename T2D, typename T3D>
//  inline void reproject(const std::vector< cv::Point3_<T3D> > & points3D, std::vector< cv::Point_<T2D> > & points2D, const cv::Mat & projectionMatrix){
//
//    reproject(points3D, points2D, (double *)projectionMatrix.data);
//
//  }
  
  /*****************************************************************************/
  // reproject
  /*****************************************************************************/
  template <typename T>
  inline cv::Point_<T> reproject(const cv::Point3_<T> & point3D, const cv::Mat & projectionMatrix){
    
    return reproject(point3D, (double *)projectionMatrix.data);
    
  }
  
  /*****************************************************************************/
  // reproject
  /*****************************************************************************/
//  template <typename T2D, typename T3D>
//  inline std::vector< cv::Point_<T2D> > reproject(const std::vector< cv::Point3_<T3D> > & points3D, const cv::Mat & projectionMatrix){
//    
//    return reprojection(points3D, (double *)projectionMatrix.data);
//    
//  }
  
  
} /* namespace vision */



#endif /* _H_COBBS_VISION_REPROJECTION_H_ */


