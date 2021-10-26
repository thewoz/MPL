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

#ifndef _H_MPL_VISION_REPROJECTION_H_
#define _H_MPL_VISION_REPROJECTION_H_

#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include <mpl/opencv.hpp>


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
  template <typename T>
  inline cv::Point_<T> reproject(const cv::Point3_<T> & point3D, const double * P){
    
    cv::Point_<T> point2D;
    
    reproject(point3D, point2D, P);
    
    return point2D;
    
  }

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
  template <typename T>
  inline std::vector<cv::Point_<T>> reproject(const std::vector<cv::Point3_<T>> & point3D, const cv::Mat & projectionMatrix){
    
      std::vector<cv::Point_<T>> points2D(point3D.size());

      for(size_t i=0; i<point3D.size(); ++i) {

        points2D[i] = reproject(point3D[i], (double *)projectionMatrix.data);

      }

    return  points2D;
    
  }
  
/*****************************************************************************/
// reproject
/*****************************************************************************/
template <typename T>
inline void reproject(const std::vector<cv::Point3_<T>> & point3D, std::vector<cv::Point_<T>> & points2D, const cv::Mat & projectionMatrix){
  
    points2D.resize(point3D.size());

    for(size_t i=0; i<point3D.size(); ++i) {

      points2D[i] = reproject(point3D[i], (double *)projectionMatrix.data);

    }
  
}
  
/*****************************************************************************/
// reproject
/*****************************************************************************/
template <typename T>
inline void reproject(const cv::Point3_<T> & point3D, cv::Point_<T> & points2D, const cv::Mat & projectionMatrix){
  

      points2D = reproject(point3D, (double *)projectionMatrix.data);

}
  
  /*****************************************************************************/
  // reproject
  /*****************************************************************************/
  template <typename T>
  inline void reproject(const cv::Point4_<T> & point4D, cv::Point_<T> & point2D, const double * P){
   
    double w = (P[8] * point4D.x + P[9] * point4D.y + P[10] * point4D.z + P[11] * point4D.w);
    
    point2D.x = (P[0] * point4D.x + P[1] * point4D.y + P[02] * point4D.z + P[03] * point4D.w) / w;
    point2D.y = (P[4] * point4D.x + P[5] * point4D.y + P[06] * point4D.z + P[07] * point4D.w) / w;

  }

   /*****************************************************************************/
   // reproject
   /*****************************************************************************/
   template <typename T>
   inline cv::Point_<T> reproject(const cv::Point4_<T> & point4D, const double * P){
     
     cv::Point_<T> point2D;
     
     reproject(point4D, point2D, P);
     
     return point2D;
     
   }
   
   /*****************************************************************************/
   // reproject
   /*****************************************************************************/
   template <typename T>
   inline cv::Point_<T> reproject(const cv::Point4_<T> & point4D, const cv::Mat & projectionMatrix){
     
     return reproject(point4D, (double *)projectionMatrix.data);
     
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
//  template <typename T2D, typename T3D>
//  inline std::vector< cv::Point_<T2D> > reproject(const std::vector< cv::Point3_<T3D> > & points3D, const cv::Mat & projectionMatrix){
//    
//    return reprojection(points3D, (double *)projectionMatrix.data);
//    
//  }
  
  
} /* namespace vision */



#endif /* _H_MPL_VISION_REPROJECTION_H_ */


