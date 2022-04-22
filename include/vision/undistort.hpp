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


#ifndef _H_MPL_VISION_UNDISTORT_H_
#define _H_MPL_VISION_UNDISTORT_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

//****************************************************************************
// namespace vision
//****************************************************************************
namespace mpl::vision {
  
//****************************************************************************
// undistort
//****************************************************************************
template <class T>
inline void undistort(const T & src, T & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
        
  std::vector<T> vsrc; vsrc.push_back(src);
  std::vector<T> dsrc;
  
  cv::undistortPoints(vsrc, dsrc, cameraMatrix, distortionCoefficients);
  
  double fx = cameraMatrix.at<double>(0,0);
  double fy = cameraMatrix.at<double>(1,1);
  double u0 = cameraMatrix.at<double>(0,2);
  double v0 = cameraMatrix.at<double>(1,2);

  dst = dsrc[0];
  
  dst.x = fx * dst.x + u0;
  dst.y = fy * dst.y + v0;
    
}

  //****************************************************************************
  // undistort
  //****************************************************************************
  template <class T>
  inline void undistort(const std::vector<T> & src, std::vector<T> & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
        
    dst.resize(src.size());
    
    cv::undistortPoints(src, dst, cameraMatrix, distortionCoefficients);
    
    double fx = cameraMatrix.at<double>(0,0);
    double fy = cameraMatrix.at<double>(1,1);
    double u0 = cameraMatrix.at<double>(0,2);
    double v0 = cameraMatrix.at<double>(1,2);

    for(std::size_t i=0; i<dst.size(); ++i){
      dst[i].x = fx * dst[i].x + u0;
      dst[i].y = fy * dst[i].y + v0;
    }
    
  }
  
  //****************************************************************************
  // undistort
  //****************************************************************************
  /*
  template <class T>
  inline void undistort(const T & src, T & dst, const cv::Mat & cameraMatrix, const cv::vector<double> & distortionCoefficients) {
    
    std::vector<T> points;
    
    points.push_back(src);
    
    cv::undistortPoints(points, points, cameraMatrix, distortionCoefficients);
    
    dst.x = GET_OMEGA_X(cameraMatrix) * points[0].x + GET_OPTICALCENTER_X(cameraMatrix);
    dst.y = GET_OMEGA_Y(cameraMatrix) * points[0].y + GET_OPTICALCENTER_Y(cameraMatrix);
    
  }
  */
  
  //****************************************************************************
  // undistort
  //****************************************************************************
  template <class T>
  inline void undistort(T & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }
  
  
  //****************************************************************************
  // undistort
  //****************************************************************************
  template <class T>
  inline void undistort(std::vector<T> & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }
  
  
  //****************************************************************************
  // undistort
  //****************************************************************************
  inline void undistort(const cv::Mat & src, cv::Mat & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    
    dst = src.clone();
    
    cv::undistort(src, dst, cameraMatrix, distortionCoefficients);
    
  }
  
  
  //****************************************************************************
  // undistort
  //****************************************************************************
  inline void undistort(cv::Mat & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }
  









 //****************************************************************************
  // undistort
  //****************************************************************************
  template <class T>
  inline void undistortNorm(const std::vector<T> & src, std::vector<T> & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    
    dst.resize(src.size());
    
    cv::undistortPoints(src, dst, cameraMatrix, distortionCoefficients);
    
  }
  
  //****************************************************************************
  // undistort
  //****************************************************************************
  /*
  template <class T>
  inline void undistort(const T & src, T & dst, const cv::Mat & cameraMatrix, const cv::vector<double> & distortionCoefficients) {
    
    std::vector<T> points;
    
    points.push_back(src);
    
    cv::undistortPoints(points, points, cameraMatrix, distortionCoefficients);
    
    dst.x = GET_OMEGA_X(cameraMatrix) * points[0].x + GET_OPTICALCENTER_X(cameraMatrix);
    dst.y = GET_OMEGA_Y(cameraMatrix) * points[0].y + GET_OPTICALCENTER_Y(cameraMatrix);
    
  }
  */
  
  //****************************************************************************
  // undistort
  //****************************************************************************
  template <class T>
  inline void undistortNorm(T & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }
  
  
  //****************************************************************************
  // undistort
  //****************************************************************************
  template <class T>
  inline void undistortNorm(std::vector<T> & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }
  
  
  //****************************************************************************
  // undistort
  //****************************************************************************
  inline void undistortNorm(const cv::Mat & src, cv::Mat & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    
    dst = src.clone();
    
    cv::undistort(src, dst, cameraMatrix, distortionCoefficients);
    
  }
  
  
  //****************************************************************************
  // undistort
  //****************************************************************************
  inline void undistortNorm(cv::Mat & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }
  



















  
  
} /* namespace vision */

#endif /* _H_MPL_VISION_UNDISTORT_H_ */
