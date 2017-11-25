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


#ifndef _H_COBBS_VISION_UNDISTORT_H_
#define _H_COBBS_VISION_UNDISTORT_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

#include <cobbs/vision/vision.hpp>

/*****************************************************************************/
// namespace vision
/*****************************************************************************/
namespace vision {
  
  /*****************************************************************************/
  // undistort
  /*****************************************************************************/
  template <class T>
  inline void undistort(const std::vector<T> & src, std::vector<T> & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    
    dst.resize(src.size());
    
    cv::undistortPoints(src, dst, cameraMatrix, distortionCoefficients);
    
    double fx = GET_OMEGA_X(cameraMatrix);
    double fy = GET_OMEGA_Y(cameraMatrix);
    
    double cx = GET_OPTICALCENTER_X(cameraMatrix);
    double cy = GET_OPTICALCENTER_Y(cameraMatrix);

    for(std::size_t i=0; i<src.size(); ++i){
      dst[i].x = fx * src[i].x + cx;
      dst[i].y = fy * src[i].y + cy;
    }
    
  }
  
  /*****************************************************************************/
  // undistort
  /*****************************************************************************/
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
  
  /*****************************************************************************/
  // undistort
  /*****************************************************************************/
  template <class T>
  inline void undistort(T & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }
  
  
  /*****************************************************************************/
  // undistort
  /*****************************************************************************/
  template <class T>
  inline void undistort(std::vector<T> & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }
  
  
  /*****************************************************************************/
  // undistort
  /*****************************************************************************/
  inline void undistort(const cv::Mat & src, cv::Mat & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    
    dst = src.clone();
    
    cv::undistort(src, dst, cameraMatrix, distortionCoefficients);
    
  }
  
  
  /*****************************************************************************/
  // undistort
  /*****************************************************************************/
  inline void undistort(cv::Mat & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }
  
  
  
} /* namespace vision */

#endif /* _H_COBBS_VISION_UNDISTORT_H_ */
