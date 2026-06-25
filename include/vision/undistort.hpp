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


#ifndef _H_MPL_VISION_UNDISTORT_H_
#define _H_MPL_VISION_UNDISTORT_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

//****************************************************************************/
// namespace mpl::vision
//****************************************************************************/
namespace mpl::vision {

  //****************************************************************************/
  // undistortNorm - undistort returning normalized (camera) coordinates,
  //                 i.e. the intrinsic matrix K is NOT reapplied.
  //****************************************************************************/
  template <class T>
  inline void undistortNorm(const std::vector<T> & src, std::vector<T> & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {

    dst.resize(src.size());

    cv::undistortPoints(src, dst, cameraMatrix, distortionCoefficients);

  }

  template <class T>
  inline void undistortNorm(const T & src, T & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {

    std::vector<T> in{src}; std::vector<T> out;

    cv::undistortPoints(in, out, cameraMatrix, distortionCoefficients);

    dst = out[0];

  }

  template <class T>
  inline void undistortNorm(std::vector<T> & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistortNorm(src, src, cameraMatrix, distortionCoefficients);
  }

  template <class T>
  inline void undistortNorm(T & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistortNorm(src, src, cameraMatrix, distortionCoefficients);
  }


  //****************************************************************************/
  // undistort - undistort points keeping them in pixel coordinates
  //             (normalized result reprojected through the intrinsic matrix K).
  //****************************************************************************/
  template <class T>
  inline void undistort(const std::vector<T> & src, std::vector<T> & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {

    undistortNorm(src, dst, cameraMatrix, distortionCoefficients);

    const double fx = cameraMatrix.at<double>(0,0);
    const double fy = cameraMatrix.at<double>(1,1);
    const double u0 = cameraMatrix.at<double>(0,2);
    const double v0 = cameraMatrix.at<double>(1,2);

    for(auto & p : dst) {
      p.x = fx * p.x + u0;
      p.y = fy * p.y + v0;
    }

  }

  template <class T>
  inline void undistort(const T & src, T & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {

    undistortNorm(src, dst, cameraMatrix, distortionCoefficients);

    dst.x = cameraMatrix.at<double>(0,0) * dst.x + cameraMatrix.at<double>(0,2);
    dst.y = cameraMatrix.at<double>(1,1) * dst.y + cameraMatrix.at<double>(1,2);

  }

  template <class T>
  inline void undistort(std::vector<T> & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }

  template <class T>
  inline void undistort(T & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    undistort(src, src, cameraMatrix, distortionCoefficients);
  }


  //****************************************************************************/
  // undistort - image undistortion
  //****************************************************************************/
  inline void undistort(const cv::Mat & src, cv::Mat & dst, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    cv::undistort(src, dst, cameraMatrix, distortionCoefficients);
  }

  inline void undistort(cv::Mat & src, const cv::Mat & cameraMatrix, const std::vector<double> & distortionCoefficients) {
    cv::Mat dst; cv::undistort(src, dst, cameraMatrix, distortionCoefficients); src = dst;
  }

} // namespace mpl::vision

#endif // _H_MPL_VISION_UNDISTORT_H_
