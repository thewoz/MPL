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

#ifndef _H_MPL_VISION_REPROJECTION_H_
#define _H_MPL_VISION_REPROJECTION_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>
#include <mpl/vision/point4d.hpp>


//*****************************************************************************/
// namespace mpl::vision
//*****************************************************************************/
namespace mpl::vision {

  //*****************************************************************************/
  // reproject - 3D point (implicit w = 1)
  //*****************************************************************************/
  template <typename T>
  inline void reproject(const cv::Point3_<T> & point3D, cv::Point_<T> & point2D, const double * P) {

    double w = P[8] * point3D.x + P[9] * point3D.y + P[10] * point3D.z + P[11];

    point2D.x = (P[0] * point3D.x + P[1] * point3D.y + P[2] * point3D.z + P[3]) / w;
    point2D.y = (P[4] * point3D.x + P[5] * point3D.y + P[6] * point3D.z + P[7]) / w;

  }

  //*****************************************************************************/
  // reproject - 4D homogeneous point
  //*****************************************************************************/
  template <typename T>
  inline void reproject(const point4d_t & point4D, cv::Point_<T> & point2D, const double * P) {

    double w = P[8] * point4D.x + P[9] * point4D.y + P[10] * point4D.z + P[11] * point4D.w;

    point2D.x = (P[0] * point4D.x + P[1] * point4D.y + P[2] * point4D.z + P[3] * point4D.w) / w;
    point2D.y = (P[4] * point4D.x + P[5] * point4D.y + P[6] * point4D.z + P[7] * point4D.w) / w;

  }

  //*****************************************************************************/
  // reproject - cv::Mat projection matrix overloads (point2D by reference)
  //*****************************************************************************/
  template <typename T, typename P3D>
  inline void reproject(const P3D & point, cv::Point_<T> & point2D, const cv::Mat & projectionMatrix) {
    reproject(point, point2D, (double *)projectionMatrix.data);
  }

  //*****************************************************************************/
  // reproject - value-returning overloads
  //*****************************************************************************/
  template <typename T>
  inline cv::Point_<T> reproject(const cv::Point3_<T> & point3D, const double * P) {
    cv::Point_<T> point2D; reproject(point3D, point2D, P); return point2D;
  }

  template <typename T>
  inline cv::Point_<T> reproject(const cv::Point3_<T> & point3D, const cv::Mat & projectionMatrix) {
    return reproject(point3D, (double *)projectionMatrix.data);
  }

  template <typename T>
  inline cv::Point_<T> reproject(const point4d_t & point4D, const double * P) {
    cv::Point_<T> point2D; reproject(point4D, point2D, P); return point2D;
  }

  inline cv::Point2d reproject(const point4d_t & point4D, const cv::Mat & projectionMatrix) {
    cv::Point2d point2D; reproject(point4D, point2D, (double *)projectionMatrix.data); return point2D;
  }

  //*****************************************************************************/
  // reproject - vector of 3D points
  //*****************************************************************************/
  template <typename T>
  inline void reproject(const std::vector<cv::Point3_<T>> & point3D, std::vector<cv::Point_<T>> & points2D, const cv::Mat & projectionMatrix) {

    points2D.resize(point3D.size());

    for(size_t i=0; i<point3D.size(); ++i)
      reproject(point3D[i], points2D[i], (double *)projectionMatrix.data);

  }

  template <typename T>
  inline std::vector<cv::Point_<T>> reproject(const std::vector<cv::Point3_<T>> & point3D, const cv::Mat & projectionMatrix) {
    std::vector<cv::Point_<T>> points2D; reproject(point3D, points2D, projectionMatrix); return points2D;
  }

} // namespace mpl::vision

#endif // _H_MPL_VISION_REPROJECTION_H_
