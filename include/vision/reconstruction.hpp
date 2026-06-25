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


#ifndef _H_MPL_VISION_RECONSTRUCTION_H_
#define _H_MPL_VISION_RECONSTRUCTION_H_

#include <cstdlib>
#include <cstdio>

#include <vector>
#include <limits>

#include <opencv2/opencv.hpp>

#include <mpl/math/math.hpp>
#include <mpl/vision/reprojection.hpp>
#include <mpl/vision/point4d.hpp>

//*****************************************************************************/
// namespace mpl::vision
//*****************************************************************************/
namespace mpl::vision {

  //*****************************************************************************/
  // namespace detail
  //*****************************************************************************/
  namespace detail {

    //*****************************************************************************/
    // _recoSVD - 3 views, homogeneous 4D output
    //*****************************************************************************/
    template <typename T2D>
    double _recoSVD(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, point4d_t & point4D) {

      cv::Mat A = cv::Mat(6, 4, CV_64F);

      for(int j=0; j<4; ++j) {

        A.at<double>(0, j) = pt1.y * prjMat1[8 + j] -         prjMat1[4 + j];
        A.at<double>(1, j) =         prjMat1[j]     - pt1.x * prjMat1[8 + j];

        A.at<double>(2, j) = pt2.y * prjMat2[8 + j] -         prjMat2[4 + j];
        A.at<double>(3, j) =         prjMat2[j]     - pt2.x * prjMat2[8 + j];

        A.at<double>(4, j) = pt3.y * prjMat3[8 + j] -         prjMat3[4 + j];
        A.at<double>(5, j) =         prjMat3[j]     - pt3.x * prjMat3[8 + j];

      }

      cv::Mat W,U,V;

      cv::Mat AA = A.t() * A;

      mpl::svd(AA, W, U, V, cv::SVD::MODIFY_A);

      point4D.x = V.at<double>(3,0);
      point4D.y = V.at<double>(3,1);
      point4D.z = V.at<double>(3,2);
      point4D.w = V.at<double>(3,3);

      return W.at<double>(3);

    }

    //*****************************************************************************/
    // _recoSVD - 2 views, homogeneous 4D output
    //*****************************************************************************/
    template <typename T2D>
    void _recoSVD(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, point4d_t & point4D) {

      cv::Mat A = cv::Mat(4, 4, CV_64F);

      for(int j=0; j<4; ++j) {

        A.at<double>(0, j) = pt1.y * prjMat1[8 + j] -         prjMat1[4 + j];
        A.at<double>(1, j) =         prjMat1[j]     - pt1.x * prjMat1[8 + j];

        A.at<double>(2, j) = pt2.y * prjMat2[8 + j] -         prjMat2[4 + j];
        A.at<double>(3, j) =         prjMat2[j]     - pt2.x * prjMat2[8 + j];

      }

      cv::Mat W,U,V;

      cv::Mat AA = A.t() * A;

      mpl::svd(AA, W, U, V, cv::SVD::MODIFY_A);

      point4D.x = V.at<double>(3,0);
      point4D.y = V.at<double>(3,1);
      point4D.z = V.at<double>(3,2);
      point4D.w = V.at<double>(3,3);

    }

    //*****************************************************************************/
    // _recoSVD - 3 views, Cartesian 3D output
    //*****************************************************************************/
    template <typename T2D, typename T3D>
    void _recoSVD(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, T3D & point3D) {

      point4d_t point4D;

      _recoSVD(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point4D);

      point3D.x = point4D.x / point4D.w;
      point3D.y = point4D.y / point4D.w;
      point3D.z = point4D.z / point4D.w;

    }

    //*****************************************************************************/
    // _recoSVD - 2 views, Cartesian 3D output
    //*****************************************************************************/
    template <typename T2D, typename T3D>
    void _recoSVD(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, T3D & point3D) {

      point4d_t point4D;

      _recoSVD(pt1, prjMat1, pt2, prjMat2, point4D);

      point3D.x = point4D.x / point4D.w;
      point3D.y = point4D.y / point4D.w;
      point3D.z = point4D.z / point4D.w;

    }

  } // namespace detail


  //*****************************************************************************/
  // reconstruct - single correspondence, raw double* projection matrices
  //*****************************************************************************/
  template <typename T2D, typename T3D>
  void reconstruct(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, T3D & point3D) {
    detail::_recoSVD(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point3D);
  }

  template <typename T2D, typename T3D>
  void reconstruct(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, T3D & point3D) {
    detail::_recoSVD(pt1, prjMat1, pt2, prjMat2, point3D);
  }

  //*****************************************************************************/
  // reconstruct - single correspondence, cv::Mat projection matrices
  //*****************************************************************************/
  template <typename T2D, typename T3D>
  inline void reconstruct(const T2D & pt1, const cv::Mat & prjMat1, const T2D & pt2, const cv::Mat & prjMat2, const T2D & pt3, const cv::Mat & prjMat3, T3D & point3D) {
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, pt3, (double*)prjMat3.data, point3D);
  }

  template <typename T2D, typename T3D>
  inline void reconstruct(const T2D & pt1, const cv::Mat & prjMat1, const T2D & pt2, const cv::Mat & prjMat2, T3D & point3D) {
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, point3D);
  }

  //*****************************************************************************/
  // reconstruct - vector of correspondences, raw double* projection matrices
  //*****************************************************************************/
  template <typename T2D, typename T3D>
  void reconstruct(const std::vector<T2D> & pt1, const double * prjMat1, const std::vector<T2D> & pt2, const double * prjMat2, const std::vector<T2D> & pt3, const double * prjMat3, std::vector<T3D> & point3D) {

    if(!(pt1.size() == pt2.size() && pt2.size() == pt3.size())) {
      fprintf(stderr, "mpl::vision::reconstruct() error: the point sets must have the same size\n");
      abort();
    }

    point3D.resize(pt1.size());

    for(std::size_t i=0; i<point3D.size(); ++i)
      detail::_recoSVD(pt1[i], prjMat1, pt2[i], prjMat2, pt3[i], prjMat3, point3D[i]);

  }

  template <typename T2D, typename T3D>
  void reconstruct(const std::vector<T2D> & pt1, const double * prjMat1, const std::vector<T2D> & pt2, const double * prjMat2, std::vector<T3D> & point3D) {

    if(!(pt1.size() == pt2.size())) {
      fprintf(stderr, "mpl::vision::reconstruct() error: the point sets must have the same size\n");
      abort();
    }

    point3D.resize(pt1.size());

    for(std::size_t i=0; i<point3D.size(); ++i)
      detail::_recoSVD(pt1[i], prjMat1, pt2[i], prjMat2, point3D[i]);

  }

  //*****************************************************************************/
  // reconstruct - vector of correspondences, cv::Mat projection matrices
  //*****************************************************************************/
  template <typename T2D, typename T3D>
  void reconstruct(const std::vector<T2D> & pt1, const cv::Mat & prjMat1, const std::vector<T2D> & pt2, const cv::Mat & prjMat2, const std::vector<T2D> & pt3, const cv::Mat & prjMat3, std::vector<T3D> & point3D) {
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, pt3, (double*)prjMat3.data, point3D);
  }

  template <typename T2D, typename T3D>
  void reconstruct(const std::vector<T2D> & pt1, const cv::Mat & prjMat1, const std::vector<T2D> & pt2, const cv::Mat & prjMat2, std::vector<T3D> & point3D) {
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, point3D);
  }


  //*****************************************************************************/
  // reconstructionError - 3 views, reprojection error, raw double* projection matrices
  //*****************************************************************************/
  // maxError: optional early-out threshold. As soon as a single-view
  // reprojection error exceeds it the function bails out returning +inf,
  // which lets callers (e.g. cameraSystem::findMatches) prune cheaply.
  // With the default (+inf) the full average is always computed.
  template <typename T2D>
  double reconstructionError(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, double maxError = std::numeric_limits<double>::max()) {

    cv::Point3d point3D;

    detail::_recoSVD(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point3D);

    cv::Point2d _pt1; reproject(point3D, _pt1, prjMat1); double error1 = cv::norm(cv::Point2d(pt1.x, pt1.y) - _pt1);
    if(error1 > maxError) return std::numeric_limits<double>::max();

    cv::Point2d _pt2; reproject(point3D, _pt2, prjMat2); double error2 = cv::norm(cv::Point2d(pt2.x, pt2.y) - _pt2);
    if(error2 > maxError) return std::numeric_limits<double>::max();

    cv::Point2d _pt3; reproject(point3D, _pt3, prjMat3); double error3 = cv::norm(cv::Point2d(pt3.x, pt3.y) - _pt3);
    if(error3 > maxError) return std::numeric_limits<double>::max();

    return (error1 + error2 + error3) / 3.0;

  }

  //*****************************************************************************/
  // reconstructionError - 2 views, reprojection error, raw double* projection matrices
  //*****************************************************************************/
  template <typename T2D>
  double reconstructionError(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, double maxError = std::numeric_limits<double>::max()) {

    cv::Point3d point3D;

    detail::_recoSVD(pt1, prjMat1, pt2, prjMat2, point3D);

    cv::Point2d _pt1; reproject(point3D, _pt1, prjMat1); double error1 = cv::norm(cv::Point2d(pt1.x, pt1.y) - _pt1);
    if(error1 > maxError) return std::numeric_limits<double>::max();

    cv::Point2d _pt2; reproject(point3D, _pt2, prjMat2); double error2 = cv::norm(cv::Point2d(pt2.x, pt2.y) - _pt2);
    if(error2 > maxError) return std::numeric_limits<double>::max();

    return (error1 + error2) / 2.0;

  }

  //*****************************************************************************/
  // reconstructionError - cv::Mat projection matrices
  //*****************************************************************************/
  template <typename T2D>
  inline double reconstructionError(const T2D & pt1, const cv::Mat prjMat1, const T2D & pt2, const cv::Mat prjMat2, const T2D & pt3, const cv::Mat prjMat3, double maxError = std::numeric_limits<double>::max()) {
    return reconstructionError(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, pt3, (double*)prjMat3.data, maxError);
  }

  template <typename T2D>
  inline double reconstructionError(const T2D & pt1, const cv::Mat prjMat1, const T2D & pt2, const cv::Mat prjMat2, double maxError = std::numeric_limits<double>::max()) {
    return reconstructionError(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, maxError);
  }


} // namespace mpl::vision

#endif // _H_MPL_VISION_RECONSTRUCTION_H_
