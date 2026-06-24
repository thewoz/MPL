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

#ifndef _H_MPL_MATH_FIT_H_
#define _H_MPL_MATH_FIT_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

//****************************************************************************/
// mpl::fit
//****************************************************************************/
namespace mpl::fit {


  //****************************************************************************/
  // parabola() - ChatGPT
  //****************************************************************************/
  template <class T>
  void parabola(const std::vector<T> & points, cv::Vec3d & coeffs, const cv::Point2d & offset = cv::Point2d(0,0)) {

      // Create matrices for the system of linear equations
      cv::Mat A((int)points.size(), 3, CV_64F);
      cv::Mat B((int)points.size(), 1, CV_64F);

      // Fill matrices A and B
      for(int i=0; i<points.size(); ++i) {
        double x = points[i].x - offset.x;
        double y = points[i].y - offset.y;
        A.at<double>(i, 0) = x * x;
        A.at<double>(i, 1) = x;
        A.at<double>(i, 2) = 1;
        B.at<double>(i, 0) = y;
      }

      // Solve the system A * coeffs = B
      cv::Mat coeffsMat;
      cv::solve(A, B, coeffsMat, cv::DECOMP_SVD);

      // Store the result in coeffs
      coeffs = cv::Vec3d(coeffsMat.at<double>(0, 0), coeffsMat.at<double>(1, 0), coeffsMat.at<double>(2, 0));

  }


  //****************************************************************************/
  // linear()
  //****************************************************************************/
  double linear(const std::vector<cv::Point2d> & points, cv::Vec2d & coeff) {

    double sumx  = 0.0;
    double sumx2 = 0.0;
    double sumy  = 0.0;
    double sumxy = 0.0;

    for(size_t i=0; i<points.size(); ++i) {
      sumx  += points[i].x;
      sumx2 += points[i].x * points[i].x;
      sumy  += points[i].y;
      sumxy += points[i].x * points[i].y;
    }

    // f(x) = ax + b
    coeff[0] = (points.size()*sumxy - sumx*sumy)  / (points.size()*sumx2 - sumx*sumx);
    coeff[1] = (sumy*sumx2 - sumx*sumxy) / (points.size()*sumx2 - sumx*sumx);

    double error = 0;

    for(int i=0; i<points.size(); ++i) {

      error += (points[i].y - ((coeff[0]*points[i].x) + coeff[1])) * (points[i].y - ((coeff[0]*points[i].x) + coeff[1]));

    }

    error /= (double)points.size();

    return error;

  }

  //****************************************************************************/
  // linear()
  //****************************************************************************/
  double linear(const std::vector<double> & x, const std::vector<double> & y, cv::Vec2d & coeff) {

    double sumx  = 0.0;
    double sumx2 = 0.0;
    double sumy  = 0.0;
    double sumxy = 0.0;

    if(x.size() != y.size()) {
      fprintf(stderr, "mpl::fit::linear() error: x and y must have the same length\n");
      abort();
    }

    size_t size = x.size();

    for(size_t i=0; i<size; ++i) {
      sumx  += x[i];
      sumx2 += x[i] * x[i];
      sumy  += y[i];
      sumxy += x[i] * y[i];
    }

    // f(x) = ax + b
    coeff[0] = (size*sumxy - sumx*sumy)  / (size*sumx2 - sumx*sumx);
    coeff[1] = (sumy*sumx2 - sumx*sumxy) / (size*sumx2 - sumx*sumx);

    double error = 0;

    for(int i=0; i<size; ++i) {

      error += (y[i] - ((coeff[0]*x[i]) + coeff[1])) * (y[i] - ((coeff[0]*x[i]) + coeff[1]));

    }

    error /= (double)size;

    return error;

  }

  //****************************************************************************/
  // orear()
  //****************************************************************************/
  double orear(const std::vector<cv::Point2d> & points, const std::vector<cv::Point2d> & sigma, cv::Vec2d & coeff, double & energy, int maxIteration = 10) {

    // Forse gli si puo dare una pulita
    double oldA = coeff[1];
    double oldB = coeff[0];

    double ao = coeff[1];
    double bo = coeff[0];

    for(int j=0; j<maxIteration; ++j) {

      double t1 = 0; double t2 = 0; double t3 = 0; double t4 = 0; double t5 = 0;

      for(int i=0; i<points.size(); ++i) {

        double wi = 1 / ((sigma[i].y*sigma[i].y) + ((bo*bo)*(sigma[i].x*sigma[i].x)));

        t1 += wi;
        t2 += wi * points[i].x * points[i].y;
        t3 += wi * points[i].x;
        t4 += wi * points[i].y;
        t5 += wi * points[i].x * points[i].x;

      }

      bo = (t1 * t2 - t3 * t4) / (t1 * t5 - t3 * t3);
      ao = (t4 - (bo * t3))  / t1;

      if(std::abs(oldA - ao) <= 0.0000001 && std::abs(oldB - bo) <= 0.0000001 ) {
        break;
      }

      coeff[1] = ao;
      coeff[0] = bo;

    }

    double error  = 0;
    energy = 0;

    for(int i=0; i<points.size(); ++i) {

      error += (points[i].y - ((coeff[0]*points[i].x) + coeff[1])) * (points[i].y - ((coeff[0]*points[i].x) + coeff[1]));

      double wi = 1 / ((sigma[i].y*sigma[i].y) + ((coeff[0]*coeff[0])*(sigma[i].x*sigma[i].x)));

      energy += wi * (points[i].y - ((coeff[0]*points[i].x) + coeff[1])) * (points[i].y - ((coeff[0]*points[i].x) + coeff[1]));
    }

    error /= (double)points.size();

    return error;

  }

} // namespace mpl::fit

#endif // _H_MPL_MATH_FIT_H_
