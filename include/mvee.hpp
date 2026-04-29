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
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 *
 * NOTE:
 * This file contains code derived from an implementation by Bojan Nikolic
 * related to Khachiyan's algorithm for the Minimum Volume Enclosing Ellipsoid.
 * Before redistributing, verify license compatibility between the original
 * GPL version and the license used by this project.
 */

#ifndef _H_MPL_MVEE_H_
#define _H_MPL_MVEE_H_

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <stdexcept>
#include <vector>

#include <opencv2/opencv.hpp>

//****************************************************************************/
// namespace mvee
//****************************************************************************/
namespace mvee {

  //****************************************************************************/
  // namespace util
  //****************************************************************************/
  namespace util {

    //****************************************************************************/
    // prepareInputPoints()
    //****************************************************************************/
    inline cv::Mat prepareInputPoints(const cv::Mat & points) {
      
      if(points.empty())
        throw std::invalid_argument("mvee::fit: input matrix is empty");
      
      if(points.channels() != 1)
        throw std::invalid_argument("mvee::fit: input matrix must be single-channel");
      
      if(points.rows <= 0 || points.cols <= 0)
        throw std::invalid_argument("mvee::fit: invalid input matrix size");

      if(points.cols <= points.rows)
        throw std::invalid_argument("mvee::fit: the number of points must be greater than the dimensionality");

      cv::Mat prepared;

      if(points.depth() != CV_64F) {
        points.convertTo(prepared, CV_64F);
      } else if(!points.isContinuous()) {
        prepared = points.clone();
      } else { prepared = points; }

      if(!cv::checkRange(prepared, true, nullptr))
        throw std::invalid_argument("mvee::fit: input matrix contains NaN or Inf");

      return prepared;
      
    }

    //****************************************************************************/
    // invertSymmetricMatrix()
    //****************************************************************************/
    inline cv::Mat invertSymmetricMatrix(const cv::Mat & matrix) {
      
      cv::Mat inverse;

      bool ok = cv::invert(matrix, inverse, cv::DECOMP_CHOLESKY);
      
      if(!ok)
        ok = cv::invert(matrix, inverse, cv::DECOMP_LU);

      if(!ok)
        ok = cv::invert(matrix, inverse, cv::DECOMP_SVD);
      
      if(!ok)
        throw std::runtime_error("mvee::fit: failed to invert covariance matrix");

      return inverse;
      
    }

    //****************************************************************************/
    // computeMVEE()
    //****************************************************************************/
    inline double computeMVEE(const cv::Mat & inputPoints, cv::Mat & A, cv::Mat & center, double eps, int maxIterations) {
      
      if(eps <= 0.0)
        throw std::invalid_argument("mvee::fit: eps must be positive");

      if(maxIterations <= 0)
        throw std::invalid_argument("mvee::fit: maxIterations must be positive");

      const cv::Mat P = prepareInputPoints(inputPoints);

      const int dims = P.rows;
      const int pointCount = P.cols;

      cv::Mat Q(dims + 1, pointCount, CV_64F);
      P.copyTo(Q(cv::Rect(0, 0, pointCount, dims)));
      Q.row(Q.rows - 1).setTo(1.0);

      cv::Mat weightedQ(dims + 1, pointCount, CV_64F);
      cv::Mat X(dims + 1, dims + 1, CV_64F);
      cv::Mat weights(pointCount, 1, CV_64F, cv::Scalar(1.0 / pointCount));

      std::vector<double> mahalanobis(pointCount, 0.0);

      double currentError = eps * 2.0;

      for(int iteration=0; iteration<maxIterations && currentError>eps; ++iteration) {
        
        const double * weightData = weights.ptr<double>(0);

        for(int row=0; row<dims+1; ++row) {
          
          const double * qRow = Q.ptr<double>(row);
          
          double * weightedRow = weightedQ.ptr<double>(row);

          for(int col=0; col<pointCount; ++col)
            weightedRow[col] = qRow[col] * weightData[col];
          
        }

        X.setTo(0.0);

        for(int xi=0; xi<dims+1; ++xi) {
          
          const double * weightedRow = weightedQ.ptr<double>(xi);
          
          double * xRow = X.ptr<double>(xi);

          for(int xj=0; xj<dims+1; ++xj) {
            
            const double * qRow = Q.ptr<double>(xj);
            
            double sum = 0.0;
            
            for(int k=0; k<pointCount; ++k)
              sum += weightedRow[k] * qRow[k];

            xRow[xj] = sum;
            
          }
          
        }

        cv::Mat solved;
        
        bool solvedOk = cv::solve(X, Q, solved, cv::DECOMP_CHOLESKY);
        
        if(!solvedOk)
          solvedOk = cv::solve(X, Q, solved, cv::DECOMP_LU);

        if(!solvedOk)
          solvedOk = cv::solve(X, Q, solved, cv::DECOMP_SVD);
        
        if(!solvedOk)
          throw std::runtime_error("mvee::fit: failed to solve the Khachiyan linear system");

        std::fill(mahalanobis.begin(), mahalanobis.end(), 0.0);

        for(int row=0; row <dims+1; ++row) {
          
          const double * qRow = Q.ptr<double>(row);
          const double * solvedRow = solved.ptr<double>(row);

          for(int col = 0; col < pointCount; ++col)
            mahalanobis[col] += qRow[col] * solvedRow[col];

        }

        double maximum = -DBL_MAX;
        int maximumIndex = 0;

        for(int col=0; col<pointCount; ++col) {
          if(mahalanobis[col] > maximum) {
            maximum = mahalanobis[col];
            maximumIndex = col;
          }
        }

        if(!std::isfinite(maximum))
          throw std::runtime_error("mvee::fit: invalid maximum Mahalanobis value");

        const double denominator = (dims + 1.0) * (maximum - 1.0);
        if(std::abs(denominator) <= DBL_EPSILON)
          break;

        const double stepSize = (maximum - dims - 1.0) / denominator;
        const double scale = 1.0 - stepSize;

        double squaredNorm = 0.0;
        double * weightMutableData = weights.ptr<double>(0);

        for(int col=0; col<pointCount; ++col) {
          
          double newWeight = weightMutableData[col] * scale;

          if(col == maximumIndex)
            newWeight += stepSize;

          const double diff = weightMutableData[col] - newWeight;
          squaredNorm += diff * diff;
          weightMutableData[col] = newWeight;
          
        }

        currentError = std::sqrt(squaredNorm);
        
      }

      cv::Mat weightedP(dims, pointCount, CV_64F);
      const double * weightData = weights.ptr<double>(0);

      for(int row=0; row<dims; ++row) {
        
        const double * pRow = P.ptr<double>(row);
        double * weightedRow = weightedP.ptr<double>(row);

        for(int col=0; col<pointCount; ++col)
          weightedRow[col] = pRow[col] * weightData[col];
        
      }

      center = P * weights;

      const cv::Mat covarianceLike = weightedP * P.t() - center * center.t();
      
      A = (1.0 / dims) * invertSymmetricMatrix(covarianceLike);

      return currentError;
      
    }

  } // namespace util

  //****************************************************************************/
  // fit()
  //****************************************************************************/
  inline double fit(const cv::Mat & points, cv::Mat & center, cv::Mat & rotation, cv::Mat & radius, double eps = 1.0e-3, int maxIterations = 1000) {
    
    cv::Mat A;
    const double finalError = util::computeMVEE(points, A, center, eps, maxIterations);

    cv::Mat eigenValues;
    cv::Mat eigenVectors;

    const bool ok = cv::eigen(A, eigenValues, eigenVectors);
   
    if(!ok) throw std::runtime_error("mvee::fit: failed to compute ellipsoid eigen decomposition");

    // cv::eigen returns eigenvectors as rows.
    // Transpose them so that rotation columns are the ellipsoid axes.
    rotation = eigenVectors.t();
    radius = cv::Mat(points.rows, 1, CV_64F);

    for(int i=0; i<points.rows; ++i) {
      
      const double lambda = eigenValues.at<double>(i, 0);

      if(lambda <= 0.0) throw std::runtime_error("mvee::fit: non-positive eigenvalue found");

      radius.at<double>(i, 0) = 1.0 / std::sqrt(lambda);
      
    }

    return finalError;
    
  }

} // namespace mvee

#endif // _H_MPL_MVEE_H_
