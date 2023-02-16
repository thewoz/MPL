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

#ifndef _H_MPL_CUBICSPLINE_H_
#define _H_MPL_CUBICSPLINE_H_

#include <cstdlib>
#include <cmath>

#include <vector>
#include <array>

#include <algorithm>

#include <opencv2/opencv.hpp>

/*****************************************************************************/
// namespace mpl::interpolation::cspline
/*****************************************************************************/
namespace mpl::interpolation::cubicspline {
  
  /*****************************************************************************/
  // findCoefficients
  /*****************************************************************************/
  // Generate the polynomials coefficients of the cubic spline for the given points
  /*****************************************************************************/
  template <typename T>
  std::vector<double> fit(const std::vector<T> & x, const std::vector<T> & y) {
    
    int size = (int)x.size();
    
    // valori sulle x
    cv::Mat X(size, 1, CV_64F);
   
    cv::Mat A(size, 4, CV_64F);

    for(int i=0; i<size; ++i){
            
      X.at<double>(i,0) = y[i];
      
      A.at<double>(i,0) = 1; A.at<double>(i,1) = x[i]; A.at<double>(i,2) = x[i]*x[i]; A.at<double>(i,3) = x[i]*x[i]*x[i];
      
    }

    
    const cv::Mat At = A.t();
    
    const cv::Mat invAtA = (At * A).inv();

    cv::Mat coeff = (At*X).t() * invAtA;
    
//    std::cout << X << std::endl;
//    std::cout << A << std::endl;
//    std::cout << At << std::endl;
//    std::cout << invAtA << std::endl;
//    std::cout << coeff << std::endl;

    std::vector<double> coefficients(4);
    
    coefficients[0] = coeff.at<double>(0,0);
    coefficients[1] = coeff.at<double>(0,1);
    coefficients[2] = coeff.at<double>(0,2);
    coefficients[3] = coeff.at<double>(0,3);

    //printf("%f %f %f %f\n", coefficients[0], coefficients[1], coefficients[2], coefficients[3]);
    
    //printf("\n\n\n");

    //exit(1);
    
    return coefficients;
    
  }
  
  
  /*****************************************************************************/
  // interpolate
  /*****************************************************************************/
  double interpolate(double x, const std::vector<double> & coefficients) {
    
    // mi calcolo la y
    return coefficients[0] + (coefficients[1] * x) + (coefficients[2] * x * x) + (coefficients[3] * x * x * x);
    
  }

} /* namespace cspline */


#endif /* _H_MPL_CUBICSPLINE_H_ */
