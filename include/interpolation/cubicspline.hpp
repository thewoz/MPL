/*
 * MIT License
 *
 * Copyright © 2017
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

#ifndef _H_MPL_CUBICSPLINE_H_
#define _H_MPL_CUBICSPLINE_H_

#include <cstdlib>
#include <cmath>

#include <vector>
#include <array>

#include <algorithm>

/*****************************************************************************/
// namespace mpl::interpolation::cspline
/*****************************************************************************/
namespace mpl::interpolation::cspline {
  
  /*****************************************************************************/
  // findCoefficients
  /*****************************************************************************/
  // Generate the polynomials coefficients of the cubic spline for the given points
  /*****************************************************************************/
  template <typename T>
  std::vector<double> findCoefficients(const std::vector<T> & x, const std::vector<T> & y) {
    
    std::size_t size = x.size();
    
    // valori sulle x
    cv::Mat X(size, 1, CV_64F);
   
    cv::Mat A(size, 4, CV_64F);

    for(std::size_t i=0; i<size; ++i){
      
      X.at<double>(i,0) = x[i];
      
      A.at<double>(i,0) = 1; A.at<double>(i,1) = y[i]; A.at<double>(i,2) = y[i]*y[i]; A.at<double>(i,3) = y[i]*y[i]*y[i];
      
    }

    const cv::Mat At = A.t();
    
    const cv::Mat invAtA = (At * A).inv();

    cv::Mat coeff = (At*X).t() * invAtA;
    
    std::vector<double> coefficients(4);
    
    coefficients[0] = coeff.at<double>(0,0);
    coefficients[1] = coeff.at<double>(0,1);
    coefficients[2] = coeff.at<double>(0,2);
    coefficients[3] = coeff.at<double>(0,3);

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
