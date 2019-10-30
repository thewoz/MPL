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


#ifndef _H_MPL_VISION_NORMALIZATION_H_
#define _H_MPL_VISION_NORMALIZATION_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

#include <mpl/mat.hpp>

/*****************************************************************************/
// namespace mpl::vision::normalization
/*****************************************************************************/
namespace mpl::vision::normalization {
  
  /*****************************************************************************/
  // isotropic - see Multiple View Geometri pp. 107
  /*****************************************************************************/
  mpl::Mat3 isotropic(std::vector<cv::Point2d> & points, double * _a = NULL) {

    cv::Point2d barycenter(0,0);

    // Mi calcolo il baricentro dei punti
    for(size_t i=0; i<points.size(); ++i)
      barycenter += points[i];

    barycenter /= (double) points.size();
    
    // Sposto i punti rispetto al baricentro
    for(size_t i=0; i<points.size(); ++i)
      points[i] -= barycenter;

    // Mi calcolo la somma di sqrt(x^2 + y^2) per tutti i punti
    double sum = 0.0;

    for(size_t i=0; i<points.size(); ++i)
      sum += sqrt((points[i].x*points[i].x)+(points[i].y*points[i].y));
    
    // mi calcolo il fattore di scala
    double a = sum / (points.size() * sqrt(2.0));
    
    if(_a != NULL) *_a = a;

    //printf("a %f\n", a);

    // Scalo tutti i punti
    for(size_t i=0; i<points.size(); ++i)
       points[i] /= a;

    mpl::Mat3 H;

    H(0,0) = 1; H(1,1) = 1; H(2,2) = a; H(0,2) = -barycenter.x; H(1,2) = -barycenter.y;

    return mpl::Mat3(H.inv());

  }



  /*****************************************************************************/
  // onBarycenter - 
  /*****************************************************************************/
  void onBarycenter(std::vector<cv::Point2d> & points) {

    cv::Point2d barycenter(0,0);

    // Mi calcolo il baricentro dei punti
    for(size_t i=0; i<points.size(); ++i)
      barycenter += points[i];

    barycenter /= (double) points.size();
    
    // Sposto i punti rispetto al baricentro
    for(size_t i=0; i<points.size(); ++i)
      points[i] -= barycenter;
   
  }

} /* mpl::vision::normalization */

#endif /* _H_MPL_VISION_NORMALIZATION_H_ */
