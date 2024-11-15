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


#ifndef _H_MPL_VISION_NORMALIZATION_H_
#define _H_MPL_VISION_NORMALIZATION_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

#include <mpl/mat.hpp>

//*****************************************************************************/
// namespace mpl::vision::normalization
//*****************************************************************************/
namespace mpl::vision::normalization {
  
  //*****************************************************************************/
  // internalParameters()
  //*****************************************************************************/
  template <class T>
  mpl::Mat3 internalParameters(std::vector<T> & points, double u0, double v0, double omega) {

    cv::Point2d barycenter(u0,v0);

    // Sposto i punti rispetto al baricentro
    for(size_t i=0; i<points.size(); ++i) {
      points[i] -= barycenter; 
      points[i] /= omega;
    }

    mpl::Mat3 H;

    //H(0,0) = 1; H(1,1) = 1; H(2,2) = a; H(0,2) = -barycenter.x; H(1,2) = -barycenter.y;
    H(0,0) = 1/omega; H(1,1) = 1/omega; H(2,2) = 1; H(0,2) = -u0/omega; H(1,2) = -v0/omega;

    return mpl::Mat3(H.inv());

  }


  //*****************************************************************************/
  // isotropic - see Multiple View Geometri pp. 107
  //*****************************************************************************/
  template <class T>
  mpl::Mat3 isotropic(std::vector<T> & points, double * _a = NULL) {

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
    double a = sum / ((points.size()-1) * sqrt(2.0));
    
    if(_a != NULL) *_a = a;

    // Scalo tutti i punti
    for(size_t i=0; i<points.size(); ++i)
       points[i] /= a;

    mpl::Mat3 H;

    //H(0,0) = 1; H(1,1) = 1; H(2,2) = a; H(0,2) = -barycenter.x; H(1,2) = -barycenter.y;
    H(0,0) = 1/a; H(1,1) = 1/a; H(2,2) = 1; H(0,2) = -barycenter.x/a; H(1,2) = -barycenter.y/a;

    return mpl::Mat3(H.inv());

  }

  
  //*****************************************************************************/
  // isotropic - see Multiple View Geometri pp. 107
  //*****************************************************************************/
  template <class Tp, typename Tcp>
  mpl::Mat3 isotropic(const std::vector<Tp> & points, std::vector<cv::Point3_<Tcp>> & pointsNorm) {

    // Alloco lo spazio
    pointsNorm.resize(points.size());
    
    cv::Point2d barycenter(0,0);

    // Mi calcolo il baricentro dei punti
    for(size_t i=0; i<points.size(); ++i)
      barycenter += points[i];

    barycenter /= (double) points.size();
    
    // Sposto i punti rispetto al baricentro
    for(size_t i=0; i<points.size(); ++i) {
      pointsNorm[i].x = points[i].x - barycenter.x;
      pointsNorm[i].y = points[i].y - barycenter.y;
      pointsNorm[i].z = 1.0;
    }

    // Mi calcolo la somma di sqrt(x^2 + y^2) per tutti i punti
    double sum = 0.0;

    for(size_t i=0; i<points.size(); ++i)
      sum += sqrt((pointsNorm[i].x*pointsNorm[i].x)+(pointsNorm[i].y*pointsNorm[i].y));
    
    // mi calcolo il fattore di scala
    double a = sum / (points.size() * sqrt(2.0));

    // Scalo tutti i punti
    for(size_t i=0; i<points.size(); ++i) {
      pointsNorm[i].x /= a;
      pointsNorm[i].y /= a;
    }
    
    mpl::Mat3 H;

    //H(0,0) = 1; H(1,1) = 1; H(2,2) = a; H(0,2) = -barycenter.x; H(1,2) = -barycenter.y;
    H(0,0) = 1/a; H(1,1) = 1/a; H(2,2) = 1; H(0,2) = -barycenter.x/a; H(1,2) = -barycenter.y/a;
    
    return mpl::Mat3(H.inv());

  }


  //*****************************************************************************/
  // onBarycenter()
  //*****************************************************************************/
  template <class T>
  void onBarycenter(std::vector<T> & points) {

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
