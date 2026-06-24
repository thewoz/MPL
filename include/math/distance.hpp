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

#ifndef _H_MPL_MATH_DISTANCE_H_
#define _H_MPL_MATH_DISTANCE_H_

#include <cstdlib>
#include <cstdio>

#include <random>

#include <opencv2/opencv.hpp>

#include <mpl/math/math.hpp>
#include <mpl/math/stat.hpp>
#include <mpl/vision/point4d.hpp>

//*****************************************************************************/
// namespace mpl::distance
//*****************************************************************************/
namespace mpl::distance {

  //*****************************************************************************/
  //  defaultDistanceOp()
  //*****************************************************************************/
  constexpr auto defaultDistanceOp = [](auto const & a, auto const & b) { return cv::norm(a - b); };

  
  //*****************************************************************************/
  //  distanceAvg() - distanza media tra i punti
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultDistanceOp)>
  double distanceAvg(const T & data, Op op = defaultDistanceOp) {

    double distance = 0;

    for(size_t i=0; i<data.size(); ++i) {

      for(size_t j=i+1; j<data.size(); ++j) {

        distance += op(data[i], data[j]);

      }
    
    }

    return distance / double(data.size());

  }
  
  //*****************************************************************************/
  //  distance() - distanza minima tra un punto e un set di punti
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultDistanceOp)>
  double distance(const T & point, const T & data, Op op = defaultDistanceOp) {
    
    double distance = DBL_MAX;
    
    for(size_t i=0; i<data.size(); ++i) {
              
      double tmpDistance = op(point, data[i]);
              
      if(tmpDistance < distance) distance = tmpDistance;
      
    }
    
    return distance;
    
  }
  
  //*****************************************************************************/
  //  distance() - distanza minima tra i punti
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultDistanceOp)>
  double distanceMin(const T & data, Op op = defaultDistanceOp) {

    double min = DBL_MAX;

    for(size_t i=0; i<data.size(); ++i) {

      for(size_t j=i+1; j<data.size(); ++j) {

       double distance = op(data[i], data[j]);
      
        if(distance < min) min = distance;

      }
    
    }

    return min;

  }


  //*****************************************************************************/
  //  distance() - distanza dal baricentro tra i punti
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultDistanceOp)>
  double distanceFromBarycenter(const T & data, Op op = defaultDistanceOp) {

    using P = typename T::value_type;   // tipo del punto (non del container)

    P barycenter = P();                 // punto a zero (es. cv::Point_ -> (0,0))

    for(size_t i=0; i<data.size(); ++i)
        barycenter += data[i];

    barycenter /= (double) data.size();

    double distance = 0;

    for(size_t i=0; i<data.size(); ++i)
        distance += op(data[i], barycenter);

    return distance / double(data.size());

  }
  

  //*****************************************************************************/
  //  medianFirstNNDistance() - Minima distanza mediana tra i punti
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultDistanceOp)>
  double medianFirstNNDistance(const T & data1, const T & data2, Op op = defaultDistanceOp) {
    
    // firstNN per ogni punto
    std::vector<float> minDist(data1.size(), FLT_MAX);
    
    //#pragma omp parallel for num_threads(2) ordered schedule(static)
    for(std::size_t i=0; i<data1.size(); ++i){
      
      for(std::size_t j=0; j<data2.size(); ++j){

        double dist = op(data1[i], data2[j]);
        
        if(dist < minDist[i]) minDist[i] = dist;

      }
      
    }
        
    return mpl::statistic::median(minDist);
    
  }
  
  
  //*****************************************************************************/
  //  medianFirstNNDistance() - Minima distanza mediana tra i punti
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultDistanceOp)>
  double medianFirstNNDistance(const T & data, Op op = defaultDistanceOp) {
    
    // firstNN per ogni punto
    std::vector<float> minDist(data.size(), FLT_MAX);
    
    //#pragma omp parallel for num_threads(2) ordered schedule(static)
    for(std::size_t i=0; i<data.size(); ++i){
      
      for(std::size_t j=i+1; j<data.size(); ++j){
        
        double dist = op(data[i], data[j]);
        
        if(dist < minDist[i]) minDist[i] = dist;
        if(dist < minDist[j]) minDist[j] = dist;

      }
      
    }
        
    return mpl::statistic::median(minDist);

  }


  //****************************************************************************/
  // minDist() - distanza minima tra due contorni (con la coppia di punti piu' vicini)
  //****************************************************************************/
  template <class T>
  double minDist(const std::vector<T> & contourA, const std::vector<T> & contourB, T & pA, T & pB) {

    if(contourA.empty() || contourB.empty()) return DBL_MAX;

    double dMin = DBL_MAX;

    for(size_t i=0; i<contourA.size(); ++i) {

      for(size_t j=0; j<contourB.size(); ++j) {

        double dist = cv::norm(contourA[i] - contourB[j]);

        if(dist < dMin) {
          dMin = dist;
          pA = contourA[i];
          pB = contourB[j];
        }

      }

    }

    return dMin;

  }


  //****************************************************************************/
  //  fromLine | distanza tra un punto e una retta  ax + by + c = 0
  //****************************************************************************/
  template <typename TP, typename TL>
  inline double fromLine(const TP & point, const cv::Vec<TL, 3> & line){

    // forma chiusa punto-retta: |a*x0 + b*y0 + c| / sqrt(a^2 + b^2)
    double a = line[0], b = line[1], c = line[2];

    double den = (a*a) + (b*b);

    if(den <= FLT_EPSILON) return INFINITY;   // retta degenere (a = b = 0)

    return std::abs((a*point.x) + (b*point.y) + c) / std::sqrt(den);

  }

  //****************************************************************************/
  //  fromLine | distanza tra un punto e una retta  y = m*x + q
  //****************************************************************************/
  template <typename TP, typename TL>
  inline double fromLine(const TP & point, const cv::Vec<TL, 2> & line){

    return std::abs(point.y - ((line[0]*point.x) + line[1])) / std::sqrt(1+(line[0]*line[0]));

  }

  //*****************************************************************************/
  //  fromPlane | distanza tra un punto 4D e un piano
  //*****************************************************************************/
  template <typename TP, typename TL>
  inline double fromPlane(const mpl::vision::point4d_t & _point, const TL * coeff) {

    if(_point.w == 0) return 0;

    cv::Point3d point;

    point.x = _point.x / _point.w;
    point.y = _point.y / _point.w;
    point.z = _point.z / _point.w;

    double value = std::abs((coeff[0]*point.x)+(coeff[1]*point.y)+(coeff[2]*point.z)+coeff[3]) / std::sqrt((coeff[0]*coeff[0])+(coeff[1]*coeff[1])+(coeff[2]*coeff[2]));

    return value;

  }


} // namespace mpl::distance


#endif // _H_MPL_MATH_DISTANCE_H_


