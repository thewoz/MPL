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

#ifndef _H_MPL_UTILS_H_
#define _H_MPL_UTILS_H_

#include <cstdlib>
#include <cstdio>

#include <random>

#include <opencv2/opencv.hpp>

#include <mpl/math.hpp>

//*****************************************************************************/
// namespace utils
//*****************************************************************************/
namespace mpl::utils {

  //*****************************************************************************/
  //  average
  //*****************************************************************************/
  template <typename T>
  double avg(T & set, double * stddev = NULL){
    
    double avg = 0;
    double temp = 0;
    
    for(auto value=set.begin(); value!=set.end(); value++){
      avg += *value;
      temp += (*value) * (*value);
    }
    
    avg /= (double)set.size();
    
    if(stddev!=NULL){
      
      *stddev = sqrt((temp - (set.size() * (avg*avg))) / (double)(set.size() - 1));
      
    }
    
    return avg;
    
  }

  
  //*****************************************************************************/
  //  median
  //*****************************************************************************/
  template <typename T>
  double median(const T & _set, double * stddev = NULL){
    
    T set = _set;
    
    std::size_t halfSize = set.size() * 0.5;
    
    //std::nth_element(minDist.begin(), minDist.begin()+halfSize+2, minDist.end());
    
    std::sort(set.begin(), set.end());
    
    double median = 0;
        
    if((set.size() % 2) != 0) median = set[halfSize];
    //else median = pow((sqrt(minDist[halfSize-1]) + sqrt(minDist[halfSize])) * 0.5, 2);
    else median = (set[halfSize-1] + set[halfSize]) * 0.5;
    
 		if(stddev!=NULL){
      
      double sum = 0;
      
      for(auto value=set.begin(); value!=set.end(); value++)
      	sum += (*value - median) * (*value - median);
      
      *stddev = sqrt(sum / (double)(set.size()-1));
      
    }
    
    return median;
    
  }
  

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
  double distanceFormBaricenter(const T & data, Op op = defaultDistanceOp) {

    T baricenter = T(0);

    for(size_t i=0; i<data.size(); ++i)
        baricenter += data[i];

    baricenter /= (double) data.size();

    double distance = 0;

    for(size_t i=0; i<data.size(); ++i)
        distance += op(data[i], baricenter);

    return distance / double(data.size());

  }
  

  //*****************************************************************************/
  //  NNDistance() - Minima distanza mediana tra i punti
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
        
    return median(minDist);
    
  }
  
  
  //*****************************************************************************/
  //  NNDistance() - Minima distanza mediana tra i punti
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
        
    return median(minDist);
    
  }

  
} /* namespace utils */


#endif /* _H_MPL_UTILS_H_ */


