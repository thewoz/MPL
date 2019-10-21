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

#ifndef _H_MPL_UTILS_H_
#define _H_MPL_UTILS_H_

#include <cstdlib>
#include <cstdio>

#include <random>

#include <opencv2/opencv.hpp>

/*****************************************************************************/
// namespace utils
/*****************************************************************************/
namespace mpl::utils {
  
  /*****************************************************************************/
  //  Mean
  /*****************************************************************************/
  template <typename T>
  double mean(T & set, double * var = NULL){
    
    double mean = 0;
    double temp = 0;
    
    for(auto value=set.begin(); value!=set.end(); value++){
      mean += *value;
      temp += (*value) * (*value);
    }
    
    mean /= (double)set.size();
    
    if(var!=NULL){
      
      *var = sqrt((temp - (set.size() * (mean*mean))) / (double)(set.size() - 1));
      
    }
    
    return mean;
    
  }
  
  
  /*****************************************************************************/
  //  median
  /*****************************************************************************/
  template <typename T>
  double median(const T & _set, double * var = NULL){
    
    T set = _set;
    
    std::size_t halfSize = set.size() * 0.5;
    
    //std::nth_element(minDist.begin(), minDist.begin()+halfSize+2, minDist.end());
    
    std::sort(set.begin(), set.end());
    
    double median = 0;
    
    //printf("set.size(%u) halfSize %d %p\n", set.size(), halfSize, var);  fflush(stdout);
    
    if((set.size() % 2) != 0) median = set[halfSize];
    //else median = pow((sqrt(minDist[halfSize-1]) + sqrt(minDist[halfSize])) * 0.5, 2);
    else median = (set[halfSize-1] + set[halfSize]) * 0.5;
    
    //printf("MERDA\n"); fflush(stdout);

 		if(var!=NULL){
      
      double sum = 0;
      
      for(auto value=set.begin(); value!=set.end(); value++)
      	sum += (*value - median) * (*value - median);
      
      *var = sqrt(sum / (double)(set.size()-1));
      
    }

    //printf("median %e\n", median);
    
    return median;
    
  }

  
//  /*****************************************************************************/
//  //  meanVarMedian
//  /*****************************************************************************/
//  template <typename T>
//  void meanVarMedian(T & set, double & mean, double & median, double & var){
//
//    mean = 0;
//
//    for(auto value=set.begin(); value!=set.end(); value++)
//      mean += *value;
//
//    mean /= (double)set.size();
//
//    std::vector<double> dist(set.size());
//
//    std::size_t i = 0;
//
//    for(auto value=set.begin(); value!=set.end(); value++, ++i)
//      dist[i] = fabs(_mean - (*value));
//
//    median = median(dist, var);
//
//  }
//
  
  /*****************************************************************************/
  //  NNDistance() - Minima distanza mediana (subsampled) tra i punti
  /*****************************************************************************/
  template <typename T>
  double distance(const std::vector<T> & set){

    double distance = 0;

    for(size_t i=0; i<set.size(); ++i) {

      for(size_t j=i+1; j<set.size(); ++j) {

        distance += mpl::norm(set[i], set[j]);

      }
    
    }

    return distance / double(set.size());

  }
  

  /*****************************************************************************/
  //  NNDistance() - Minima distanza mediana (subsampled) tra i punti
  /*****************************************************************************/
  template <typename T>
  double distanceFormBaricenter(const std::vector<T> & set) {

    T baricenter = T(0);

    for(size_t i=0; i<set.size(); ++i)
        baricenter += set[i];

    baricenter /= (double) set.size();

    double distance = 0;

    for(size_t i=0; i<set.size(); ++i) {

        distance += mpl::norm(set[i], baricenter);

    }

    return distance / double(set.size());

  }
  

  /*****************************************************************************/
  //  NNDistance() - Minima distanza mediana (subsampled) tra i punti
  /*****************************************************************************/
  template <typename T1, typename T2>
  double NNDistance(const std::vector<T1> & setA, const std::vector<T2> & setB, double * var = NULL, float subSamplePercentage = 100.0){
    
    std::size_t howMany;
    
    std::vector<float> minDist;
    
    if(subSamplePercentage == 100.0) {
      
      minDist.resize(setA.size(), FLT_MAX);
      
      //#pragma omp parallel for num_threads(2) ordered schedule(static)
      for(std::size_t i=0; i<setA.size(); ++i){
        
        for(std::size_t j=0; j<setB.size(); ++j){
          
          double dist = fabs(cv::norm(setA[i] - setB[j]));
          
          if(minDist[i] > dist) minDist[i] = dist;
          
        }
        
      }

      howMany = setA.size();
            
    } else {
      
      howMany = (setA.size() / 100.0) * subSamplePercentage;
      
      if(howMany == 0) {
        
        do {
          
          subSamplePercentage *= 2.0;
          
          if(subSamplePercentage > 100.0) subSamplePercentage = 100.0;
          
          howMany = (setA.size() / 100.0) * subSamplePercentage;
          
          if(subSamplePercentage == 100.0) break;
          
        } while(howMany <= 0);
        
      }
      
      if(howMany == 0) {
        fprintf(stderr, "");
        abort();
      }
      
      std::vector<std::size_t> shuffled(setA.size());
      
      for(std::size_t i=0; i<setA.size(); ++i) shuffled[i] = i;
      
  #ifndef DEBUG_MODE
      
      std::random_device rd;
      std::mt19937 g(rd());
      
      std::shuffle(shuffled.begin(), shuffled.end(), g);
      
    //  std::random_shuffle(shuffled.begin(), shuffled.end());
  #endif
      
      minDist.resize(howMany, FLT_MAX);

      //#pragma omp parallel for num_threads(2) ordered schedule(static)
      for(std::size_t i=0; i<howMany; ++i){
        
        for(std::size_t j=0; j<setB.size(); ++j){
          
          float dist = fabs(cv::norm(setA[shuffled[i]] - setB[j]));
          
          if(minDist[i] > dist) minDist[i] = dist;
          
        }
        
      }
    
    } /* else */
    
    
    /*
    std::size_t halfSize = howMany * 0.5;

    //std::nth_element(minDist.begin(), minDist.begin()+halfSize+2, minDist.end());
    
    std::sort(minDist.begin(), minDist.end());
    
    double median = 0;
    
    if((howMany % 2) != 0) median = minDist[halfSize];
    //else median = pow((sqrt(minDist[halfSize-1]) + sqrt(minDist[halfSize])) * 0.5, 2);
    else median = (minDist[halfSize-1] + minDist[halfSize]) * 0.5;

    if(var!=NULL){
      
      
    }*/
    
    //printf("minDist.size(%u) %p\n", minDist.size(), var); fflush(stdout);
    
    return median(minDist,var);
    
  }
  
  
  /*****************************************************************************/
  //  NNDistance() - Minima distanza mediana (subsampled) tra i punti
  /*****************************************************************************/
  template <typename T1>
  double NNDistance(const std::vector<T1> & setA, double * var = NULL, float subSamplePercentage = 100.0){
    
    std::size_t howMany;
    
    std::vector<float> minDist;
    
    if(subSamplePercentage == 100.0) {
      
      minDist.resize(setA.size(), FLT_MAX);
      
      //#pragma omp parallel for num_threads(2) ordered schedule(static)
      for(std::size_t i=0; i<setA.size(); ++i){
        
        for(std::size_t j=i+1; j<setA.size(); ++j){
          
          double dist = fabs(setA[i] - setA[j]);
          
          if(minDist[i] > dist) minDist[i] = dist;
          if(minDist[j] > dist) minDist[j] = dist;

        }
        
      }
      
      howMany = setA.size();
      
    } else {
      
      howMany = (setA.size() / 100.0) * subSamplePercentage;
      
      if(howMany == 0) {
        
        do {
          
          subSamplePercentage *= 2.0;
          
          if(subSamplePercentage > 100.0) subSamplePercentage = 100.0;
          
          howMany = (setA.size() / 100.0) * subSamplePercentage;
          
          if(subSamplePercentage == 100.0) break;
          
        } while(howMany <= 0);
        
      }
      
      if(howMany == 0) {
        fprintf(stderr, "");
        abort();
      }
      
      std::vector<std::size_t> shuffled(setA.size());
      
      for(std::size_t i=0; i<setA.size(); ++i) shuffled[i] = i;
      
#ifndef DEBUG_MODE
      
      std::random_device rd;
      std::mt19937 g(rd());
      
      std::shuffle(shuffled.begin(), shuffled.end(), g);
      
     // std::random_shuffle(shuffled.begin(), shuffled.end());
#endif
      
      minDist.resize(howMany, FLT_MAX);
      
      //#pragma omp parallel for num_threads(2) ordered schedule(static)
      for(std::size_t i=0; i<howMany; ++i){
        
        for(std::size_t j=0; j<setA.size(); ++j){
          
          if(shuffled[i] == j) continue;
          
          float dist = fabs(setA[shuffled[i]] - setA[j]);
          
          if(minDist[i] > dist) minDist[i] = dist;
          
        }
        
      }
      
    } /* else */
    
    
    /*
     std::size_t halfSize = howMany * 0.5;
     
     //std::nth_element(minDist.begin(), minDist.begin()+halfSize+2, minDist.end());
     
     std::sort(minDist.begin(), minDist.end());
     
     double median = 0;
     
     if((howMany % 2) != 0) median = minDist[halfSize];
     //else median = pow((sqrt(minDist[halfSize-1]) + sqrt(minDist[halfSize])) * 0.5, 2);
     else median = (minDist[halfSize-1] + minDist[halfSize]) * 0.5;
     
     if(var!=NULL){
     
     
     }*/
    
    return median(minDist,var);
    
  }
  
  
} /* namespace utils */


#endif /* _H_MPL_UTILS_H_ */


