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

#ifndef _H_MPL_CLUSTERING_H_
#define _H_MPL_CLUSTERING_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

#include <mpl/utils.hpp>


/*****************************************************************************/
// mpl::clustering
/*****************************************************************************/
namespace mpl::clustering {
  
  /*****************************************************************************/
  //  byDistance() - clasterizzo in base alla distanza
  /*****************************************************************************/
  template <typename T>
  std::vector<std::vector<std::size_t>> byDistance(const T & set, double maxDist){
    
    std::vector<std::vector<std::size_t> > clusters;
    
    std::vector<bool> isTaken(set.size(), false);
    
    // ciclo su tutti i punti 3D
    for(std::size_t i=0; i<set.size(); ++i){
      
      // se il punto non e' stato ancora preso
      if(!isTaken[i]){
        
        isTaken[i] = true;
        
        clusters.push_back(std::vector<std::size_t>());
        
        std::vector<std::size_t> & tmpCluster = clusters.back();
        
        tmpCluster.push_back(i);
        
        // ciclo su i punti che fanno parte del cluster
        for(std::size_t j=0; j<tmpCluster.size(); ++j){
          
          // ciclo su tutti i punti del set iniziale
          for(std::size_t k=0; k<set.size(); ++k){
            
            // se il punto non e' stato ancora preso
            if(!isTaken[k]){
              
              // mi calcolo la distanza tra i punti
              double dist = set[tmpCluster[j]] - set[k];
              
              // mi sta abbastanza vicino
              if(dist <= maxDist){
                
                // assegno lo stesso cluster id
                isTaken[k] = true;
                
                // aggiungo il punto al cluster
                tmpCluster.push_back(k);
                
              }
              
            }
            
          }
          
        }
        
      }
      
    }
    
    return clusters;
    
  }
  
  
  /*****************************************************************************/
  //  byNNDistance() - clasterizzo in base alla distanza primo vicino
  /*****************************************************************************/
  template <typename T>
  std::vector<std::vector<std::size_t>> byNNDistance(const T & set, double factor = 1.0){
    
    double NNDistance = mpl::utils::NNDistance(set);
    
    double thresholdDist = NNDistance * factor;
    
    return mpl::clustering::byDistance(set, thresholdDist);
    
  }
  
  
  /*****************************************************************************/
  //  kmeans
  /*****************************************************************************/
  void kmeans(const cv::Mat points, int howMany, std::vector<cv::Point2f> & _centers, cv::KmeansFlags flags = cv::KMEANS_RANDOM_CENTERS, int attempts = 100) {
    
    cv::Mat labels; cv::Mat centers;
    
    if(points.rows > howMany) {
      fprintf(stderr, "mpl::clustering::kmeans() - the number of input points must be greate or equal that the number of cluster to found\n");
      abort();
    }
    
    cv::kmeans(points, howMany, labels, cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10000, 0.00001), attempts, flags, centers);
    
    _centers.resize(centers.rows);
    
    for(int i=0; i<centers.rows; ++i)
      _centers[i] = centers.at<cv::Point2f>(i);
    
  }
  
  
} /* namespace mpl::clustering */


#endif /* _H_MPL_CLUSTERING_H_ */


