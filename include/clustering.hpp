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
  //  namespace neighbor
  /*****************************************************************************/
  namespace neighbor {
    
    /*****************************************************************************/
    //  byDistance() - clasterizzo in base alla distanza
    /*****************************************************************************/
    template <typename T1, typename T2>
    std::vector<std::vector<uint32_t> > byDistance(const T1 & setA, const T2 & setB, double maxDist, std::size_t maxNeighborSize = std::numeric_limits<std::size_t>::max()){
      
      std::vector<std::vector<uint32_t> > neighbor(setA.size());
      
      // ciclo su tutti i punti 3D
      for(std::size_t i=0; i<setA.size(); ++i){
        
       /// neighbor.push_back(std::vector<uint32_t>());
        //printf("a "); setA[i].print();

        std::vector<uint32_t> & tmpNeighbor = neighbor[i];
        
        for(uint32_t j=0; j<setB.size(); ++j){
          
          // mi calcolo la distanza tra i punti
          double dist = cv::norm(setA[i], setB[j]);
          
          //printf("b "); setB[j].print();

          //printf("D %e", dist);
          
          // mi sta abbastanza vicino
          if(dist <= maxDist){
            
            //printf(" ok ");
            
            // aggiungo il punto al cluster
            tmpNeighbor.push_back(j);
            
            if(tmpNeighbor.size() > maxNeighborSize){
              tmpNeighbor.clear();
              //printf(" troppi \n");
              break;
            }
            
          }
          
          //printf("\n");
          
        }
        
      }
      
      return neighbor;
      
    }
    
  } /* namespace neighbor */  
  
  
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


