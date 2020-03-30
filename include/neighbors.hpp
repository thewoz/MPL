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

#ifndef _H_MPL_NEIGHBORS_H_
#define _H_MPL_NEIGHBORS_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

#include <mpl/utils.hpp>

/*****************************************************************************/
//  namespace neighbors
/*****************************************************************************/
namespace mpl::neighbors {
  
  //***************************************************************************
  //  findNeighbors()
  //***************************************************************************
  template <typename T1, typename T2>
  std::vector<std::vector<uint32_t> > byDistance(const T1 & setA, const T2 & setB, double maxDist, std::size_t maxNeighborSize = std::numeric_limits<std::size_t>::max()){

    std::vector<std::vector<uint32_t> > neighbor(setA.size());

    // ciclo su tutti i punti nel primo insieme
    for(std::size_t i=0; i<setA.size(); ++i){

      std::vector<uint32_t> & tmpNeighbor = neighbor[i];

      // ciclo su tutti i punti del secono insieme
      for(uint32_t j=0; j<setB.size(); ++j){

        // mi calcolo la distanza tra i punti
        double dist = cv::norm(setA[i], setB[j]);

        // mi sta abbastanza vicino e non sono lo stesso punto
        if(dist <= maxDist && setA[i] != setB[j]){

          // aggiungo il punto al cluster
          tmpNeighbor.push_back(j);

          // se il punto ha piu di maxNeighborSize vicini lo butto
          if(tmpNeighbor.size() > maxNeighborSize){
            tmpNeighbor.clear();
            break;
          }

        }

      }

    }

    return neighbor;

  }
  
  
  //****************************************************************************/
  //  metric()
  //****************************************************************************/
  template <class T>
  std::vector<T> metric(const T & point, const std::vector<T> & points, double maxDist) {
    
    std::vector<T> neighbors;
    
    for(size_t i=0; i<points.size(); ++i) {
      
      double dist = cv::norm(point, points[i]);
      
      // Se il punto e' abbastanza vicino e i punti sono uguali
      if(dist <= maxDist && point != points[i])
        neighbors.push_back(points[i]);
      
    }
    
    return neighbors;
    
  }
  
  
  //****************************************************************************/
  //  topological()
  //****************************************************************************/
  template <class T>
  std::vector<T> topological(const T & point, const std::vector<T> & points, size_t neighborsSize) {
    
    struct dist_t {
      
      size_t index;
      double dist;
      
      bool operator < (const dist_t & value) const { return (dist<value.dist); }
      
    };
    
    std::vector<dist_t> distances(points.size());
    
    std::vector<T> neighbors;
    
    // Mi calcolo le distanze tra il punto e gli altri punti
    for(size_t i=0; i<points.size(); ++i) {
      
      double dist = cv::norm(point, points[i]);
      
      distances[i].index = i;
      distances[i].dist  = dist;
      
    }
    
    // Ordino i punti
    //std::nth_element(distances.begin(), distances.begin()+neighborsSize, distances.end());
    std::sort(distances.begin(), distances.end());

    neighbors.resize(neighborsSize);
        
    // Mi prendo i primi N vicini
    for(size_t i=0; i<neighborsSize; ++i) {
      // Se il punto stava nell'insieme lo salto
      if(point == points[distances[i].index]) continue;
      neighbors[i] = points[distances[i].index];
    }
      
    return neighbors;
    
  }

  
} /* mpl::neighbors */

#endif /* _H_MPL_NEIGHBORS_H_ */


