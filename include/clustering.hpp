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
#include <cassert>
#include <deque>

#include <opencv2/opencv.hpp>

#include <mpl/utils.hpp>



//*****************************************************************************/
// mpl::clustering
//*****************************************************************************/
namespace mpl::clustering {
  
  constexpr auto defaultOp = [](auto const & a, auto const & b){ return cv::norm(a - b); };

  //*****************************************************************************/
  //  connectedComponents() - clasterizzo in base alla distanza
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultOp)>
  std::vector<std::vector<std::size_t>> connectedComponents(const T & data, double maxDist, Op op = defaultOp) {
    
    std::vector<std::vector<std::size_t> > clusters;
    
    std::vector<bool> isTaken(data.size(), false);
    
    // ciclo su tutti i punti 3D
    for(std::size_t i=0; i<data.size(); ++i){
      
      // se il punto non e' stato ancora preso
      if(!isTaken[i]){
        
        isTaken[i] = true;
        
        clusters.push_back(std::vector<std::size_t>());
        
        std::vector<std::size_t> & tmpCluster = clusters.back();
        
        tmpCluster.push_back(i);
        
        // ciclo su i punti che fanno parte del cluster
        for(std::size_t j=0; j<tmpCluster.size(); ++j){
          
          // ciclo su tutti i punti del set iniziale
          for(std::size_t k=0; k<data.size(); ++k){
            
            // se il punto non e' stato ancora preso
            if(!isTaken[k]){
              
              // mi calcolo la distanza tra i punti
              double dist = op(data[tmpCluster[j]], data[k]);
              
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
  
  //*****************************************************************************/
  //  connectedComponentsMedianFirstNN() - clasterizzo in base alla distanza mediana del primo vicino
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultOp)>
  std::vector<std::vector<std::size_t>> connectedComponentsMedianFirstNN(const T & data, double factor = 1.0, Op op = defaultOp) {
    
    double NNDistance = mpl::utils::medianFirstNNDistance(data, op);
    
    double thresholdDist = NNDistance * factor;
    
    return mpl::clustering::connectedComponents(data, thresholdDist, op);
    
  }
  
  //*****************************************************************************/
  //  kmeans
  //*****************************************************************************/
  void kmeans(const cv::Mat & points, int howMany, std::vector<std::vector<std::size_t>> & clusters, cv::KmeansFlags flags = cv::KMEANS_RANDOM_CENTERS, int attempts = 100) {
    
    clusters.clear();

    cv::Mat labels; cv::Mat centers;
    
    if(points.rows < howMany) {
      throw std::invalid_argument("mpl::clustering::kmeans(): points.rows must be >= howMany");
    }
    
    cv::kmeans(points, howMany, labels, cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 0.1), attempts, flags);
    
    clusters.resize(howMany);
    
    for(int i=0; i<labels.rows; i++){
      int c = labels.at<int>(i,0);
      clusters[c].push_back(i);
    }

  }

  //*****************************************************************************/
  // gmm()
  //
  // Applica un Gaussian Mixture Model (EM) ai punti in input e assegna
  // ciascun punto al cluster con probabilità a posteriori massima.
  //
  //*****************************************************************************/
  void gmm(const cv::Mat & points, int howMany, std::vector<std::vector<std::size_t>> & clusters) {
             
    clusters.clear();

    cv::Ptr<cv::ml::EM> model = cv::ml::EM::create();
    model->setClustersNumber(howMany);
    model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_DIAGONAL);
    model->setTermCriteria(cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.1));

    if(!model->trainEM(points)) {
      fprintf(stderr, "mpl::clustering::gmm() - error\n");
      abort();
    }

    clusters.resize(howMany);

    for(int i = 0; i < points.rows; ++i) {
      
      cv::Mat probs;
      model->predict2(points.row(i), probs);

      cv::Point maxLoc;
      cv::minMaxLoc(probs, nullptr, nullptr, nullptr, &maxLoc);

      int c = (probs.rows == 1) ? maxLoc.x : maxLoc.y;
      clusters[c].push_back(i);
      
    }
    
  }

  //*****************************************************************************/
  // mahalanobis()
  //
  // Seleziona i punti che si trovano entro una certa soglia di distanza
  // di Mahalanobis dal centro del dataset.
  //
  //*****************************************************************************/
  void mahalanobis(const cv::Mat & points, float threshold, std::vector<std::size_t> & cluster) {
      
    cluster.clear();
    if(points.rows < 2) return;

    cv::Mat mean, covar;
    cv::calcCovarMatrix(points, covar, mean, cv::COVAR_NORMAL | cv::COVAR_ROWS, CV_32F);

    covar = covar / float(points.rows - 1);
    covar += cv::Mat::eye(covar.size(), covar.type()) * 1e-6f;

    cv::Mat invCovar;
    cv::invert(covar, invCovar, cv::DECOMP_SVD);

    for(int i=0; i<points.rows; ++i) {
      cv::Mat diff = points.row(i) - mean;
      cv::Mat tmp = diff * invCovar * diff.t();
      float d2 = tmp.at<float>(0,0);
      if(d2 <= threshold)
        cluster.push_back(i);
    }
    
  }


  //*****************************************************************************/
  // nearestNeighbor()
  //
  // Seleziona i punti la cui distanza dal nearest neighbor più vicino
  // è inferiore a una soglia calcolata come:
  //
  //   threshold = mean(nearest_neighbor_distance) * scale
  //
  //*****************************************************************************/
  void nearestNeighbor(const cv::Mat & points, float scale, std::vector<std::size_t> & cluster) {

    cluster.clear();

    cv::flann::Index flannIndex(points, cv::flann::LinearIndexParams());

    // Prepara matrici per indici e distanze
    cv::Mat indices(points.rows, 2, CV_32S);
    cv::Mat dists(  points.rows, 2, CV_32F);

    // kNN search
    flannIndex.knnSearch(points, indices, dists, 2, cv::flann::SearchParams());

    // mi calcolo la media delle distanze
    std::vector<float> nnDist(points.rows);
    double sum = 0.0;
    for(int i = 0; i < points.rows; ++i) {
      float d = dists.at<float>(i,1); // distanza al quadrato
      nnDist[i] = d;
      sum += d;
    }

    // mi calcolo la distanza media
    float meanDist = static_cast<float>(sum / points.rows);

    // mi calcolo la soglia
    float threshold = meanDist * scale;

    // butto via i punti non buoni
    for(int i = 0; i < points.rows; ++i) {
        if(nnDist[i] <= threshold)
          cluster.push_back(i);
    }
    
  }

  //*****************************************************************************/
  //  dbscan
  //*****************************************************************************/
  //
  // maxDistance      Raggio di vicinanza
  // minClusterSize   Numero minimo di punti per formare un cluster
  // clusters         Label di cluster per ciascun punto (>=0: cluster ID, NOISE: -1)
  //
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultOp)>
  std::vector<std::size_t> dbscan(const T & data, double maxDistance, int minClusterSize,
                                  std::vector<std::vector<std::size_t>> & clusters, Op op = defaultOp) {
      
    const int n = static_cast<int>(data.size());

    enum { UNVISITED, VISITED };

    std::vector<int> labels(n, -1);
    std::vector<int> state(n, UNVISITED);

    int clusterId = 0;

    auto countNeighbors = [&](int idx) {
      int count = 0;
      for(int j=0; j<n; ++j) {
        if(j != idx && op(data[idx], data[j]) <= maxDistance) {
          ++count;
          if(count >= minClusterSize - 1)
            return count;
        }
      }
      return count;
    };

    for(int i=0; i<n; ++i) {
      if(state[i] == VISITED) continue;

      state[i] = VISITED;

      if(countNeighbors(i) < minClusterSize - 1) {
        labels[i] = -1;
        continue;
      }

      labels[i] = clusterId;
      std::vector<int> seeds = {i};

      for(std::size_t k=0; k<seeds.size(); ++k) {
        int current = seeds[k];

        for(int j=0; j<n; ++j) {
          if(j == current) continue;

          if(op(data[current], data[j]) <= maxDistance) {
            if(state[j] == UNVISITED) {
              state[j] = VISITED;
              if(countNeighbors(j) >= minClusterSize - 1)
                seeds.push_back(j);
            }

            if(labels[j] == -1)
              labels[j] = clusterId;
          }
        }
      }

      ++clusterId;
    }

    clusters.clear();
    clusters.resize(clusterId);

    std::vector<std::size_t> noise;
    for(int i=0; i<n; ++i) {
      if(labels[i] >= 0) clusters[labels[i]].push_back(i);
      else noise.push_back(i);
    }

    return noise;
  }

} /* namespace mpl::clustering */


#endif /* _H_MPL_CLUSTERING_H_ */

