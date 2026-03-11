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
  //  byDistance() - clasterizzo in base alla distanza
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultOp)>
  std::vector<std::vector<std::size_t>> byDistance(const T & data, double maxDist, Op op = defaultOp) {
    
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
  //  byNNDistance() - clasterizzo in base alla distanza primo vicino
  //*****************************************************************************/
  template <typename T, typename Op = decltype(defaultOp)>
  std::vector<std::vector<std::size_t>> byMedianFirstNNDistance(const T & data, double factor = 1.0, Op op = defaultOp) {
    
    double NNDistance = mpl::utils::medianFirstNNDistance(data, op);
    
    double thresholdDist = NNDistance * factor;
    
    return mpl::clustering::byDistance(data, thresholdDist, op);
    
  }
  

  //*****************************************************************************/
  //  kmeans
  //*****************************************************************************/
//NOTE: VA TESTATO

  void kmeans(const cv::Mat & points, int howMany, std::vector<std::vector<std::size_t>> & clusters, cv::KmeansFlags flags = cv::KMEANS_RANDOM_CENTERS, int attempts = 100) {
    
    clusters.clear();

    cv::Mat labels; cv::Mat centers;
    
    if(points.rows > howMany) {
      fprintf(stderr, "mpl::clustering::kmeans() - the number of input points must be greate or equal that the number of cluster to found\n");
      abort();
    }
    
    cv::kmeans(points, howMany, labels, cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 0.1), attempts, flags);
    
    clusters.resize(howMany);
    
    for(int i=0; i<labels.rows; i++){
      int c = labels.at<int>(i,0);
      clusters[c].push_back(i);
    }

  }


  //*****************************************************************************/
  //  gmm
  //*****************************************************************************/
//NOTE: VA TESTATO

  void gmm(const cv::Mat & points, int howMany, std::vector<std::vector<std::size_t>> & clusters) {
     
    clusters.clear();

    // creo l’oggetto EM
    cv::Ptr<cv::ml::EM> gmm = cv::ml::EM::create();
    gmm->setClustersNumber(howMany);
    gmm->setCovarianceMatrixType(cv::ml::EM::COV_MAT_DIAGONAL);
    gmm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.1));

    // alleno il modello
    gmm->trainEM(points);

    // esttraggo i parametri
    cv::Mat weights = gmm->getWeights();
    std::vector<cv::Mat> means = gmm->getMeans();
    std::vector<cv::Mat> covs;
    gmm->getCovs(covs);

    // assegno ogni punto al cluster più probabile:
    cv::Mat labels, probs;
    gmm->predict2(points, labels);

    clusters.resize(howMany);
      
    for(int i=0; i<labels.rows; i++){
      int c = labels.at<int>(i,0);
      clusters[c].push_back(i);
    }
    
  }

  //*****************************************************************************/
  //  mahalanobis
  //*****************************************************************************/
//NOTE: VA TESTATO

  void mahalanobis(const cv::Mat & points, float chi2Threshold, std::vector<std::size_t> & cluster) {
    
    cluster.clear();
    
    // Calcolo media e covarianza
    cv::Mat mean, covar;
    cv::calcCovarMatrix(points, covar, mean, cv::COVAR_NORMAL | cv::COVAR_ROWS, CV_32F);
    
    // covarianza campionaria
    covar = covar / float(points.rows - 1);

    // inversione
    cv::Mat invCovar = covar.inv();

    // mi calcolo mahalanobis per ogni punto
    for(int i=0; i<points.rows; ++i) {
      cv::Mat diff = points.row(i) - mean.t();
      cv::Mat tmp = diff * invCovar * diff.t();
      float d2 = tmp.at<float>(0,0);
      if(d2 <= chi2Threshold)
        cluster.push_back(i);
    }

  }

//*****************************************************************************/
//  nearestNeighborApprox
//*****************************************************************************/
//NOTE: VA TESTATO

void nearestNeighborApprox(const cv::Mat & points, float scaleFactor, std::vector<std::size_t> & cluster) {

  cluster.clear();

  // Costruisci l’indice FLANN (KD-Tree)
  cv::flann::Index flannIndex(points, cv::flann::KDTreeIndexParams(5));

  // Prepara matrici per indici e distanze (k=2: self + nearest neighbor)
  cv::Mat indices(points.rows, 2, CV_32S);
  cv::Mat dists(  points.rows, 2, CV_32F);

  // kNN search
  flannIndex.knnSearch(points, indices, dists, 2, cv::flann::SearchParams(32));

  // mi calcolo la media delle stanze
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
  float threshold = meanDist * scaleFactor;

  // butto via i punti non buoni
  for(int i = 0; i < points.rows; ++i) {
      if(nnDist[i] <= threshold)
        cluster.push_back(i);
  }
  
}

//*****************************************************************************/
//  nearestNeighbor
//*****************************************************************************/
//NOTE: VA TESTATO
void nearestNeighbor(const cv::Mat & points, float scaleFactor, std::vector<std::size_t> & cluster) {

  cluster.clear();

  // Costruisci l’indice FLANN (KD-Tree)
  cv::flann::Index flannIndex(points, cv::flann::LinearIndexParams());

  // Prepara matrici per indici e distanze (k=2: self + nearest neighbor)
  cv::Mat indices(points.rows, 2, CV_32S);
  cv::Mat dists(  points.rows, 2, CV_32F);

  // kNN search
  flannIndex.knnSearch(points, indices, dists, 2, cv::flann::SearchParams());

  // mi calcolo la media delle stanze
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
  float threshold = meanDist * scaleFactor;

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
std::vector<std::size_t> dbscan(const T & data, double maxDistance, int minClusterSize, std::vector<std::vector<std::size_t>> & clusters, Op op = defaultOp) {
    
  const int n = static_cast<int>(data.size());

  enum { UNVISITED, VISITED };

  std::vector<int> labels(n, -1);
  std::vector<int> state(n, UNVISITED);

  int clusterId = 0;

  // Conta vicini con early stop
  auto countNeighbors = [&](int idx) {
    int count = 0;
    for(int j=0; j<n; ++j) {
      if(j != idx && op(data[idx], data[j]) <= maxDistance) {
        ++count;
        if(count >= minClusterSize)
          return count;
      }
    }
    return count;
  };

  for(int i=0; i<n; ++i) {
    
    if(state[i] == VISITED)  continue;

    state[i] = VISITED;

    if(countNeighbors(i) < minClusterSize) {
      labels[i] = -1;
      continue;
    }

    // Nuovo cluster
    labels[i] = clusterId;

    std::vector<int> seeds;
    seeds.push_back(i);

    // Espansione iterativa
    for(size_t k=0; k< eeds.size(); ++k) {
      
      int current = seeds[k];

      for(int j=0; j<n; ++j) {
        
        if(j == current) continue;

        if(op(data[current], data[j]) <= maxDistance) {
          
          if(state[j] == UNVISITED) {
            state[j] = VISITED;
            if(countNeighbors(j) >= minClusterSize)
              seeds.push_back(j);
          }

          if(labels[j] == -1)
            labels[j] = clusterId;
        }
        
      }
      
    }

    ++clusterId;
    
  }

  // Costruzione clusters
  clusters.clear();
  clusters.resize(clusterId);

  std::vector<std::size_t> noise;

  for(int i=0; i<n; ++i) {
   if (labels[i] >= 0)
     clusters[labels[i]].push_back(i);
   else
     noise.push_back(i);
  }

  return noise;
  
}


} /* namespace mpl::clustering */


#endif /* _H_MPL_CLUSTERING_H_ */
