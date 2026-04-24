/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2026
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

#ifndef _H_MPL_ALPHA_SHAPE_2D_H_
#define _H_MPL_ALPHA_SHAPE_2D_H_

#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

//*****************************************************************************
// namespace mpl
//*****************************************************************************
namespace mpl {

//*****************************************************************************
// namespace utils
//*****************************************************************************
namespace utils {

// Tolleranza di default per confrontare punti floating point.
// Va bene soprattutto se le coordinate sono in pixel o comunque su una scala simile.
inline constexpr float kPointEpsilon = 1e-4f;

// Chiave intera per indicizzare una cella della griglia spaziale.
// Utilizzata per velocizzare il matching tra punti originali e i vertici dei triangoli restituiti da OpenCV
struct gridKey_t {
  int x; int y;
  bool operator==(const gridKey_t & other) const noexcept {
    return x == other.x && y == other.y;
  }
};

// Hash per GridKey per usarla in unordered_map
struct gridKeyHash_t {
  std::size_t operator () (const gridKey_t & key) const noexcept {
    const std::size_t h1 = std::hash<int>{}(key.x);
    const std::size_t h2 = std::hash<int>{}(key.y);
    return h1 ^ (h2 + 0x9e3779b9u + (h1 << 6) + (h1 >> 2));
  }
};

// Hash per una coppia di interi.
// Gli edge vengono rappresentati come pair<int, int> dove i due interi sono indici di punti originali
struct intPairHash_t {
  std::size_t operator () (const std::pair<int, int> & value) const noexcept {
    const std::size_t h1 = std::hash<int>{}(value.first);
    const std::size_t h2 = std::hash<int>{}(value.second);
    return h1 ^ (h2 + 0x9e3779b9u + (h1 << 6) + (h1 >> 2));
  }
};

// Questo alias viene usato per verificare a compile-time che PointT
// abbia davvero i campi pubblici x e y.
//  template <typename PointT>
//  using HasXY = std::void_t<decltype(std::declval<PointT>().x),decltype(std::declval<PointT>().y)>;
//
//  // Funzione che converte un punto generico PointT in cv::Point2f
//  // Funziona per qualsiasi tipo con membri pubblici x e y.
//  template <typename PointT, typename = HasXY<PointT>>
//  inline cv::Point2f toCvPoint(const PointT& p) {
//    return cv::Point2f(static_cast<float>(p.x), static_cast<float>(p.y));
//  }

//// Distanza quadratica tra due punti con tolleranza
//inline bool almostEqual(const cv::Point2f & a, const cv::Point2f & b, const float eps = kPointEpsilon) {
//  const float dx = a.x - b.x;
//  const float dy = a.y - b.y;
//  return dx * dx + dy * dy < eps * eps;
//}

// Distanza quadratica tra due punti
template <class T>
inline float squaredDistance(const T & a, const cv::Point2f & b) {
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// Distanza euclidea standard.
inline double distance2D(const cv::Point2f & a, const cv::Point2f & b) {
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  return std::sqrt(dx * dx + dy * dy);
}

// Area del triangolo definito da tre punti
// Se i punti sono allineati o quasi allineati, l'area è quasi zero.
inline double triangleArea(const cv::Point2f & a, const cv::Point2f & b, const cv::Point2f & c) {
  return 0.5 * std::abs(a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
}

// Calcola il raggio della circonferenza circoscritta al triangolo.
// Questo è il criterio centrale dell'alpha shape:
// teniamo il triangolo solo se il suo circumradius è < alpha.
inline double circumradius(const cv::Point2f & a, const cv::Point2f & b, const cv::Point2f & c) {
  
  const double sideA = distance2D(b, c);
  const double sideB = distance2D(a, c);
  const double sideC = distance2D(a, b);
  
  const double area = triangleArea(a, b, c);
  
  // Se il triangolo è degenere, consideriamo il raggio infinito così non verrà mai accettato.
  if(area < 1e-8) return std::numeric_limits<double>::infinity();
  
  return (sideA * sideB * sideC) / (4.0 * area);
  
}

// Verifica che un punto sia interno al rettangolo.
// Serve a scartare triangoli spurii vicino al bordo del Subdiv2D.
inline bool isInsideRect(const cv::Rect2f & rect, const cv::Point2f & point) {
  return point.x >= rect.x && point.x <= rect.x + rect.width && point.y >= rect.y && point.y <= rect.y + rect.height;
}

// Normalizza un edge non orientato. (a,b) e (b,a) devono essere lo stesso edge.
inline std::pair<int, int> makeEdgeKey(int a, int b) {
  return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

// Trasforma un punto 2D nella chiave della cella di griglia in cui cade.
inline gridKey_t makeGridKey(const cv::Point2f & point, float cellSize) {
  return { static_cast<int>(std::floor(point.x / cellSize)), static_cast<int>(std::floor(point.y / cellSize)) };
}

// Converte tutto il dataset in cv::Point2f. Internamente usiamo questi punti per OpenCV
//  template <typename PointT, typename = HasXY<PointT>>
//  inline std::vector<cv::Point2f> convertToCvPoints(const std::vector<PointT> & points) {
//
//    std::vector<cv::Point2f> cvPoints;
//    cvPoints.reserve(points.size());
//
//    for(const auto & p : points)
//      cvPoints.push_back(toCvPoint(p));
//
//    return cvPoints;
//
//  }

// Costruisce una griglia spaziale:
// ogni cella contiene gli indici dei punti originali che vi ricadono
template <class T>
inline std::unordered_map<gridKey_t, std::vector<int>, gridKeyHash_t> buildPointGrid(const std::vector<T> & points, float cellSize) {
  
  std::unordered_map<gridKey_t, std::vector<int>, gridKeyHash_t> grid; grid.reserve(points.size() * 2);
  
  for(int i=0; i<(int)points.size(); ++i)
    grid[makeGridKey(points[i], cellSize)].push_back(i);
  
  return grid;
  
}

// Dato un punto q restituito da OpenCV dentro un triangolo,
// cerca il corrispondente indice del punto originale.
// Invece di prendere il primo match compatibile, questa versione prende
// il candidato più vicino entro eps. È più robusta se ci sono punti vicini.
template <class T>
inline int findOriginalPointIndex(const cv::Point2f & q, const std::vector<T> & points,
                                  const std::unordered_map<gridKey_t, std::vector<int>, gridKeyHash_t> & grid,
                                  float cellSize, float eps = kPointEpsilon) {
  
  const gridKey_t baseKey = makeGridKey(q, cellSize);
  
  int bestIndex = -1;
  float bestDist2 = std::numeric_limits<float>::infinity();
  
  // Cerchiamo nella cella del punto e nelle 8 celle vicine.
  // Questo basta nella pratica per compensare piccoli errori numerici.
  for(int dy=-1; dy<=1; ++dy) {
    
    for(int dx=-1; dx<=1; ++dx) {
      
      const gridKey_t candidateKey{ baseKey.x + dx, baseKey.y + dy };
      const auto it = grid.find(candidateKey);
      
      if(it == grid.end()) continue;
      
      for(const int pointIndex : it->second) {
        const float dist2 = squaredDistance(points[pointIndex], q);
        if(dist2 < eps * eps && dist2 < bestDist2) {
          bestDist2 = dist2;
          bestIndex = pointIndex;
        }
      }
    }
  }
  
  return bestIndex;
  
}

// Costruisce gli edge di bordo dell'alpha shape usando gli indici
// dei punti originali.
//
// Procedura:
// 1. triangolazione di Delaunay
// 2. filtro dei triangoli con circumradius < alpha
// 3. conteggio degli edge
// 4. edge presenti una sola volta = bordo
template <class T>
inline std::vector<std::pair<int, int>>
computeBoundaryEdgesByIndex(const std::vector<T> & points, double alpha, float pointMatchEps = kPointEpsilon, float gridCellSize = 1.0f) {
  
  std::vector<std::pair<int, int>> boundaryEdges;
  
  if(points.size() < 4)
    return boundaryEdges;
  
  // Bounding box dei punti.
  cv::Rect2f boundingBox = cv::boundingRect(points);
  
  // Allargo un po' il bounding box per stare più tranquillo con Subdiv2D
  const float margin = 10.0f;
  boundingBox.x -= margin;
  boundingBox.y -= margin;
  boundingBox.width += 2.0f * margin;
  boundingBox.height += 2.0f * margin;
  
  // Triangolazione di Delaunay tramite OpenCV
  cv::Subdiv2D subdiv(boundingBox);
  for(const auto & point : points)
    subdiv.insert(point);
  
  // Ogni triangolo è restituito come [x1, y1, x2, y2, x3, y3]
  std::vector<cv::Vec6f> triangleList;
  subdiv.getTriangleList(triangleList);
  
  // Griglia dei punti originali per matching rapido.
  const auto pointGrid = utils::buildPointGrid(points, gridCellSize);
  
  // Conta quante volte compare ciascun edge nei triangoli accettati.
  std::unordered_map<std::pair<int, int>, int, utils::intPairHash_t> edgeCount;
  edgeCount.reserve(triangleList.size() * 3);
  
  for(const auto & triangle : triangleList) {
    
    const cv::Point2f a(triangle[0], triangle[1]);
    const cv::Point2f b(triangle[2], triangle[3]);
    const cv::Point2f c(triangle[4], triangle[5]);
    
    // Scarta triangoli che escono dal rettangolo valido.
    if(!isInsideRect(boundingBox, a) || !isInsideRect(boundingBox, b) || !isInsideRect(boundingBox, c))
      continue;
    
    // Applica il criterio alpha.
    const double r = circumradius(a, b, c);
    if(r >= alpha)
      continue;
    
    // Match dei vertici del triangolo con i punti originali.
    const int ia = findOriginalPointIndex(a, points, pointGrid, gridCellSize, pointMatchEps);
    const int ib = findOriginalPointIndex(b, points, pointGrid, gridCellSize, pointMatchEps);
    const int ic = findOriginalPointIndex(c, points, pointGrid, gridCellSize, pointMatchEps);
    
    // Se anche solo uno non viene trovato, scarto il triangolo.
    if(ia < 0 || ib < 0 || ic < 0)
      continue;
    
    // Evita triangoli degeneri lato indici.
    if(ia == ib || ib == ic || ic == ia)
      continue;
    
    ++edgeCount[makeEdgeKey(ia, ib)];
    ++edgeCount[makeEdgeKey(ib, ic)];
    ++edgeCount[makeEdgeKey(ic, ia)];
    
  }
  
  boundaryEdges.reserve(edgeCount.size());
  
  // Gli edge di bordo compaiono una sola volta.
  for(const auto & item : edgeCount) {
    if(item.second == 1)
      boundaryEdges.push_back(item.first);
  }
  
  return boundaryEdges;
  
}

// Calcola l'area assoluta di un contorno definito come lista di indici
template <class T>
inline double contourAreaByIndex(const std::vector<int> & contour, const std::vector<T> & points) {
  
  if(contour.size() < 4)
    return 0.0;
  
  double area = 0.0;
  
  for(std::size_t i=0; i+1<contour.size(); ++i) {
    const T & p = points[contour[i]];
    const T & q = points[contour[i + 1]];
    area += static_cast<double>(p.x) * q.y - static_cast<double>(q.x) * p.y;
  }
  
  return 0.5 * std::abs(area);
  
}

// Ricostruisce tutti i contorni chiusi a partire dagli edge di bordo.
// Ogni contorno restituito è una sequenza ordinata di indici.
// L'ultimo indice coincide col primo, così il contorno è già "chiuso".
inline std::vector<std::vector<int>> extractClosedContoursByIndex(const std::vector<std::pair<int, int>> & boundaryEdges) {
  
  std::vector<std::vector<int>> contours;
  
  if(boundaryEdges.empty())
    return contours;
  
  // Costruisce il grafo di adiacenza dei vertici di bordo.
  std::unordered_map<int, std::vector<int>> adjacency;
  adjacency.reserve(boundaryEdges.size() * 2);
  
  for(const auto & edge : boundaryEdges) {
    adjacency[edge.first].push_back(edge.second);
    adjacency[edge.second].push_back(edge.first);
  }
  
  // In un contorno chiuso regolare, ogni vertice dovrebbe avere grado 2.
  // Se non succede, lo segnaliamo.
  bool irregularTopology = false;
  for(const auto & item : adjacency) {
    if(item.second.size() != 2) {
      irregularTopology = true;
      break;
    }
  }
  
  if(irregularTopology) {
    std::cerr << "[WARNING] Alpha shape: topologia del bordo non regolare (esistono vertici con grado diverso da 2)." << std::endl;
  }
  
  // Insieme degli edge non ancora visitati.
  std::unordered_set<std::pair<int, int>, utils::intPairHash_t> unvisitedEdges;
  unvisitedEdges.reserve(boundaryEdges.size() * 2);
  
  for(const auto & edge : boundaryEdges)
    unvisitedEdges.insert(makeEdgeKey(edge.first, edge.second));
  
  // Finché ci sono edge non visitati, provo a estrarre un contorno.
  while(!unvisitedEdges.empty()) {
    
    const auto seedEdge = *unvisitedEdges.begin();
    
    const int start = seedEdge.first;
    int current = start;
    int previous = -1;
    
    std::vector<int> contour;
    bool closed = false;
    
    while(true) {
      
      contour.push_back(current);
      
      const auto adjIt = adjacency.find(current);
      
      if(adjIt == adjacency.end() || adjIt->second.empty())
        break;
      
      int next = -1;
      
      // Primo tentativo, scegli un vicino con edge non visitato e diverso dal precedente.
      for(const int candidate : adjIt->second) {
        const auto edgeKey = makeEdgeKey(current, candidate);
        
        if(unvisitedEdges.find(edgeKey) == unvisitedEdges.end())
          continue;
        
        if(candidate != previous) {
          next = candidate;
          unvisitedEdges.erase(edgeKey);
          break;
        }
        
      }
      
      // Secondo tentativo, se il primo fallisce, accetta qualunque edge non visitato.
      if(next < 0) {
        
        for(const int candidate : adjIt->second) {
          
          const auto edgeKey = makeEdgeKey(current, candidate);
          
          if(unvisitedEdges.find(edgeKey) != unvisitedEdges.end()) {
            next = candidate;
            unvisitedEdges.erase(edgeKey);
            break;
          }
          
        }
        
      }
      
      // Se non c'è più nessun edge valido, il percorso si interrompe.
      if(next < 0)
        break;
      
      previous = current;
      current = next;
      
      // Se torno al punto iniziale, ho chiuso un ciclo.
      if(current == start) {
        contour.push_back(start);
        closed = true;
        break;
      }
      
    }
    
    // Un contorno valido deve avere almeno 3 vertici distinti
    // più il primo ripetuto in coda.
    if(closed && contour.size() >= 4)
      contours.push_back(std::move(contour));
    
  }
  
  return contours;
  
}

} // namespace utils


//*****************************************************************************
// alphaShape()
//*****************************************************************************
// Restituisce un vettore ordinato di punti lungo il contorno.
//
// Se esistono più contorni chiusi:
// - stampa un warning
// - restituisce quello con area maggiore
//
//*****************************************************************************
template <class T>
std::vector<cv::Point2f> alphaShape(const std::vector<T> & points, double alpha, float pointMatchEps = utils::kPointEpsilon, float gridCellSize = 1.0f) {
  
  if(points.size() < 4) {
    std::cerr << "[WARNING] Alpha shape: troppi pochi punti" << std::endl;
    return {};
  }
  
  // Estrazione degli edge di bordo.
  const auto boundaryEdges =  utils::computeBoundaryEdgesByIndex(points, alpha, pointMatchEps, gridCellSize);
  
  // Ricostruzione dei contorni chiusi.
  const auto contours = utils:: extractClosedContoursByIndex(boundaryEdges);
  
  if(contours.empty()) {
    std::cerr << "[WARNING] Alpha shape: nessun contorno chiuso trovato." << std::endl;
    return {};
  }
  
  // Se ci sono più contorni, scelgo quello con area maggiore.
  int bestContourIndex = 0;
  double bestArea =  utils::contourAreaByIndex(contours[0], points);
  
  for(int i=1; i<(int)contours.size(); ++i) {
    double area =  utils::contourAreaByIndex(contours[i], points);
    if(area > bestArea) {
      bestArea = area;
      bestContourIndex = i;
    }
  }
  
  if(contours.size() > 1) {
    std::cerr << "[WARNING] Alpha shape: trovati " << contours.size() << " contorni chiusi. Sara' restituito quello con area maggiore." << std::endl;
  }
  
  // Conversione finale: dagli indici del contorno ai punti originali PointT.
  std::vector<cv::Point2f> orderedBoundary;
  orderedBoundary.reserve(contours[bestContourIndex].size());
  
  for(int pointIndex : contours[bestContourIndex])
    orderedBoundary.push_back(points[pointIndex]);
  
  return orderedBoundary;
  
}

} /* namespace mpl */

#endif // _H_MPL_ALPHA_SHAPE_2D_H_





