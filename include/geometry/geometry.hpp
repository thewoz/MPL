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

#ifndef _H_MPL_GEOMETRIC_H_
#define _H_MPL_GEOMETRIC_H_

#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include <mpl/stdio.hpp>
#include <mpl/utils.hpp>
#include <mpl/clustering.hpp>
#include <mpl/geometry/kabsch.hpp>
#include <mpl/neighbors.hpp>
#include <mpl/vision/point4d.hpp>

//****************************************************************************/
// mpl::math::fit
//****************************************************************************/
namespace mpl::math::fit {


  //****************************************************************************/
  // parabola() - ChatGPT
  //****************************************************************************/
  template <class T>
  void parabola(const std::vector<T> & points, cv::Vec3d & coeffs, const cv::Point2d & offset = cv::Point2d(0,0)) {

      // Create matrices for the system of linear equations
      cv::Mat A((int)points.size(), 3, CV_64F);
      cv::Mat B((int)points.size(), 1, CV_64F);

      // Fill matrices A and B
      for(int i=0; i<points.size(); ++i) {
        double x = points[i].x - offset.x;
        double y = points[i].y - offset.y;
        A.at<double>(i, 0) = x * x;
        A.at<double>(i, 1) = x;
        A.at<double>(i, 2) = 1;
        B.at<double>(i, 0) = y;
      }

      // Solve the system A * coeffs = B
      cv::Mat coeffsMat;
      cv::solve(A, B, coeffsMat, cv::DECOMP_SVD);

      // Store the result in coeffs
      coeffs = cv::Vec3d(coeffsMat.at<double>(0, 0), coeffsMat.at<double>(1, 0), coeffsMat.at<double>(2, 0));
    
  }


  //****************************************************************************/
  // linear()
  //****************************************************************************/
  double linear(const std::vector<cv::Point2d> & points, cv::Vec2d & coeff) {
    
    double sumx  = 0.0;
    double sumx2 = 0.0;
    double sumy  = 0.0;
    double sumxy = 0.0;
    
    for(size_t i=0; i<points.size(); ++i) {
      sumx  += points[i].x;
      sumx2 += points[i].x * points[i].x;
      sumy  += points[i].y;
      sumxy += points[i].x * points[i].y;
    }
    
    // f(x) = ax + b
    coeff[0] = (points.size()*sumxy - sumx*sumy)  / (points.size()*sumx2 - sumx*sumx);
    coeff[1] = (sumy*sumx2 - sumx*sumxy) / (points.size()*sumx2 - sumx*sumx);
    
    double error = 0;
    
    for(int i=0; i<points.size(); ++i) {
      
      error += (points[i].y - ((coeff[0]*points[i].x) + coeff[1])) * (points[i].y - ((coeff[0]*points[i].x) + coeff[1]));
      
    }
    
    error /= (double)points.size();
    
    return error;
    
  }

  //****************************************************************************/
  // linear()
  //****************************************************************************/
  double linear(const std::vector<double> & x, const std::vector<double> & y, cv::Vec2d & coeff) {
    
    double sumx  = 0.0;
    double sumx2 = 0.0;
    double sumy  = 0.0;
    double sumxy = 0.0;
    
    if(x.size() != y.size()) {
      fprintf(stderr, "error linear() x and y must have the same lenght\n");
      abort();
    }
    
    size_t size = x.size();
    
    for(size_t i=0; i<size; ++i) {
      sumx  += x[i];
      sumx2 += x[i] * x[i];
      sumy  += y[i];
      sumxy += x[i] * y[i];
    }
    
    // f(x) = ax + b
    coeff[0] = (size*sumxy - sumx*sumy)  / (size*sumx2 - sumx*sumx);
    coeff[1] = (sumy*sumx2 - sumx*sumxy) / (size*sumx2 - sumx*sumx);
    
    double error = 0;
    
    for(int i=0; i<size; ++i) {
      
      error += (y[i] - ((coeff[0]*x[i]) + coeff[1])) * (y[i] - ((coeff[0]*x[i]) + coeff[1]));
      
    }
    
    error /= (double)size;
    
    return error;
    
  }

  //****************************************************************************/
  // orear()
  //****************************************************************************/
  double orear(const std::vector<cv::Point2d> & points, const std::vector<cv::Point2d> & sigma, cv::Vec2d & coeff, double & energy, int maxIteration = 10) {
    
    // Forse gli si puo dare una pulita
    double oldA = coeff[1];
    double oldB = coeff[0];
    
    double ao = coeff[1];
    double bo = coeff[0];
    
    for(int j=0; j<maxIteration; ++j) {
      
      double t1 = 0; double t2 = 0; double t3 = 0; double t4 = 0; double t5 = 0;
      
      for(int i=0; i<points.size(); ++i) {
        
        double wi = 1 / ((sigma[i].y*sigma[i].y) + ((bo*bo)*(sigma[i].x*sigma[i].x)));
        
        t1 += wi;
        t2 += wi * points[i].x * points[i].y;
        t3 += wi * points[i].x;
        t4 += wi * points[i].y;
        t5 += wi * points[i].x * points[i].x;
        
      }
      
      bo = (t1 * t2 - t3 * t4) / (t1 * t5 - t3 * t3);
      ao = (t4 - (bo * t3))  / t1;
      
      if(std::abs(oldA - ao) <= 0.0000001 && std::abs(oldB - bo) <= 0.0000001 ) {
        break;
      }
      
      coeff[1] = ao;
      coeff[0] = bo;
      
    }
    
    double error  = 0;
    energy = 0;
    
    for(int i=0; i<points.size(); ++i) {
      
      error += (points[i].y - ((coeff[0]*points[i].x) + coeff[1])) * (points[i].y - ((coeff[0]*points[i].x) + coeff[1]));
      
      double wi = 1 / ((sigma[i].y*sigma[i].y) + ((coeff[0]*coeff[0])*(sigma[i].x*sigma[i].x)));
      
      energy += wi * (points[i].y - ((coeff[0]*points[i].x) + coeff[1])) * (points[i].y - ((coeff[0]*points[i].x) + coeff[1]));
    }
    
    error /= (double)points.size();
    
    return error;
    
  }

} /* namespace mpl::math::fit */


//****************************************************************************/
// geometry
//****************************************************************************/
namespace mpl::geometry {

  //****************************************************************************/
  //  barycenter
  //****************************************************************************/
  template <class T>
  inline T barycenter(const std::vector<T> & points) {
    
    T barycenter;
    
    for(size_t i=0; i<points.size(); ++i){
      
      barycenter += points[i];
      
    }
    
    barycenter /= (double) points.size();
    
    return barycenter;
    
  }

  //*****************************************************************************/
  //  dotAngle
  //*****************************************************************************/
  inline float dotAngle(const cv::Point3f & vec1, const cv::Point3f & vec2){
    
    float vec1vec2 = (vec1.x*vec2.x)+(vec1.y*vec2.y)+(vec1.z*vec2.z);
    
    float vec1Mod = sqrt((vec1.x*vec1.x)+(vec1.y*vec1.y)+(vec1.z*vec1.z));
    float vec2Mod = sqrt((vec2.x*vec2.x)+(vec2.y*vec2.y)+(vec2.z*vec2.z));
    
    float modVec1Vec2 = vec1Mod * vec2Mod;
    
    if(modVec1Vec2 <= FLT_EPSILON) return 1;
    
    // if(std::isnan(vec1vec2 / modVec1Vec2)){ return 1; }
    
    return (vec1vec2 / modVec1Vec2);
    
  }

  //*****************************************************************************/
  //  dotAngle
  //*****************************************************************************/
  inline float dotAngle(const cv::Point2f & vec1, const cv::Point2f & vec2){
    
    float vec1vec2 = (vec1.x*vec2.x)+(vec1.y*vec2.y);
    
    float vec1Mod = sqrt((vec1.x*vec1.x)+(vec1.y*vec1.y));
    float vec2Mod = sqrt((vec2.x*vec2.x)+(vec2.y*vec2.y));
    
    float modVec1Vec2 = vec1Mod * vec2Mod;
    
    if(modVec1Vec2 <= FLT_EPSILON) return 1;
    
    // if(std::isnan(vec1vec2 / modVec1Vec2)){ return 1; }
    
    return (vec1vec2 / modVec1Vec2);
    
  }

  //****************************************************************************//
  // computeRotationalMatrix
  //****************************************************************************//
  template<typename T>
  inline cv::Mat computeRotationalMatrix(T roll, T pitch, T yaw) {
    
    cv::Mat RX = (cv::Mat_<T>(3,3) << 1, 0, 0,
                  0, cos(roll), -sin(roll),
                  0, sin(roll),  cos(roll));
    
    cv::Mat RY = (cv::Mat_<T>(3,3) << cos(pitch),  0, -sin(pitch),
                  0,  1,           0,
                  sin(pitch), 0,  cos(pitch));
    
    cv::Mat RZ = (cv::Mat_<T>(3,3) << cos(yaw), -sin(yaw), 0,
                  sin(yaw),  cos(yaw), 0,
                  0,         0, 1);
    
    return RX * RY * RZ;
    
  }

  template<typename T> inline cv::Mat computeRotationalMatrix(const cv::Point3_<T> & pt) { return computeRotationalMatrix(pt.x, pt.y, pt.z); }
  template<typename T> inline cv::Mat computeRotationalMatrix(const cv::Vec<T,3> & vec)  { return computeRotationalMatrix(vec[0], vec[1], vec[2]); }
  template<typename T> inline cv::Mat computeRotationalMatrix(const T * vec)             { return computeRotationalMatrix(vec[0], vec[1], vec[2]); }
  template<typename T> inline cv::Mat computeRotationalMatrix(const T & vec)             { return computeRotationalMatrix(vec[0], vec[1], vec[2]); }

  //*****************************************************************************/
  //   namespace utils of applyRTS
  //*****************************************************************************/
  namespace utilsApplyRTS {

    template <typename TT>
    void _applyRTS(std::vector<TT> & P, const cv::Point3d & p0, const double * R, const double * T, double S) {
      
      cv::Point3d tmpPoint;
      
      for(int i=0; i<P.size(); ++i){
        
        tmpPoint.x = (((R[0]*(P[i].x-p0.x)) + (R[1]*(P[i].y-p0.y)) + (R[2]*(P[i].z-p0.z))) * S) + T[0] + p0.x;
        tmpPoint.y = (((R[3]*(P[i].x-p0.x)) + (R[4]*(P[i].y-p0.y)) + (R[5]*(P[i].z-p0.z))) * S) + T[1] + p0.y;
        tmpPoint.z = (((R[6]*(P[i].x-p0.x)) + (R[7]*(P[i].y-p0.y)) + (R[8]*(P[i].z-p0.z))) * S) + T[2] + p0.z;
        
        P[i].x = tmpPoint.x;
        P[i].y = tmpPoint.y;
        P[i].z = tmpPoint.z;
        
      }
      
    }

    template <typename TT>
    void _applyRTS(TT & P, const cv::Point3d & p0, const double * R, const double * T, double S) {
      
      cv::Point3d tmpPoint;
      
      tmpPoint.x = (((R[0]*(P.x-p0.x)) + (R[1]*(P.y-p0.y)) + (R[2]*(P.z-p0.z))) * S) + T[0] + p0.x;
      tmpPoint.y = (((R[3]*(P.x-p0.x)) + (R[4]*(P.y-p0.y)) + (R[5]*(P.z-p0.z))) * S) + T[1] + p0.y;
      tmpPoint.z = (((R[6]*(P.x-p0.x)) + (R[7]*(P.y-p0.y)) + (R[8]*(P.z-p0.z))) * S) + T[2] + p0.z;
      
      P.x = tmpPoint.x;
      P.y = tmpPoint.y;
      P.z = tmpPoint.z;
      
    }

    template <typename TT>
    void _applyRTS(std::vector<TT> & P, const cv::Point2d & p0, const double * R, const double * T, double S) {
      
      cv::Point2d tmpPoint;
      
      for(int i=0; i<P.size(); ++i){
        
        tmpPoint.x = (((R[0]*(P[i].x-p0.x)) + (R[1]*(P[i].y-p0.y))) * S) + T[0] + p0.x;
        tmpPoint.y = (((R[2]*(P[i].x-p0.x)) + (R[3]*(P[i].y-p0.y))) * S) + T[1] + p0.y;
        
        P[i].x = tmpPoint.x;
        P[i].y = tmpPoint.y;
        
      }
      
    }

    template <typename TT>
    void _applyRTS(TT & P, const cv::Point2d & p0, const double * R, const double * T, double S) {
      
      cv::Point2d tmpPoint;
      
      tmpPoint.x = (((R[0]*(P.x-p0.x)) + (R[1]*(P.y-p0.y))) * S) + T[0] + p0.x;
      tmpPoint.y = (((R[2]*(P.x-p0.x)) + (R[3]*(P.y-p0.y))) * S) + T[1] + p0.y;
      
      P.x = tmpPoint.x;
      P.y = tmpPoint.y;
      
    }

  } /* namespace utilsApplyRTS */

  //*****************************************************************************/
  // applyRTS
  //*****************************************************************************/
  template <typename TT>
  inline void applyRTS3D(std::vector<TT> & P, const cv::Point3d & p0, cv::Mat & R, cv::Mat & T, double S = 1.0) {
    
    utilsApplyRTS::_applyRTS(P, p0, (double*) R.data, (double*) T.data, S);
    
  }

  template <typename TT>
  inline void applyRTS3D(TT & P, const cv::Point3d & p0, cv::Mat & R, cv::Mat & T, double S = 1.0) {
    
    utilsApplyRTS::_applyRTS(P, p0, (double*) R.data, (double*) T.data, S);
    
  }

  //*****************************************************************************/
  // applyRTS
  //*****************************************************************************/
  template <typename TT>
  inline void applyRTS3D(std::vector<TT> & P, cv::Mat & R, cv::Mat & T, double S = 1.0) {
    
    cv::Point3d p0 = cv::Point3d(0.0, 0.0, 0.0);
    
    for(std::size_t i=0; i<P.size(); ++i)
      p0 += P[i];
    
    p0 /= (double)P.size();
    
    utilsApplyRTS::_applyRTS(P, p0, (double*)R.data, (double*)T.data, S);
    
  }

  //*****************************************************************************/
  // applyRTS
  //*****************************************************************************/
  template <typename TT>
  inline void applyRTS2D(std::vector<TT> & P, const cv::Point2d & p0, cv::Mat & R, cv::Mat & T, double S = 1.0) {
    
    utilsApplyRTS::_applyRTS(P, p0, (double*) R.data, (double*) T.data, S);
    
  }

  //*****************************************************************************/
  // applyRTS
  //*****************************************************************************/
  template <typename TT>
  inline void applyRTS2D(TT & P, const cv::Point2d & p0, cv::Mat & R, cv::Mat & T, double S = 1.0) {
    
    utilsApplyRTS::_applyRTS(P, p0, (double*) R.data, (double*) T.data, S);
    
  }

  //*****************************************************************************/
  // applyRTS
  //*****************************************************************************/
  template <typename TT>
  inline void applyRTS2D(std::vector<TT> & P, cv::Mat & R, cv::Mat & T, double S = 1.0) {
    
    cv::Point2d p0 = cv::Point2d(0.0, 0.0);
    
    for(std::size_t i=0; i<P.size(); ++i)
      p0 += P[i];
    
    p0 /= (double)P.size();
    
    utilsApplyRTS::_applyRTS(P, p0, (double*)R.data, (double*)T.data, S);
    
  }

  //*****************************************************************************/
  // rotate2D
  //*****************************************************************************/
  template <typename T>
  void rotate2D(T & P, double angleRad) {
    
    T tmp;
    
    tmp.x = std::cos(angleRad)*P.x - std::sin(angleRad)*P.y;
    tmp.y = std::sin(angleRad)*P.x + std::cos(angleRad)*P.y;
    
    P = tmp;
    
  }

  //*****************************************************************************/
  // rotate2D
  //*****************************************************************************/
  template <typename T>
  void rotate2D(std::vector<T> & P, double angleRad) {
    
    T tmp;
    
    for(size_t i=0; i<P.size(); ++i) {
      tmp.x = std::cos(angleRad)*P[i].x - std::sin(angleRad)*P[i].y;
      tmp.y = std::sin(angleRad)*P[i].x + std::cos(angleRad)*P[i].y;
      P[i] = tmp;
    }
    
  }

  //*****************************************************************************/
  // rotate2D
  //*****************************************************************************/
  template <typename T>
  void rotate2D(std::vector<T> & P, const std::vector<double> & angleRad) {
    
    T tmp;
    
    for(size_t i=0; i<P.size(); ++i) {
      tmp.x = std::cos(angleRad[i])*P[i].x - std::sin(angleRad[i])*P[i].y;
      tmp.y = std::sin(angleRad[i])*P[i].x + std::cos(angleRad[i])*P[i].y;
      P[i] = tmp;
    }
    
  }

  //*****************************************************************************/
  // rotate
  //*****************************************************************************/
  template <typename T, typename TT>
  inline void rotate2D(std::vector<T> & P, const TT & center, const std::vector<double> & angleRad) {
    
    for(size_t i=0; i<P.size(); ++i)
      P[i] -= center;
    
    rotate2D(P, angleRad);
    
    for(size_t i=0; i<P.size(); ++i)
      P[i] += center;
    
  }

  //*****************************************************************************/
  // rotate
  //*****************************************************************************/
  template <typename T, typename TT>
  inline void rotate2D(T & P, const TT & center, double angleRad) {
    
    P -= center;
    
    rotate2D(P, angleRad);
    
    P += center;
    
  }

  //*****************************************************************************/
  // rotate
  //*****************************************************************************/
  template <typename T, typename TT>
  inline void rotate2D(std::vector<T> & P, const TT & center, double angleRad) {
    
    for(size_t i=0; i<P.size(); ++i)
      P[i] -= center;
    
    rotate2D(P, angleRad);
    
    for(size_t i=0; i<P.size(); ++i)
      P[i] += center;
    
  }

  /*/****************************************************************************/
  //  intersection | between two lines
  //*****************************************************************************/
  // Retta ax + by + c = 0
  template <class T>
  inline bool intersection(const cv::Vec3d & lineA, const cv::Vec3d & lineB, cv::Point_<T> & point) {
    
    double det = (lineA[0]*lineB[1]) - (lineA[1]*lineB[0]);
    
    if(fabs(det) <= FLT_EPSILON) return false;
    
    double invDet = 1.0 / det;
    
    point.x = invDet * (lineA[1]*lineB[2]-lineA[2]*lineB[1]);
    point.y = invDet * (lineA[2]*lineB[0]-lineA[0]*lineB[2]);
    
    return true;
    
  }

  //****************************************************************************/
  //  distance
  //****************************************************************************/
  namespace distance {

    //****************************************************************************/
    //  dist | between a point and point
    //****************************************************************************/
    template <typename TP1, typename TP2>
    inline double fromPoint(const TP1 & point1, const TP2 & point2) {
      
      return cv::norm(point1 - point2);
      
    }
  
    //****************************************************************************/
    //  dist | between a point and a lines powed
    //****************************************************************************/
    template <typename TP, typename TL>
    inline double fromLine(const TP & point, const cv::Vec<TL, 3> & line){
      
      // linea ortogonale a "line" passante per point
      cv::Vec3f lineP = cv::Vec3f(line[1], -line[0], (line[0]*point.y)-(line[1]*point.x));
      
      TP cross;
      
      if(!intersection(line, lineP, cross)) return INFINITY;
      
      return cv::norm(point - cross);
      
    }

    //****************************************************************************/
    //  dist | between a point and a lines powed
    //****************************************************************************/
    template <typename TP, typename TL>
    inline double fromLine(const TP & point, const cv::Vec<TL, 2> & line){
      
      return abs(point.y - ((line[0]*point.x) + line[1])) / sqrt(1+(line[0]*line[0]));
      
    }

    //*****************************************************************************/
    //  dist | between a point 4D and a plane
    //*****************************************************************************/
    template <typename TP, typename TL>
    inline double fromPlane(const point4d_t & _point, const TL * coeff) {
      
      if(_point.w == 0) return 0;
      
      cv::Point3d point;
      
      point.x = _point.x / _point.w;
      point.y = _point.y / _point.w;
      point.z = _point.z / _point.w;
      
      double value = abs((coeff[0]*point.x)+(coeff[1]*point.y)+(coeff[2]*point.z)+coeff[3]) / std::sqrt((coeff[0]*coeff[0])+(coeff[1]*coeff[1])+(coeff[2]*coeff[2]));
      
      return value;
      
    }

  } /* namespace distance */

  //*****************************************************************************/
  // findBestRTS
  //*****************************************************************************/
  template <int dim, typename type, typename type_p0>
  void findBestRTS(const std::vector<type> & pointsA, const std::vector<type> & pointsB, type_p0 & p0, cv::Mat & R, cv::Mat & T, double & S, double distanceNNFactor, uint32_t maxIter = 100){
    
  #if(0)
    
    // Calcolo il baricentro dei due set di punti
    type_p0 baricenterA = mpl::geometry::barycenter(pointsA);
    type_p0 baricenterB = mpl::geometry::barycenter(pointsB);
    
    type_p0 offset = baricenterB - baricenterA;
    
    //std::cout << "baricenter offset " << offset << std::endl;
    
    std::vector<type> firstMovedPointsA = pointsA;
    
    for(size_t i=0; i<pointsA.size(); ++i) firstMovedPointsA[i] += offset;
    
    // mi calcolo la distanza NN tra i punti in A e quelli in B
    double NNdist = mpl::utils::NNDistance(firstMovedPointsA, pointsB) * 1.5;
    
    // printf("NNdist %f\n", NNdist);
    
    // mi trovo i vicini in base alla distanza
    std::vector<std::vector<uint32_t> > match = mpl::neighbors::byDistance(firstMovedPointsA, pointsB, NNdist, 1);
    
  #endif
    
  #if(1)
    
    // mi calcolo la distanza NN tra i punti in A e quelli in B
    double NNdist = mpl::utils::NNDistance(pointsA, pointsB) * distanceNNFactor;
    
    //NNdist = 0.16;
    
    //printf("NNdist %f\n", NNdist);
    
    // mi trovo i vicini in base alla distanza
    std::vector<std::vector<uint32_t> > match = mpl::neighbors::byDistance(pointsA, pointsB, NNdist, 1);
    
  #endif
    
    // numero di coppie 1 a 1 tra i punti in A e in B
    uint32_t size = 0;
    
    for(uint32_t i=0; i<match.size(); ++i) if(match[i].size() == 1) ++size;
    
    //printf("Monogami %u\n", size);
    
    if constexpr (dim==3) { if(size == 0) { T = cv::Mat::zeros(3,1,CV_64F);  R = cv::Mat::eye(3,3,CV_64F); S = 1; return; } }
    if constexpr (dim==2) { if(size == 0) { T = cv::Mat::zeros(2,1,CV_64F);  R = cv::Mat::eye(2,2,CV_64F); S = 1; return; } }
    
    uint32_t iter = 1;
    
    double diff = DBL_MAX;
    
    while(diff > DBL_EPSILON && iter < maxIter) {
      
      // copio le coppie 1 a 1 in P e Q
      cv::Mat P = cv::Mat(dim, size, CV_64F);
      cv::Mat Q = cv::Mat(dim, size, CV_64F);
      
      uint32_t index = 0;
      
      for(uint32_t i=0; i<match.size(); ++i) {
        
        if(match[i].size() == 1) {
          
          uint32_t j = match[i][0];
          
          if constexpr (dim==3) {
            P.at<double>(0,index) = pointsA[i].x;
            P.at<double>(1,index) = pointsA[i].y;
            P.at<double>(2,index) = pointsA[i].z;
            
            Q.at<double>(0,index) = pointsB[j].x;
            Q.at<double>(1,index) = pointsB[j].y;
            Q.at<double>(2,index) = pointsB[j].z;
          }
          
          if constexpr (dim==2) {
            P.at<double>(0,index) = pointsA[i].x;
            P.at<double>(1,index) = pointsA[i].y;
            
            Q.at<double>(0,index) = pointsB[j].x;
            Q.at<double>(1,index) = pointsB[j].y;
            
            //printf("%f %f - %f %f\n", pointsA[i].x, pointsA[i].y, pointsB[j].x, pointsB[j].y);
            
          }
          
          ++index;
          
        }
        
      }
      
      // cerco la migliore rota-traslazione e scaling tra i punti
      if constexpr (dim==3) p0 = geometry::kabsch::solve3D(P, Q, R, T, &S);
      if constexpr (dim==2) p0 = geometry::kabsch::solve2D(P, Q, R, T, &S);
      
      // applico e riruoto
      std::vector<type> movedPointsA = pointsA;
      
      if constexpr (dim==3) geometry::applyRTS3D(movedPointsA, p0, R, T, S);
      if constexpr (dim==2) geometry::applyRTS2D(movedPointsA, p0, R, T, S);
      
      double newNNdist = mpl::utils::NNDistance(movedPointsA, pointsB) * distanceNNFactor;
      
      // mi trovo i vicini in base alla distanza
      match = mpl::neighbors::byDistance(movedPointsA, pointsB, newNNdist, 1);
      
      // numero di coppie 1 a 1 tra i punti in A e in B
      uint32_t tmpSize = 0;
      
      for(uint32_t i=0; i<match.size(); ++i) if(match[i].size() == 1) ++tmpSize;
      
      if(tmpSize == 0) { return; }
      
      diff = fabs(newNNdist-NNdist);
      
      NNdist = newNNdist;
      
      //printf("%d %f %u\n", iter, NNdist, tmpSize);
      
      size = tmpSize;
      
      ++iter;
      
    }
    
  }

  //*****************************************************************************/
  // minDistLines
  //*****************************************************************************
  // Find intersection point of lines in 3D space, in the least squares sense.
  //   vettA:          Nx3-matrix containing starting point of N lines
  //   vettB:          Nx3-matrix containing end point of N lines
  //   point:          the best intersection point of the N lines, in least squares sense
  // Convert from the Anders Eikenes 2012 Matlab code
  //******************************************************************************/
  template <class T>
  void minDistLines(const std::vector<T> & vettA, const std::vector<T> & vettB, T & point) {
    
    std::vector<double> nx(vettB.size());
    std::vector<double> ny(vettB.size());
    std::vector<double> nz(vettB.size());
    
    for(size_t i=0; i<vettB.size(); ++i){
      nx[i] = vettB[i].x;
      ny[i] = vettB[i].y;
      nz[i] = vettB[i].z;
    }
    
    double SXX = 0; for(size_t i=0; i<nx.size(); ++i){ SXX += nx[i]*nx[i]-1; }
    double SYY = 0; for(size_t i=0; i<ny.size(); ++i){ SYY += ny[i]*ny[i]-1; }
    double SZZ = 0; for(size_t i=0; i<nz.size(); ++i){ SZZ += nz[i]*nz[i]-1; }
    
    double SXY = 0; for(size_t i=0; i<nx.size(); ++i){ SXY += nx[i]*ny[i]; }
    double SXZ = 0; for(size_t i=0; i<ny.size(); ++i){ SXZ += nx[i]*nz[i]; }
    double SYZ = 0; for(size_t i=0; i<nz.size(); ++i){ SYZ += ny[i]*nz[i]; }
    
    cv::Mat S = cv::Mat(cv::Size(3,3), CV_64FC1);
    S.at<double>(0,0) = SXX; S.at<double>(0,1) = SXY; S.at<double>(0,2) = SXZ;
    S.at<double>(1,0) = SXY; S.at<double>(1,1) = SYY; S.at<double>(1,2) = SYZ;
    S.at<double>(2,0) = SXZ; S.at<double>(2,1) = SYZ; S.at<double>(2,2) = SZZ;
    
    double CX = 0;
    double CY = 0;
    double CZ = 0;
    
    for(size_t i=0; i<vettA.size(); ++i) {
      
      double CX1 = vettA[i].x * ((nx[i]*nx[i]) - 1.0);
      double CX2 = vettA[i].y * (nx[i]*ny[i]);
      double CX3 = vettA[i].z * (nx[i]*nz[i]);
      
      double CY1 = vettA[i].x * (nx[i]*ny[i]);
      double CY2 = vettA[i].y * ((ny[i]*ny[i]) - 1.0);
      double CY3 = vettA[i].z * (ny[i]*nz[i]);
      
      double CZ1 = vettA[i].x * (nx[i]*nz[i]);
      double CZ2 = vettA[i].y * (ny[i]*nz[i]);
      double CZ3 = vettA[i].z * ((nz[i]*nz[i]) - 1.0);
      
      CX += CX1 + CX2 + CX3;
      CY += CY1 + CY2 + CY3;
      CZ += CZ1 + CZ2 + CZ3;
      
    }
    
    cv::Mat C = cv::Mat(cv::Size(1,3), CV_64FC1);
    C.at<double>(0,0) = CX;
    C.at<double>(1,0) = CY;
    C.at<double>(2,0) = CZ;
    
    cv::Mat R;
    cv::solve(S, C, R, cv::DECOMP_NORMAL);
    
    point.x = R.at<double>(0,0);
    point.y = R.at<double>(0,1);
    point.z = R.at<double>(0,2);
    
  }

  template <class T>
  T minDistLines(const std::vector<T> & vettA, const std::vector<T> & vettB) {
    
    T point;
    
    minDistLines(vettA, vettB, point);
    
    return point;
    
  }

  //****************************************************************************
  // rotationX
  //****************************************************************************
  // this function returns the rotation matrix related to a rotation of an angle "angle" about x-axis
  cv::Mat computeRotationalMatrixInX(double angle) {
    
    cv::Mat R(3, 3, CV_64F);
    
    R.at<double>(0,0) = 1; R.at<double>(0,1) = 0;          R.at<double>(0,2) = 0;
    R.at<double>(1,0) = 0; R.at<double>(1,1) = cos(angle); R.at<double>(1,2) = -sin(angle);
    R.at<double>(2,0) = 0; R.at<double>(2,1) = sin(angle); R.at<double>(2,2) =  cos(angle);
    
    return R;
    
  }

  //****************************************************************************
  // rotationY
  //****************************************************************************
  // this function returns the rotation matrix related to a rotation of an angle "angle" about y-axis
  Mat computeRotationalMatrixInY(double angle){
    
    cv::Mat R(3, 3, CV_64F);
    
    R.at<double>(0,0) =  cos(angle); R.at<double>(0,1) = 0; R.at<double>(0,2) = sin(angle);
    R.at<double>(1,0) = 0;           R.at<double>(1,1) = 1; R.at<double>(1,2) = 0;
    R.at<double>(2,0) = -sin(angle); R.at<double>(2,1) = 0; R.at<double>(2,2) = cos(angle);
    
    return R;
    
  }

  //****************************************************************************
  // rotationZ
  //****************************************************************************
  // this function returns the rotation matrix related to a rotation of an angle "angle" about z-axis
  cv::Mat computeRotationalMatrixInZ(double angle){
    
    cv::Mat R(3, 3, CV_64F);
    
    R.at<double>(0,0) = cos(angle); R.at<double>(0,1) = -sin(angle); R.at<double>(0,2) = 0;
    R.at<double>(1,0) = sin(angle); R.at<double>(1,1) =  cos(angle); R.at<double>(1,2) = 0;
    R.at<double>(2,0) = 0;          R.at<double>(2,1) = 0;           R.at<double>(2,2) = 1;
    
    return R;
    
  }

  //****************************************************************************
  // getTranslationFromSphericalAngle
  //****************************************************************************
  cv::Mat getTranslationFromSphericalAngle(double d, double delta, double epsilon){
    
    cv::Mat T(3, 1, CV_64F);
    
    T.at<double>(0) = d * cos(epsilon);
    T.at<double>(1) = d * sin(epsilon) * sin(delta);
    T.at<double>(2) = d * sin(epsilon) * cos(delta);
    
    return T;
    
  }



//*****************************************************************************/
// getAxes
//*****************************************************************************/
// MUOVERE DA QUI
//  void getAxes(const cv::Mat & R, cv::Mat & axes) {
//
//     gsl_matrix *A = gsl_matrix_alloc(3,3);
//
//     gsl_matrix_memcpy(A, U);
//
//     gsl_vector_complex *eval = gsl_vector_complex_alloc(3);
//
//     gsl_eigen_nonsymmv_workspace *w = gsl_eigen_nonsymmv_alloc(3);
//
//     gsl_matrix_complex *evec = gsl_matrix_complex_alloc(3,3);
//
//     gsl_eigen_nonsymmv(A, eval, evec, w);
//
//     point3D_t Rotation_Axis;
//
//     for(int i = 0; i < 3; i++){
//
//       gsl_complex eval_i = gsl_vector_complex_get (eval, i);
//
//       gsl_vector_complex_view evec_i = gsl_matrix_complex_column (evec, i);
//
//       if(GSL_IMAG(eval_i) == 0){
//
//         Rotation_Axis.x = GSL_REAL(gsl_vector_complex_get(&evec_i.vector, 0));
//         Rotation_Axis.y = GSL_REAL(gsl_vector_complex_get(&evec_i.vector, 1));
//         Rotation_Axis.z = GSL_REAL(gsl_vector_complex_get(&evec_i.vector, 2));
//
//         Rotation_Axis.abs_value = Rotation_Axis.absolute_value();
//
//         for(int j = 0; j < 3; ++j) {
//           gsl_complex z = gsl_vector_complex_get(&evec_i.vector, j);
//           if(GSL_IMAG(z)!=0) fprintf(stderr, "warning not real eigenvector\n");
//         }
//
//       } else {
//         if(output!=NULL) fprintf(output, "%04d C %e %e %e\n", frame+min_frame, GSL_REAL(eval_i),GSL_IMAG(eval_i), atan(GSL_IMAG(eval_i)/(double)GSL_REAL(eval_i)));
//       }
//
//     }
//
//  }

} /* namespace geometry */

#endif /* _H_MPL_GEOMETRIC_H_ */





