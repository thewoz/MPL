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

#ifndef _H_MPL_GEOMETRY_GEOMETRY_H_
#define _H_MPL_GEOMETRY_GEOMETRY_H_

#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include <mpl/core/stdio.hpp>
#include <mpl/math/distance.hpp>
#include <mpl/math/clustering.hpp>
#include <mpl/math/kabsch.hpp>
#include <mpl/core/neighbors.hpp>
#include <mpl/vision/point4d.hpp>
#include <mpl/math/fit.hpp>

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

  //****************************************************************************/
  // computeRotationalMatrix
  //****************************************************************************/
  template<typename T>
  inline cv::Mat computeRotationalMatrix(T roll, T pitch, T yaw) {
    
    cv::Mat RX = (cv::Mat_<T>(3,3) << 1, 0, 0,
                  0, cos(roll), -sin(roll),
                  0, sin(roll),  cos(roll));
    
    cv::Mat RY = (cv::Mat_<T>(3,3) << cos(pitch),  0, -sin(pitch),
                  0,  1,           0,
                  -sin(pitch), 0,  cos(pitch));
    
    cv::Mat RZ = (cv::Mat_<T>(3,3) << cos(yaw), -sin(yaw), 0,
                  sin(yaw),  cos(yaw), 0,
                  0,         0, 1);
    
    return RX * RY * RZ;
    
  }

  template<typename T> inline cv::Mat computeRotationalMatrix(const cv::Point3_<T> & pt) { return computeRotationalMatrix(pt.x, pt.y, pt.z); }
  template<typename T> inline cv::Mat computeRotationalMatrix(const cv::Vec<T,3> & vec)  { return computeRotationalMatrix(vec[0], vec[1], vec[2]); }
  template<typename T> inline cv::Mat computeRotationalMatrix(const T * vec)             { return computeRotationalMatrix(vec[0], vec[1], vec[2]); }
  template<typename T> inline cv::Mat computeRotationalMatrix(const T & vec)             { return computeRotationalMatrix(vec[0], vec[1], vec[2]); }

  //****************************************************************************/
  // rotationX
  //****************************************************************************/
  // this function returns the rotation matrix related to a rotation of an angle "angle" about x-axis
  cv::Mat computeRotationalMatrixInX(double angle) {

    cv::Mat R(3, 3, CV_64F);

    R.at<double>(0,0) = 1; R.at<double>(0,1) = 0;          R.at<double>(0,2) = 0;
    R.at<double>(1,0) = 0; R.at<double>(1,1) = cos(angle); R.at<double>(1,2) = -sin(angle);
    R.at<double>(2,0) = 0; R.at<double>(2,1) = sin(angle); R.at<double>(2,2) =  cos(angle);

    return R;

  }

  //****************************************************************************/
  // rotationY
  //****************************************************************************/
  // this function returns the rotation matrix related to a rotation of an angle "angle" about y-axis
  cv::Mat computeRotationalMatrixInY(double angle){

    cv::Mat R(3, 3, CV_64F);

    R.at<double>(0,0) =  cos(angle); R.at<double>(0,1) = 0; R.at<double>(0,2) = sin(angle);
    R.at<double>(1,0) = 0;           R.at<double>(1,1) = 1; R.at<double>(1,2) = 0;
    R.at<double>(2,0) = -sin(angle); R.at<double>(2,1) = 0; R.at<double>(2,2) = cos(angle);

    return R;

  }

  //****************************************************************************/
  // rotationZ
  //****************************************************************************/
  // this function returns the rotation matrix related to a rotation of an angle "angle" about z-axis
  cv::Mat computeRotationalMatrixInZ(double angle){

    cv::Mat R(3, 3, CV_64F);

    R.at<double>(0,0) = cos(angle); R.at<double>(0,1) = -sin(angle); R.at<double>(0,2) = 0;
    R.at<double>(1,0) = sin(angle); R.at<double>(1,1) =  cos(angle); R.at<double>(1,2) = 0;
    R.at<double>(2,0) = 0;          R.at<double>(2,1) = 0;           R.at<double>(2,2) = 1;

    return R;

  }

  //*****************************************************************************/
  //   namespace detail of applyRTS
  //*****************************************************************************/
  namespace detail {

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

  } // namespace detail

  //*****************************************************************************/
  // applyRTS
  //*****************************************************************************/
  template <typename TT>
  inline void applyRTS3D(std::vector<TT> & P, const cv::Point3d & p0, const cv::Mat & R, const cv::Mat & T, double S = 1.0) {
    
    detail::_applyRTS(P, p0, (double*) R.data, (double*) T.data, S);
    
  }

  template <typename TT>
  inline void applyRTS3D(TT & P, const cv::Point3d & p0, const cv::Mat & R, const cv::Mat & T, double S = 1.0) {
    
    detail::_applyRTS(P, p0, (double*) R.data, (double*) T.data, S);
    
  }

  //*****************************************************************************/
  // applyRTS
  //*****************************************************************************/
  template <typename TT>
  inline void applyRTS3D(std::vector<TT> & P, const cv::Mat & R, const cv::Mat & T, double S = 1.0) {
    
    cv::Point3d p0 = cv::Point3d(0.0, 0.0, 0.0);
    
    for(std::size_t i=0; i<P.size(); ++i)
      p0 += P[i];
    
    p0 /= (double)P.size();
    
    detail::_applyRTS(P, p0, (double*)R.data, (double*)T.data, S);
    
  }

  //*****************************************************************************/
  // applyRTS
  //*****************************************************************************/
  template <typename TT>
  inline void applyRTS2D(std::vector<TT> & P, const cv::Point2d & p0, const cv::Mat & R, const cv::Mat & T, double S = 1.0) {
    
    detail::_applyRTS(P, p0, (double*) R.data, (double*) T.data, S);
    
  }

  //*****************************************************************************/
  // applyRTS
  //*****************************************************************************/
  template <typename TT>
  inline void applyRTS2D(TT & P, const cv::Point2d & p0, const cv::Mat & R, const cv::Mat & T, double S = 1.0) {
    
    detail::_applyRTS(P, p0, (double*) R.data, (double*) T.data, S);
    
  }

  //*****************************************************************************/
  // applyRTS
  //*****************************************************************************/
  template <typename TT>
  inline void applyRTS2D(std::vector<TT> & P, const cv::Mat & R, const cv::Mat & T, double S = 1.0) {
    
    cv::Point2d p0 = cv::Point2d(0.0, 0.0);
    
    for(std::size_t i=0; i<P.size(); ++i)
      p0 += P[i];
    
    p0 /= (double)P.size();
    
    detail::_applyRTS(P, p0, (double*)R.data, (double*)T.data, S);
    
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

  //*****************************************************************************/
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
  //  isInside
  //****************************************************************************/
  template <class T>
  bool isInside(const cv::Rect & rect, const T & point) {
    return (point.x >= rect.x) &&
            (point.x < rect.x + rect.width) &&
            (point.y >= rect.y) &&
            (point.y < rect.y + rect.height);
  }


  //*****************************************************************************/
  // findBestRTS
  //*****************************************************************************/
  template <int dim, typename ptT, typename pt0T>
  void findBestRTS(const std::vector<ptT> & pointsA, const std::vector<ptT> & pointsB, pt0T & p0, cv::Mat & R, cv::Mat & T, double & S, double distanceNNFactor, size_t maxIter = 100){

    // mi calcolo la distanza NN tra i punti in A e quelli in B
    double NNdist = mpl::distance::medianFirstNNDistance(pointsA, pointsB) * distanceNNFactor;

    //NNdist = 0.16;

    //printf("NNdist %f\n", NNdist);

    // mi trovo i vicini in base alla distanza
    std::vector<std::vector<size_t> > match = mpl::neighbors::byDistance(pointsA, pointsB, NNdist, 1);

    // numero di coppie 1 a 1 tra i punti in A e in B
    size_t size = 0;
    
    for(size_t i=0; i<match.size(); ++i) if(match[i].size() == 1) ++size;
    
    //printf("Monogami %u\n", size);
    
    if constexpr (dim==3) { if(size == 0) { T = cv::Mat::zeros(3,1,CV_64F);  R = cv::Mat::eye(3,3,CV_64F); S = 1; return; } }
    if constexpr (dim==2) { if(size == 0) { T = cv::Mat::zeros(2,1,CV_64F);  R = cv::Mat::eye(2,2,CV_64F); S = 1; return; } }
    
    size_t iter = 1;
    
    double diff = DBL_MAX;
    
    while(diff > DBL_EPSILON && iter < maxIter) {
      
      // copio le coppie 1 a 1 in P e Q
      cv::Mat P = cv::Mat(dim, size, CV_64F);
      cv::Mat Q = cv::Mat(dim, size, CV_64F);
      
      size_t index = 0;
      
      for(size_t i=0; i<match.size(); ++i) {
        
        if(match[i].size() == 1) {
          
          size_t j = match[i][0];
          
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
      if constexpr (dim==3) p0 = mpl::kabsch::solve3D(P, Q, R, T, &S);
      if constexpr (dim==2) p0 = mpl::kabsch::solve2D(P, Q, R, T, &S);
      
      // applico e riruoto
      std::vector<ptT> movedPointsA = pointsA;
      
      if constexpr (dim==3) geometry::applyRTS3D(movedPointsA, p0, R, T, S);
      if constexpr (dim==2) geometry::applyRTS2D(movedPointsA, p0, R, T, S);
      
      double newNNdist = mpl::distance::medianFirstNNDistance(movedPointsA, pointsB) * distanceNNFactor;
      
      // mi trovo i vicini in base alla distanza
      match = mpl::neighbors::byDistance(movedPointsA, pointsB, newNNdist, 1);
      
      // numero di coppie 1 a 1 tra i punti in A e in B
      size_t tmpSize = 0;
      
      for(size_t i=0; i<match.size(); ++i) if(match[i].size() == 1) ++tmpSize;
      
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
  //*****************************************************************************/
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

  //****************************************************************************/
  // getTranslationFromSphericalAngle
  //****************************************************************************/
  cv::Mat getTranslationFromSphericalAngle(double d, double delta, double epsilon){
    
    cv::Mat T(3, 1, CV_64F);
    
    T.at<double>(0) = d * cos(epsilon);
    T.at<double>(1) = d * sin(epsilon) * sin(delta);
    T.at<double>(2) = d * sin(epsilon) * cos(delta);
    
    return T;
    
  }

  //****************************************************************************/
  // getAxes
  //****************************************************************************/
  void getAxes(const cv::Mat & R, cv::Mat & axes) {
    
    CV_Assert(R.rows == 3 && R.cols == 3);
    CV_Assert(R.type() == CV_32F || R.type() == CV_64F);

    cv::Mat A;
    R.convertTo(A, CV_64F);

    // Caso identità o quasi-identità: asse non univoco
    if(cv::norm(A - cv::Mat::eye(3,3,CV_64F), cv::NORM_INF) < 1e-12) {
      axes = (cv::Mat_<double>(3,1) << 1.0, 0.0, 0.0);
      return;
    }

    cv::Mat eigenvalues, eigenvectors;
    cv::eigenNonSymmetric(A, eigenvalues, eigenvectors);

    int bestIdx = -1;
    double bestScore = std::numeric_limits<double>::infinity();

    for(int i=0; i<eigenvalues.rows; ++i) {
      double lambda = eigenvalues.at<double>(i, 0);
      double score = std::abs(lambda - 1.0);
      if(score < bestScore) {
        bestScore = score;
        bestIdx = i;
      }
    }

    axes = eigenvectors.row(bestIdx).t();

    double n = cv::norm(axes);
    if(n > 0.0) axes /= n;
    
  }


} // namespace geometry

#endif // _H_MPL_GEOMETRY_GEOMETRY_H_





