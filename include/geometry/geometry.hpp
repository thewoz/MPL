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

#ifndef _H_MPL_GEOMETRIC_H_
#define _H_MPL_GEOMETRIC_H_

#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include <mpl/stdio.hpp>
#include <mpl/opencv.hpp>
#include <mpl/utils.hpp>
#include <mpl/clustering.hpp>
#include <mpl/geometry/kabsch.hpp>
#include <mpl/neighbors.hpp>


/*****************************************************************************/
// geometry
/*****************************************************************************/
namespace mpl::geometry {
  
  
  /*****************************************************************************/
  //  dotAngle
  /*****************************************************************************/
  inline float dotAngle(const cv::Point3f & vec1, const cv::Point3f & vec2){
    
    float vec1vec2 = (vec1.x*vec2.x)+(vec1.y*vec2.y)+(vec1.z*vec2.z);
    
    float vec1Mod = sqrt((vec1.x*vec1.x)+(vec1.y*vec1.y)+(vec1.z*vec1.z));
    float vec2Mod = sqrt((vec2.x*vec2.x)+(vec2.y*vec2.y)+(vec2.z*vec2.z));
    
    float modVec1Vec2 = vec1Mod * vec2Mod;
    
    if(modVec1Vec2 <= FLT_EPSILON) return 1;
    
    // if(std::isnan(vec1vec2 / modVec1Vec2)){ return 1; }
    
    return (vec1vec2 / modVec1Vec2);
    
  }
  
  /*****************************************************************************/
  //  dotAngle
  /*****************************************************************************/
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
   
  
  /*****************************************************************************/
  //   namespace utils of applyRTS
  /*****************************************************************************/
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
    void _applyRTS(std::vector<TT> & P, const cv::Point2d & p0, const double * R, const double * T, double S) {
      
      cv::Point2d tmpPoint;
      
      for(int i=0; i<P.size(); ++i){
        
        tmpPoint.x = (((R[0]*(P[i].x-p0.x)) + (R[1]*(P[i].y-p0.y))) * S) + T[0] + p0.x;
        tmpPoint.y = (((R[2]*(P[i].x-p0.x)) + (R[3]*(P[i].y-p0.y))) * S) + T[1] + p0.y;
        
        P[i].x = tmpPoint.x;
        P[i].y = tmpPoint.y;
        
      }
      
    }
    
  }
  
  /*****************************************************************************/
  // applyRTS
  /*****************************************************************************/
  template <typename TT>
  inline void applyRTS3D(std::vector<TT> & P, const cv::Point3d & p0, cv::Mat & R, cv::Mat & T, double S = 1.0) {
    
    utilsApplyRTS::_applyRTS(P, p0, (double*) R.data, (double*) T.data, S);
    
  }
  
  /*****************************************************************************/
  // applyRTS
  /*****************************************************************************/
  template <typename TT>
  inline void applyRTS3D(std::vector<TT> & P, cv::Mat & R, cv::Mat & T, double S = 1.0) {
    
    cv::Point3d p0 = cv::Point3d(0.0, 0.0, 0.0);
    
    for(std::size_t i=0; i<P.size(); ++i)
      p0 += P[i];
    
    p0 /= P.size();

    utilsApplyRTS::_applyRTS(P, p0, (double*)R.data, (double*)T.data, S);
    
  }
  
  /*****************************************************************************/
  // applyRTS
  /*****************************************************************************/
  template <typename TT>
  inline void applyRTS2D(std::vector<TT> & P, const cv::Point2d & p0, cv::Mat & R, cv::Mat & T, double S = 1.0) {
    
    utilsApplyRTS::_applyRTS(P, p0, (double*) R.data, (double*) T.data, S);
    
  }
  
  /*****************************************************************************/
  // applyRTS
  /*****************************************************************************/
  template <typename TT>
  inline void applyRTS2D(std::vector<TT> & P, cv::Mat & R, cv::Mat & T, double S = 1.0) {
        
    cv::Point2d p0 = cv::Point2d(0.0, 0.0);
    
    for(std::size_t i=0; i<P.size(); ++i)
      p0 += P[i];
    
    p0 /= P.size();
    
    utilsApplyRTS::_applyRTS(P, p0, (double*)R.data, (double*)T.data, S);
    
  }

  /*****************************************************************************/
  // rotate2D
  /*****************************************************************************/
  template <typename T>
  void rotate2D(std::vector<T> & P, double angleRad) {
    
    T tmp;
    
    for(size_t i=0; i<P.size(); ++i) {
      tmp.x = std::cos(angleRad)*P[i].x - std::sin(angleRad)*P[i].y;
      tmp.y = std::sin(angleRad)*P[i].x + std::cos(angleRad)*P[i].y;
      P[i] = tmp;
    }
    
  }
  
  /*****************************************************************************/
  // rotate2D
  /*****************************************************************************/
  template <typename T>
  void rotate2D(std::vector<T> & P, const std::vector<double> & angleRad) {
    
    T tmp;
    
    for(size_t i=0; i<P.size(); ++i) {
      tmp.x = std::cos(angleRad[i])*P[i].x - std::sin(angleRad[i])*P[i].y;
      tmp.y = std::sin(angleRad[i])*P[i].x + std::cos(angleRad[i])*P[i].y;
      P[i] = tmp;
    }
    
  }
  
  /*****************************************************************************/
  // rotate
  /*****************************************************************************/
  template <typename T>
  inline void rotate2D(std::vector<T> & P, const T & center, const std::vector<double> & angleRad) {
  
    for(size_t i=0; i<P.size(); ++i)
      P[i] -= center;
    
    rotate2D(P, angleRad);
    
    for(size_t i=0; i<P.size(); ++i)
      P[i] += center;
  
  }

  /*****************************************************************************/
  // rotate
  /*****************************************************************************/
  template <typename T>
  inline void rotate2D(std::vector<T> & P, const T & center, double angleRad) {
  
    for(size_t i=0; i<P.size(); ++i)
      P[i] -= center;
    
    rotate2D(P, angleRad);
    
    for(size_t i=0; i<P.size(); ++i)
      P[i] += center;
  
  }
  
  /*****************************************************************************/
  //  intersection | between two lines
  /*****************************************************************************/
  // Retta ax + by + c = 0
  template <class T>
  inline bool intersection(const cv::Vec3d & lineA, const cv::Vec3d & lineB, cv::Point_<T> & point){
    
    double det = (lineA[0]*lineB[1]) - (lineA[1]*lineB[0]);
    
    if(fabs(det) <= FLT_EPSILON) return false;
    
    double invDet = 1.0 / det;
    
    point.x = invDet * (lineA[1]*lineB[2]-lineA[2]*lineB[1]);
    point.y = invDet * (lineA[2]*lineB[0]-lineA[0]*lineB[2]);
    
    return true;
    
  }

  
  namespace distance {
  
  
  /*****************************************************************************/
  //  dist | between a point and a lines powed
  /*****************************************************************************/
  template <typename TP, typename TL>
  inline double fromLine(const TP & point, const TL & line){
    
    //linea ortogonale a "line" passante per point
    cv::Vec3f lineP = cv::Vec3f(line[1], -line[0], (line[0]*point.y)-(line[1]*point.x));
    
    TP cross;
    
    if(!intersection(line, lineP, cross)) return INFINITY;
    
    return cv::norm(point, cross);
        
  }
  
  /*****************************************************************************/
  //  dist | between a point 4D and a plane
  /*****************************************************************************/
  template <typename TP, typename TL>
  inline double fromPlane(const cv::Point4_<TP> & _point, const TL * coeff) {
    
    if(_point.w == 0) return 0;
    
    cv::Point3d point;
    
    point.x = _point.x / _point.w;
    point.y = _point.y / _point.w;
    point.z = _point.z / _point.w;

    double value = abs((coeff[0]*point.x)+(coeff[1]*point.y)+(coeff[2]*point.z)+coeff[3]) / std::sqrt((coeff[0]*coeff[0])+(coeff[1]*coeff[1])+(coeff[2]*coeff[2]));
      
    return value;
    
  }
  
  }
  
  
  /*****************************************************************************/
  // findBestRTS
  /*****************************************************************************/
  template <int dim, typename type, typename type_p0>
  void findBestRTS(const type & pointsA, const type & pointsB, type_p0 & p0, cv::Mat & R, cv::Mat & T, double & S, double distanceNNFactor, uint32_t maxIter = 100){
    
    // mi calcolo la distanza NN tra i punti in A e quelli in B
    double NNdist = mpl::utils::NNDistance(pointsA, pointsB) * 1.5;
    
    // mi trovo i vicini in base alla distanza
    std::vector<std::vector<uint32_t> > match = mpl::neighbors::firstN(pointsA, pointsB, NNdist, 1);
    
    // numero di coppie 1 a 1 tra i punti in A e in B
    uint32_t size = 0;
    
    for(uint32_t i=0; i<match.size(); ++i) if(match[i].size() == 1) ++size;
    
    if constexpr (dim==3) { if(size == 0) { T = cv::Mat::zeros(3,1,CV_64F);  R = cv::Mat::eye(3,3,CV_64F); S = 1; return; printf("Monogami %u\n", size); } }
    if constexpr (dim==2) { if(size == 0) { T = cv::Mat::zeros(2,1,CV_64F);  R = cv::Mat::eye(2,2,CV_64F); S = 1; return; printf("Monogami %u\n", size); } }
    
    uint32_t iter = 1;
    
    double diff = DBL_MAX;
    
    while(diff > DBL_EPSILON && iter < maxIter){
      
      // copio le coppie 1 a 1 in P e Q
      cv::Mat P = cv::Mat(dim, size, CV_64F);
      cv::Mat Q = cv::Mat(dim, size, CV_64F);
      
      uint32_t index = 0;
      
      for(uint32_t i=0; i<match.size(); ++i){
        
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
          }
          
          ++index;
          
        }
        
      }
      
      // cerco la migliore rota-traslazione e scaling tra i punti
      if constexpr (dim==3) p0 = geometry::kabsch::solve3D(P, Q, R, T, &S);
      if constexpr (dim==2) p0 = geometry::kabsch::solve2D(P, Q, R, T, &S);
      
      // applico e riruoto
      type movedPointsA = pointsA;
      
      if constexpr (dim==3) geometry::applyRTS3D(movedPointsA, p0, R, T, S);
      if constexpr (dim==2) geometry::applyRTS2D(movedPointsA, p0, R, T, S);
      
      double newNNdist = mpl::utils::NNDistance(movedPointsA, pointsB) * 1.5;
      
      // mi trovo i vicini in base alla distanza
      match = mpl::neighbors::firstN(movedPointsA, pointsB, NNdist, 1);
      
      // numero di coppie 1 a 1 tra i punti in A e in B
      uint32_t tmpSize = 0;
      
      for(uint32_t i=0; i<match.size(); ++i) if(match[i].size() == 1) ++tmpSize;
      
      if(tmpSize == 0) { return; }
      
      diff = fabs(newNNdist-NNdist);
      
      NNdist = newNNdist;
      
      size = tmpSize;
      
      ++iter;
      
    }
    
  }
  
  /*****************************************************************************/
  // minDistLines
  /*****************************************************************************
  / Find intersection point of lines in 3D space, in the least squares sense.
  /   vettA:          Nx3-matrix containing starting point of N lines
  /   vettB:          Nx3-matrix containing end point of N lines
  /   point:          the best intersection point of the N lines, in least squares sense
  / Convert from the Anders Eikenes 2012 Matlab code
  ******************************************************************************/
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
  //this function returns the rotation matrix related to a rotation
  //of an angle "angle" about x-axis
  cv::Mat rotationX(double angle) {
    cv::Mat rotation(3,3,CV_64FC1);
    double cos_angle=cos(angle);
    double sin_angle=sin(angle);
    rotation=0.0;
    rotation.at<double>(0,0)=1.0;
    rotation.at<double>(1,1)=cos_angle;
    rotation.at<double>(1,2)=-sin_angle;
    rotation.at<double>(2,1)=sin_angle;
    rotation.at<double>(2,2)=cos_angle;
    return rotation;
  }
  
  //****************************************************************************
  // rotationY
  //****************************************************************************
  //this function returns the rotation matrix related to a rotation
  //of an angle "angle" about y-axis
  Mat rotationY(double angle){
    cv::Mat rotation(3,3,CV_64FC1);
    double cos_angle=cos(angle);
    double sin_angle=sin(angle);
    rotation=0.0;
    rotation.at<double>(0,0)=cos_angle;
    rotation.at<double>(0,2)=sin_angle;
    rotation.at<double>(1,1)=1.0;
    rotation.at<double>(2,0)=-sin_angle;
    rotation.at<double>(2,2)=cos_angle;
    return rotation;
  }
  
  //****************************************************************************
  // rotationZ
  //****************************************************************************
  //this function returns the rotation matrix related to a rotation
  //of an angle "angle" about z-axis
  cv::Mat rotationZ(double angle){
    cv::Mat rotation(3,3,CV_64FC1);
    double cos_angle=cos(angle);
    double sin_angle=sin(angle);
    rotation=0.0;
    rotation.at<double>(0,0)=cos_angle;
    rotation.at<double>(0,1)=-sin_angle;
    rotation.at<double>(1,0)=sin_angle;
    rotation.at<double>(1,1)=cos_angle;
    rotation.at<double>(2,2)=1.0;
    return rotation;
  }
  

  
} /* namespace geometry */

#endif /* _H_MPL_GEOMETRIC_H_ */





