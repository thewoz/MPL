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

#ifndef _H_COBBS_GEOMETRIC_H_
#define _H_COBBS_GEOMETRIC_H_

#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include <mpl/utils.hpp>

#include <kabsch.hpp>


/*****************************************************************************/
// geometric
/*****************************************************************************/
namespace geometric {
  
  
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
        tmpPoint.z = (((R[6]*(P[i].x-p0.x)) + (R[6]*(P[i].y-p0.y)) + (R[8]*(P[i].z-p0.z))) * S) + T[2] + p0.z;
        
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
  //  intersection | between two lines
  /*****************************************************************************/
  inline bool intersection(const cv::Vec3f & lineA, const cv::Vec3f & lineB, cv::Point2f & point){
    
    double det = (lineA[0]*lineB[1]) - (lineA[1]*lineB[0]);
    
    if(fabs(det) <= FLT_EPSILON) return false;
    
    double invDet = 1.0 / det;
    
    point.x = invDet * (lineA[1]*lineB[2]-lineA[2]*lineB[1]);
    point.y = invDet * (lineA[2]*lineB[0]-lineA[0]*lineB[2]);
    
    
    double a = lineA[0]/lineA[1];
    double b = lineB[0]/lineB[1];
    double c = lineA[2]/lineA[1];
    double d = lineB[2]/lineB[1];
    
    cv::Point2f pt;
    
    pt.x = (d-c) / (a-b);
    pt.y = ((a*d)-(b*c)) / (a-b);

    //printf("BB %.8f %.8f - %.8f %.8f - %e %e\n", point.x, point.y, pt.x, pt.y, det, invDet);
    
    return true;

  }
  
  
  /*****************************************************************************/
  //  dist | between a point and a lines powed
  /*****************************************************************************/
  template <typename TP, typename TL>
  inline float dist(const TP & point, const TL & line){
    
    //linea ortogonale a "line" passante per point
    cv::Vec3f lineP = cv::Vec3f(line[1], -line[0], (line[0]*point.y)-(line[1]*point.x));
    
    TP cross;
    
    if(!intersection(line, lineP, cross)) return INFINITY;
    
    return cv::norm(point - cross);
        
  }
  
  /*****************************************************************************/
  // findBestRTS
  /*****************************************************************************/
  template <int dim, typename type, typename type_p0>
  void findBestRTS(const type & pointsA, const type & pointsB, type_p0 & p0, cv::Mat & R, cv::Mat & T, double & S, double distanceNNFactor, uint32_t maxIter = 100){
    
    // mi calcolo la distanza NN tra i punti in A e quelli in B
    double NNdist = utils::NNDistance(pointsA, pointsB) * 1.5;
    
    // mi trovo i vicini in base alla distanza
    std::vector<std::vector<uint32_t> > match = utils::neighbor::byDistance(pointsA, pointsB, NNdist, 1);
    
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
      if constexpr (dim==3) p0 = kabsch::solve3D(P, Q, R, T, &S);
      if constexpr (dim==2) p0 = kabsch::solve2D(P, Q, R, T, &S);
      
      // applico e riruoto
      type movedPointsA = pointsA;
      
      if constexpr (dim==3) geometric::applyRTS3D(movedPointsA, p0, R, T, S);
      if constexpr (dim==2) geometric::applyRTS2D(movedPointsA, p0, R, T, S);
      
      double newNNdist = utils::NNDistance(movedPointsA, pointsB) * 1.5;
      
      // mi trovo i vicini in base alla distanza
      match = utils::neighbor::byDistance(movedPointsA, pointsB, NNdist, 1);
      
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
  
  
} /* namespace geometric */

#endif /* _H_COBBS_GEOMETRIC_H_ */





