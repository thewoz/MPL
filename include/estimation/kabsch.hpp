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


#ifndef _MPL_KABSCH_H_
#define _MPL_KABSCH_H_

#include <cstdlib>
#include <cstdio>

#include <cmath>

#include <opencv2/opencv.hpp>

//*****************************************************************************/
//  mpl::geometry::kabsch
//*****************************************************************************/
namespace mpl::geometry::kabsch {
  
  struct info_t {
    
    cv::Mat Q;
    cv::Mat R;
    
    cv::Mat T;
    
    double S;
    
    cv::Mat p0;
    
  };
  
  struct info2D_t {
    
    cv::Mat Q;
    cv::Mat R;
    
    cv::Mat T;
    
    double S;
    
    cv::Point2d p0;
    
  };
  
  struct info3D_t {
    
    cv::Mat Q;
    cv::Mat R;
    
    cv::Mat T;
    
    double S;
    
    cv::Point3d p0;
    
  };
  
  
  //*****************************************************************************/
  // namespace util_kabsch
  //*****************************************************************************/
  namespace util_kabsch {
  
    cv::Mat _solve(uint32_t size, uint32_t dims, const double * P, const double * Q, cv::Mat & R, cv::Mat & T, double * S = NULL, double * barycenterP = NULL, double * barycenterQ = NULL, double EPS = DBL_EPSILON) {
            
      double * p0 = (double *) calloc(dims, sizeof(double));
      double * q0 = (double *) calloc(dims, sizeof(double));
      
      if(barycenterP != NULL && barycenterP[0] != DBL_MAX) {
        
        for(uint32_t i=0; i<dims; ++i)
          p0[i] = barycenterP[i];

      } else {
        
        for(uint32_t i=0; i<dims; ++i){
          const double * row = &P[i*size];
          for(uint32_t k=0; k<size; ++k){
            p0[i] += row[k];
          }
        }
        
        for(uint32_t i=0; i<dims; ++i){
          p0[i] /= size;
        }
        
      }
      
      if(barycenterQ != NULL && barycenterQ[0] != DBL_MAX) {
        
        for(uint32_t i=0; i<dims; ++i)
          q0[i] = barycenterQ[i];

      } else {
        
        for(uint32_t i=0; i<dims; ++i){
          const double * row = &Q[i*size];
          for(uint32_t k=0; k<size; ++k){
            q0[i] += row[k];
          }
        }
        
        for(uint32_t i=0; i<dims; ++i){
          q0[i] /= size;
        }
        
      }
        
      cv::Mat A = cv::Mat::zeros(dims, dims, CV_64F);
      
      for(uint32_t dp=0; dp<dims; ++dp){
        
        const double * rowP = &P[dp*size];
        
        for(uint32_t dq=0; dq<dims; ++dq){
          
          const double * rowQ = &Q[dq*size];
          
          for(uint32_t k=0; k<size; ++k){
            
            A.at<double>(dp,dq) += (rowP[k]-p0[dp]) * (rowQ[k]-q0[dq]);
            
          }
          
        }
        
      }
      
      cv::Mat W,U,V;
      
      cv::SVDecomp(A, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
      
      R = V.t() * U.t();
      
      if(cv::determinant(R) < 0){
        V.col(dims-1) *= -1.0;
        R = V.t() * U.t();
      }
      
      T = cv::Mat(dims, 1, CV_64F);
      
      for(uint32_t i=0; i<dims; ++i){
        T.at<double>(i) = q0[i] - p0[i];
      }
      
      /* compute the optimal scaling */
      if(S != NULL){
        
        if(size > 1){
          
          double nom = 0.0;
          double dom = 0.0;
          
          double * RP = (double *) calloc(size, sizeof(double));
          
          for(uint32_t ir=0; ir<dims; ++ir) {
            
            memset(RP, 0, size*sizeof(double));
            
            double * rowR = R.ptr<double>(ir);
            
            for(uint32_t ip=0; ip<dims; ++ip){
              
              const double * rowP = &P[ip*size];
              
              for(uint32_t j=0; j<size; ++j){
                RP[j] += rowR[ip] * (rowP[j]-p0[ip]);
                
              }
              
            }
            
            const double * rowQ = &Q[ir*size];
            
            for(uint32_t j=0; j<size; ++j){
              nom += (rowQ[j]-q0[ir]) * RP[j];
              dom += RP[j] * RP[j];
            }
            
          }
          
          *S = nom / dom;
          
          free(RP);
          
        } else { *S = 1.0; }
        
      }

      // copio p0
      cv::Mat ret;
      ret.create(dims, 1, CV_64F);
      
      for(uint32_t i=0; i<dims; ++i){
        ((double*)ret.data)[i] = p0[i];
      }
      
      free(p0);
      free(q0);
      
      return ret;
      
    }
    
  }

  //*****************************************************************************/
  // solve
  //*****************************************************************************/
  cv::Mat solve(const cv::Mat & P, const cv::Mat & Q, cv::Mat & R, cv::Mat & T, double * S = NULL, double * barycenterP = NULL, double * barycenterQ = NULL, double EPS = DBL_EPSILON) {
    
    assert(P.type()  == CV_64FC1);
    assert(Q.type()  == CV_64FC1);

    assert(P.size()  == Q.size());
    assert(P.type()  == Q.type());
    assert(P.depth() == Q.depth());
        
    return util_kabsch::_solve(P.cols, P.rows, (double*)P.data, (double*)Q.data, R, T, S, barycenterP, barycenterQ, EPS);
    
  }

  //*****************************************************************************/
  // solve3D
  //*****************************************************************************/
  cv::Point3d solve3D(const cv::Mat & P, const cv::Mat & Q, cv::Mat & R, cv::Mat & T, double * S = NULL, cv::Point3d _barycenterP = cv::Point3d(DBL_MAX,DBL_MAX,DBL_MAX),  cv::Point3d _barycenterQ = cv::Point3d(DBL_MAX,DBL_MAX,DBL_MAX), double EPS = DBL_EPSILON) {

    double barycenterP[3] = { _barycenterP.x, _barycenterP.y, _barycenterP.z };
    double barycenterQ[3] = { _barycenterQ.x, _barycenterQ.y, _barycenterQ.z };

    cv::Mat _p0 = solve(P, Q, R, T, S, barycenterP, barycenterQ, EPS);

    return cv::Point3d(((double*)_p0.data)[0],((double*)_p0.data)[1], ((double*)_p0.data)[2]);

  }

  //*****************************************************************************/
  // solve2D
  //*****************************************************************************/
  cv::Point2d solve2D(const cv::Mat & P, const cv::Mat & Q, cv::Mat & R, cv::Mat & T, double * S = NULL, cv::Point2d _barycenterP = cv::Point2d(DBL_MAX,DBL_MAX), cv::Point2d _barycenterQ = cv::Point2d(DBL_MAX,DBL_MAX), double EPS = DBL_EPSILON) {
        
    double barycenterP[2] = { _barycenterP.x, _barycenterP.y };
    double barycenterQ[2] = { _barycenterQ.x, _barycenterQ.y };

    cv::Mat _p0 = solve(P, Q, R, T, S, barycenterP, barycenterQ, EPS);

    return cv::Point2d(((double*)_p0.data)[0],((double*)_p0.data)[1]);
    
  }
  
} /* mpl::geometry::kabsch */

#endif /* _MPL_KABSCH_H_ */
