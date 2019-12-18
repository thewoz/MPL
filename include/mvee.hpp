/*
* MIT License
*
* Copyright © 2017
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

#ifndef _H_MPL_MVEE_H_
#define _H_MPL_MVEE_H_

#include <cstdlib>
#include <cstdio>

#include <cmath>

#include <opencv2/opencv.hpp>


/*****************************************************************************/
// namespace mvee
/*****************************************************************************/
namespace mvee {

  /*****************************************************************************/
  // Bojan Nikolic <bojan@bnikolic.co.uk>
  // Initial version 2010
  //
  // This file is part of BNMin1 and is licensed under GNU General
  // Public License version 2
  //
  // file ellipsoids.hxx
  //
  // Computation and use of ellipsoids releated to sets of
  // points. References are to Todd and Yildirim, "On Khachiyan's
  // Algorithm for the Computation of Minimum Volume Enclosing
  // Ellipsoids", 2005
  // http://www.bnikolic.co.uk/blog/cpp-khachiyan-min-cov-ellipsoid.html
  /*****************************************************************************/

  
  /*****************************************************************************/
  // namespace util
  /*****************************************************************************/
  namespace util {
        
    double _mveeV1(const cv::Mat & P, cv::Mat & A, cv::Mat & c, double eps, size_t maxiter) {
      
      int d = P.rows;
      int N = P.cols;
      
      cv::Mat Q = cv::Mat(d+1, N, CV_64F);
      
      P.copyTo(Q(cv::Rect(0, 0, N, d)));
      
      Q.row(Q.rows-1).setTo(1.0);
      
      cv::Mat p(N, 1, CV_64F, cv::Scalar(1.0/P.cols));
      
      double ceps = eps*2;
      
      for(size_t i=0; i<maxiter && ceps>eps; ++i){
        
        cv::Mat O = cv::Mat(d+1, N, CV_64F);
        
        for(int i=0; i<O.rows; ++i)
        for(int j=0; j<O.cols; ++j)
		      O.at<double>(i,j) = Q.at<double>(i,j) * p.at<double>(j);
        
        cv::Mat X = O * Q.t();
        
        //std::cout << "X " << X << std::endl;
        
        //return 0.0;
        
        cv::Mat G;
        
        cv::solve(X, Q, G, cv::DECOMP_LU);
        
        cv::Mat M;
        
        cv::reduce(G.mul(Q), M, 0, cv::REDUCE_SUM);
        
        // cv::Mat X = Q * cv::Mat::diag(p) * Q();
        
        // cv::Mat M = Q.t() * X.inv() * Q;
        
        double maximum = - DBL_MAX;
        
        int index = 0;
        
        for(int i=0; i<M.cols; ++i){
          if(M.at<double>(0,i) > maximum) { maximum = M.at<double>(0,i); index = i; }
        }
        
        //printf("%e\n", maximum);
        
        //minMaxLoc(M, NULL, &maximum, NULL, &j);
        
        const double step_size = (maximum-d-1)/((d+1)*(maximum-1));
        
        //if(fabs(1-step_size) <= FLT_EPSILON) break;
        
        cv::Mat newp = p * (1-step_size);
        
        newp.at<double>(index) += step_size;
        
        ceps = cv::norm(newp, p);
        
        p = newp;
        
      }
      
      //cv::Mat PN = (P * p) * P.t();
      
      cv::Mat O = cv::Mat(d, N, CV_64F);
      
      for(int i=0; i<O.rows; ++i)
      for(int j=0; j<O.cols; ++j)
		    O.at<double>(i,j) = P.at<double>(i,j) * p.at<double>(j);
      
      cv::Mat PN = O * P.t();
      
      cv::Mat M2 = P * p;
      
      cv::Mat M3 = M2 * M2.t();
         
      A = (1.0/d) * (PN-M3).inv();
      
      c = P * p;
      
      return ceps;
      
    }
    
    
    double _mveeV2(const cv::Mat & P, cv::Mat & A, cv::Mat & c, double eps, size_t maxiter) {
      
      uint32_t dims = P.rows;
      uint32_t size = P.cols;
      
      cv::Mat Q = cv::Mat(dims+1,   size, CV_64F);
      cv::Mat O = cv::Mat(dims+1,   size, CV_64F);
      cv::Mat X = cv::Mat(dims+1, dims+1, CV_64F);
      cv::Mat E = cv::Mat(size,        1, CV_64F, cv::Scalar(1.0/P.cols));

      P.copyTo(Q(cv::Rect(0, 0, size, dims)));
      
      Q.row(Q.rows-1).setTo(1.0);
      
      double ceps = eps*2;
      
      double * ptrQ = (double*) Q.data;
      double * ptrO = (double*) O.data;
      double * ptrX = (double*) X.data;
      double * ptrP = (double*) P.data;
      double * ptrE = (double*) E.data;

      double * M = (double*) calloc(size, sizeof(double));

  
      //std::cout << "O " << O << std::endl;
      //std::cout << "Q " << Q << std::endl;
      //std::cout << "E " << E << std::endl;

      for(size_t i=0; i<maxiter && ceps>eps; ++i){
        
        /* O = Q * E */
        for(uint32_t d=0; d<dims+1; ++d){
          
          double * rowO = &ptrO[d*size];
          double * rowQ = &ptrQ[d*size];
          
          for(uint32_t k=0; k<size; ++k)
            rowO[k] = rowQ[k] * ptrE[k];
          
        }
        
        memset(ptrX, 0, (dims+1)*(dims+1)*sizeof(double));

        //std::cout << "X " << X << std::endl;

        /* X = O * Q^T */
        for(uint32_t xi=0; xi<dims+1; ++xi){
          
          double * rowX = &ptrX[xi*(dims+1)];
          double * rowO = &ptrO[xi*size];

          for(uint32_t xj=0; xj<dims+1; ++xj){

            double * rowQ = &ptrQ[xj*size];
            
            for(uint32_t k=0; k<size; ++k){
              
              rowX[xj] += rowO[k] * rowQ[k];
              
            }
            
          }
          
        }
        
        //std::cout << "X " << X << std::endl;

        //return 0.0;
        
        cv::Mat G;
        
        cv::solve(X, Q, G, cv::DECOMP_LU);
        
        //cv::Mat M;
        
        //cv::reduce(G.mul(Q), M, 0, CV_REDUCE_SUM);
        
        memset(M, 0, size*sizeof(double));
        
        for(uint32_t d=0; d<dims+1; ++d){
          
          double * rowQ = &((double*) Q.data)[d*size];
          double * rowG = &((double*) G.data)[d*size];
          
          for(uint32_t k=0; k<size; ++k){

            M[k] += rowQ[k] * rowG[k];
            
          }
          
        }
        
        double maximum = - DBL_MAX;
        
        int index = 0;
        
        //double * ptrM = (double*) M.data;
        
        for(int k=0; k<size; ++k){
          if(M[k] > maximum) { maximum = M[k]; index = k; }
        }
        
        const double step_size = (maximum-dims-1)/((dims+1)*(maximum-1));
        
        double delta = 1 - step_size;
        
        //cv::Mat F = E * (1-step_size);

        double norm = 0.0;
        
        for(int k=0; k<size; ++k){
          
          double tmp = ptrE[k] * delta;
          
          if(k==index) tmp += step_size;
          
          norm += (ptrE[k] - tmp) * (ptrE[k] - tmp);
          
          ptrE[k] = tmp;
          
        }
        
        //F.at<double>(index) += step_size;
        
        //ptrF[index] += step_size;
        
        ceps = sqrt(norm); //norm(F-E);
        
        //memcpy(ptrE, ptrF, size * sizeof(double));
        
        //E = F;
        
        //ptrE = (double*) E.data;
        
      }
      
      /* PN = (P * E) * P^T */
      
      /* O = P * E */
      
      cv::Mat OO = cv::Mat(dims, size, CV_64F);
      
      double * ptrOO = (double*) OO.data;
      
      for(int i=0; i<dims; ++i){
        
              double * rowOO = &ptrOO[i*size];
        const double * rowP  = &ptrP[i*size];
        
        for(int j=0; j<size; ++j)
          rowOO[j] = rowP[j] * ptrE[j];
        
      }
      
      //cv::Mat PN = ;
      
      cv::Mat M2 = P * E;
      
      c = M2;

      cv::Mat M3 = M2 * M2.t();
      
      A = (1.0/dims) * (OO * P.t()-M3).inv();
      
      
      return ceps;
      
    }
    
  }
  
  /*****************************************************************************/
  // fit
  /*****************************************************************************/
  void fitV1(const cv::Mat & P, cv::Mat & center, cv::Mat & rotation, cv::Mat & radius, double eps = 1.0e-03, int maxiter = 1000){
    
    cv::Mat A;
    
    util::_mveeV1(P, A, center, eps, maxiter);

    cv::Mat U, W, V;
    
    cv::SVDecomp(A, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    rotation = V;
    
    radius = cv::Mat(P.rows, 1, CV_64F);
    
    for(uint32_t i=0; i<P.rows; ++i)
      radius.at<double>(i) = 1.0/(sqrt(W.at<double>(i)));
    
  }
  
  /*****************************************************************************/
  // fit
  /*****************************************************************************/
  void fitV2(const cv::Mat & P, cv::Mat & center, cv::Mat & rotation, cv::Mat & radius, double eps = 1.0e-03, int maxiter = 1000){
    
    cv::Mat A;
    
    util::_mveeV2(P, A, center, eps, maxiter);
    
    cv::Mat U, W, V;
    
    cv::SVDecomp(A, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    
    rotation = V;
    
    radius = cv::Mat(P.rows, 1, CV_64F);
    
    for(uint32_t i=0; i<P.rows; ++i)
    radius.at<double>(i) = 1.0/(sqrt(W.at<double>(i)));
    
  }

  
}

#endif /* _H_MPL_MVEE_H_ */
