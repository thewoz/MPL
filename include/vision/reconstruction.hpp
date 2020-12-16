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


#ifndef _H_MPL_VISION_RECONSTRUCTION_H_
#define _H_MPL_VISION_RECONSTRUCTION_H_

#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include <mpl/math.hpp>
#include <mpl/vision/reprojection.hpp>
#include <mpl/opencv.hpp>


/*****************************************************************************/
// namespace vision
/*****************************************************************************/
namespace mpl::vision {
  
  /*****************************************************************************/
  // utils::reco
  /*****************************************************************************/
  namespace utils::reco {
    
    /*****************************************************************************/
    // _recoSVD
    /*****************************************************************************/
    template <typename T2D, typename T4D>
    double _recoSVD(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, cv::Point4_<T4D> & point4D) {
      
      cv::Mat A = cv::Mat(6, 4, CV_64F);
      
      for(int j=0; j<4; ++j) {
        
        A.at<double>(0, j) = pt1.y * prjMat1[8 + j] -         prjMat1[4 + j];
        A.at<double>(1, j) =         prjMat1[j]     - pt1.x * prjMat1[8 + j];
        
        A.at<double>(2, j) = pt2.y * prjMat2[8 + j] -         prjMat2[4 + j];
        A.at<double>(3, j) =         prjMat2[j]     - pt2.x * prjMat2[8 + j];
        
        A.at<double>(4, j) = pt3.y * prjMat3[8 + j] -         prjMat3[4 + j];
        A.at<double>(5, j) =         prjMat3[j]     - pt3.x * prjMat3[8 + j];
        
      }
      
      cv::Mat W,U,V;
     
      cv::Mat AA = A.t() * A;

      mpl::math::svd(AA, W, U, V, cv::SVD::MODIFY_A);
      
      point4D.x = V.at<double>(3,0);
      point4D.y = V.at<double>(3,1);
      point4D.z = V.at<double>(3,2);
      point4D.w = V.at<double>(3,3);
            
      return W.at<double>(3);

    }
    
    /*****************************************************************************/
    // _recoSVD
    /*****************************************************************************/
    template <typename T2D, typename T4D>
    void _recoSVD(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, cv::Point4_<T4D> & point4D) {
      
    cv::Mat A = cv::Mat(4, 4, CV_64F);
      
    for(int j=0; j<4; ++j) {
      
      A.at<double>(0, j) = pt1.y * prjMat1[8 + j] -         prjMat1[4 + j];
      A.at<double>(1, j) =         prjMat1[j]     - pt1.x * prjMat1[8 + j];
      
      A.at<double>(2, j) = pt2.y * prjMat2[8 + j] -         prjMat2[4 + j];
      A.at<double>(3, j) =         prjMat2[j]     - pt2.x * prjMat2[8 + j];
  
    }
          
    cv::Mat W,U,V;

    cv::Mat AA = A.t() * A;    

    mpl::math::svd(AA, W, U, V, cv::SVD::MODIFY_A);
    
    point4D.x = V.at<double>(3,0);
    point4D.y = V.at<double>(3,1);
    point4D.z = V.at<double>(3,2);
    point4D.w = V.at<double>(3,3);
    
  }
    
    /*****************************************************************************/
    // _recoSVD
    /*****************************************************************************/
    template <typename T2D, typename T3D>
    void _recoSVD(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, T3D & point3D) {
    
      cv::Point4d point4D;
      
      _recoSVD(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point4D);
      
      point3D.x = point4D.x / point4D.w;
      point3D.y = point4D.y / point4D.w;
      point3D.z = point4D.z / point4D.w;

    }
    
    /*****************************************************************************/
    // _recoSVD
    /*****************************************************************************/
    template <typename T2D, typename T3D>
    void _recoSVD(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, T3D & point3D) {
      
      cv::Point4d point4D;
           
      _recoSVD(pt1, prjMat1, pt2, prjMat2, point4D);
           
      point3D.x = point4D.x / point4D.w;
      point3D.y = point4D.y / point4D.w;
      point3D.z = point4D.z / point4D.w;
      
    }
  
    /*****************************************************************************/
    // cofactor
    /*****************************************************************************/
    inline double cofactor(const double L[4][4], int i, int j) {
      
      const int hi[3] = { (i + 1) % 4 , (i + 2) % 4 , (i + 3) % 4 };
      const int hj[3] = { (j + 1) % 4 , (j + 2) % 4 , (j + 3) % 4 };
      
      const int sigd = (((i + j) % 2) == 0) ? +1 : -1;
      
      const double modd =
      L[hi[0]][hj[0]] * (L[hi[1]][hj[1]] * L[hi[2]][hj[2]] - L[hi[1]][hj[2]] * L[hi[2]][hj[1]]) +
      L[hi[1]][hj[0]] * (L[hi[2]][hj[1]] * L[hi[0]][hj[2]] - L[hi[2]][hj[2]] * L[hi[0]][hj[1]]) +
      L[hi[2]][hj[0]] * (L[hi[0]][hj[1]] * L[hi[1]][hj[2]] - L[hi[0]][hj[2]] * L[hi[1]][hj[1]]) ;
      
      return sigd * modd;
    }
    
    /*****************************************************************************/
    // _reco
    /*****************************************************************************/
    template <typename T2D, typename T4D>
    void _reco(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, cv::Point4_<T4D> & point4D) {
      
      //* Variabili d'appoggio */
      double r[3][3];
      
      /* Ricopio i Punti */
      r[0][0] = pt1.x; r[0][1] = pt1.y; r[0][2] = (double)1.0;
      r[1][0] = pt2.x; r[1][1] = pt2.y; r[1][2] = (double)1.0;
      r[2][0] = pt3.x; r[2][1] = pt3.y; r[2][2] = (double)1.0;
      
      double L[4][4]; /* Obj -> autovalore minimo di L */
      
      /* Azzero L */
      for(int i=0; i<4; ++i)
        for(int j=i; j<4; ++j)
          L[i][j] = 0.0;
      
      const double * P[3] = { prjMat1, prjMat2, prjMat3 };
      
      for(int c=0; c<3; ++c) {
        
        double A[2][4] ; /* A = [r]_x * P */
        
        for(int i=0; i<2; ++i) {
          for(int j=0; j<4; ++j) {
            A[i][j] = r[c][(i + 1) % 3] * P[c][4 * ((i + 2) % 3) + j] - r[c][(i + 2) % 3] * P[c][4 * ((i + 1) % 3) + j] ;
          }
        }
        
        /* L += A^T * A   */
        for(int i=0; i<4; ++i) {
          for(int j=i; j<4; ++j) {
            for(int k=0; k<2; ++k) {
              L[i][j] += A[k][i] * A[k][j];
            }
          }
        }
        
      } // for C
      
      double eps_shift = 1.0e-06 ;
      
      for(int i=0; i<4; ++i) {
        L[i][i] += eps_shift;
        for(int j=0; j<i; ++j)
          L[i][j] = L[j][i];
      }
      
      /* Inverto la matrice L */
      double DetL =
      L[0][0] * cofactor(L,0,0) +
      L[1][0] * cofactor(L,1,0) +
      L[2][0] * cofactor(L,2,0) +
      L[3][0] * cofactor(L,3,0) ;
      
      double Lm1[4][4]; /* Obj -> autovalore minimo di L */
      
      for(int i=0; i<4; ++i)
        for(int j=i; j<4; ++j)
          Lm1[i][j] = cofactor(L,j,i) / DetL;
      
      for(int i=0; i<4; ++i)
        for(int j=0; j<i; ++j)
          Lm1[i][j] = Lm1[j][i];
      
      /* Parto con un vettore uniforme */
      double BUF0[4] = { 0.5 , 0.5 , 0.5 , 0.5 };
      double BUF1[4];
      
      double * R0 = BUF0;
      double * R1 = BUF1;
      
      double R_diff;
      
      const double eps_tol = 1.0E-12;
      
      do {
        
        double R1_mod = 0.0;
        
        for(int i=0; i<4; ++i) {
          
          R1[i] = 0.0;
          
          for(int j=0; j<4; ++j)
            R1[i] += Lm1[i][j] * R0[j];
          
          R1_mod += R1[i] * R1[i];
          
        }
        
        R1_mod = sqrt(R1_mod);
        
        R_diff = 0.0;
        
        for(int i=0; i<4; ++i) {
          R1[i] /= R1_mod;
          R_diff += (R1[i] - R0[i]) * (R1[i] - R0[i]);
        }
        
        double * Rt = R0; R0 = R1; R1 = Rt;
        
      }  while(R_diff > eps_tol);
      
      point4D.x = R0[0];
      point4D.y = R0[1];
      point4D.z = R0[2];
      point4D.w = R0[3];

    }
    
    /*****************************************************************************/
    // _reco
    /*****************************************************************************/
    template <typename T2D, typename T4D>
    void _reco(const cv::Point_<T2D> & pt1, const double * prjMat1, const cv::Point_<T2D> & pt2, const double * prjMat2, cv::Point4_<T4D> & point4D) {
      
      //* Variabili d'appoggio */
      double r[2][3];
      
      /* Ricopio i Punti */
      r[0][0] = pt1.x; r[0][1] = pt1.y; r[0][2] = (double)1.0;
      r[1][0] = pt2.x; r[1][1] = pt2.y; r[1][2] = (double)1.0;
      
      double L[4][4]; /* Obj -> autovalore minimo di L */
      
      /* Azzero L */
      for(int i=0; i<4; ++i)
        for(int j=i; j<4; ++j)
          L[i][j] = 0.0;
      
      const double * P[2] = { prjMat1, prjMat2 };
      
      for(int c=0; c<2; ++c) {
        
        double A[2][4] ; /* A = [r]_x * P */
        
        for(int i=0; i<2; ++i) {
          for(int j=0; j<4; ++j) {
            A[i][j] = r[c][(i + 1) % 3] * P[c][4 * ((i + 2) % 3) + j] - r[c][(i + 2) % 3] * P[c][4 * ((i + 1) % 3) + j] ;
          }
        }
        
        /* L += A^T * A   */
        for(int i=0; i<4; ++i) {
          for(int j=i; j<4; ++j) {
            for(int k=0; k<2; ++k) {
              L[i][j] += A[k][i] * A[k][j];
            }
          }
        }
        
      } // for C
      
      double eps_shift = 1.0e-06 ;
      
      for(int i=0; i<4; ++i) {
        L[i][i] += eps_shift;
        for(int j=0; j<i; ++j)
          L[i][j] = L[j][i];
      }
      
      /* Inverto la matrice L */
      double DetL =
      L[0][0] * cofactor(L,0,0) +
      L[1][0] * cofactor(L,1,0) +
      L[2][0] * cofactor(L,2,0) +
      L[3][0] * cofactor(L,3,0) ;
      
      double Lm1[4][4]; /* Obj -> autovalore minimo di L */
      
      for(int i=0; i<4; ++i)
        for(int j=i; j<4; ++j)
          Lm1[i][j] = cofactor(L,j,i) / DetL;
      
      for(int i=0; i<4; ++i)
        for(int j=0; j<i; ++j)
          Lm1[i][j] = Lm1[j][i];
      
      /* Parto con un vettore uniforme */
      double BUF0[4] = { 0.5 , 0.5 , 0.5 , 0.5 };
      double BUF1[4];
      
      double * R0 = BUF0;
      double * R1 = BUF1;
      
      double R_diff;
      
      const double eps_tol = 1.0E-12;
      
      do {
        
        double R1_mod = 0.0;
        
        for(int i=0; i<4; ++i) {
          
          R1[i] = 0.0;
          
          for(int j=0; j<4; ++j)
            R1[i] += Lm1[i][j] * R0[j];
          
          R1_mod += R1[i] * R1[i];
          
        }
        
        R1_mod = sqrt(R1_mod);
        
        R_diff = 0.0;
        
        for(int i=0; i<4; ++i) {
          R1[i] /= R1_mod;
          R_diff += (R1[i] - R0[i]) * (R1[i] - R0[i]);
        }
        
        double * Rt = R0; R0 = R1; R1 = Rt;
        
      }  while(R_diff > eps_tol);
      
      point4D.x = R0[0];
      point4D.y = R0[1];
      point4D.z = R0[2];
      point4D.w = R0[3];
      
    }

    /*****************************************************************************/
    // _reco
    /*****************************************************************************/
    template <typename T2D, typename T3D>
    void _reco(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, T3D & point3D) {
      
      cv::Point4d point4D;
          
      _reco(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point4D);
          
      point3D.x = point4D.x / point4D.w;
      point3D.y = point4D.y / point4D.w;
      point3D.z = point4D.z / point4D.w;
      
    }
    
    /*****************************************************************************/
    // _reco
    /*****************************************************************************/
    template <typename T2D, typename T3D>
    void _reco(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, T3D & point3D) {
      
      cv::Point4d point4D;
          
      _reco(pt1, prjMat1, pt2, prjMat2, point4D);
          
      point3D.x = point4D.x / point4D.w;
      point3D.y = point4D.y / point4D.w;
      point3D.z = point4D.z / point4D.w;
      
    }
    
  }/* namespace utils::reco */
  
  
 /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T4D>
  double reconstruct(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, T4D & point4D) {
    
    //TODO: check matrix
    
#ifdef USE_SVD
    return utils::reco::_recoSVD(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point4D);
#else
    utils::reco::_reco(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point4D); return 0;
#endif

  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T4D>
  void reconstruct(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, cv::Point4_<T4D> & point4D) {

    //TODO: check matrix
    
#ifdef USE_SVD
    utils::reco::_recoSVD(pt1, prjMat1, pt2, prjMat2, point4D);
#else
    utils::reco::_reco(pt1, prjMat1, pt2, prjMat2, point4D);
#endif

  }
    
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T4D>
  inline double reconstruct(const T2D & pt1, const cv::Mat & prjMat1, const T2D & pt2, const cv::Mat & prjMat2, const T2D & pt3, const cv::Mat & prjMat3, T4D & point4D) {
    
    return reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, pt3, (double*)prjMat3.data, point4D);
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T4D>
  inline void reconstruct(const T2D & pt1, const cv::Mat & prjMat1, const T2D & pt2, const cv::Mat & prjMat2, cv::Point4_<T4D> & point4D) {
    
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, point4D);
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T3D>
  void reconstruct(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, T3D & point3D) {
    
    //TODO: check matrix
    
#ifdef USE_SVD
    utils::reco::_recoSVD(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point3D);
#else
    utils::reco::_reco(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point3D);
#endif

  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T3D>
  void reconstruct(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, T3D & point3D) {

    //TODO: check matrix
    
#ifdef USE_SVD
    utils::reco::_recoSVD(pt1, prjMat1, pt2, prjMat2, point3D);
#else 
    utils::reco::_reco(pt1, prjMat1, pt2, prjMat2, point3D);
#endif

  }
    
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T3D>
  inline void reconstruct(const T2D & pt1, const cv::Mat & prjMat1, const T2D & pt2, const cv::Mat & prjMat2, const T2D & pt3, const cv::Mat & prjMat3, T3D & point3D) {
    
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, pt3, (double*)prjMat3.data, point3D);
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T3D>
  inline void reconstruct(const T2D & pt1, const cv::Mat & prjMat1, const T2D & pt2, const cv::Mat & prjMat2, T3D & point3D) {
    
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, point3D);
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D>
  inline cv::Point3d reconstruct(const T2D & pt1, const cv::Mat & prjMat1, const T2D & pt2, const cv::Mat & prjMat2) {
    
    cv::Point3d point3D;
    
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, point3D);
    
    return point3D;
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T4D>
  void reconstruct(const std::vector<T2D> & pt1, const double * prjMat1, const std::vector<T2D> & pt2, const double * prjMat2, const std::vector<T2D> & pt3, const double * prjMat3, std::vector<cv::Point4_<T4D>> & point4D) {
    
    if(!(pt1.size() == pt2.size() && pt2.size() == pt3.size())){
      fprintf(stderr, "ss\n");
      abort();
    }
    
    point4D.resize(pt1.size());

#ifdef USE_SVD
    cv::Mat A; A.create(6, 4, CV_64F);
#endif
    
    for(std::size_t i=0; i<point4D.size(); ++i){
   
#ifdef USE_SVD
      utils::reco::_recoSVD(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point4D, A);
#else
      utils::reco::_reco(pt1[i], prjMat1, pt2[i], prjMat2, pt3[i], prjMat3, point4D[i]);
#endif

    }
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T4D>
  void reconstruct(const std::vector<T2D> & pt1, const double * prjMat1, const std::vector<T2D> & pt2, const double * prjMat2, std::vector<cv::Point4_<T4D>> & point4D) {
    
    if(!(pt1.size() == pt2.size())){
      fprintf(stderr, "ss\n");
      abort();
    }

    point4D.resize(pt1.size());
    
#ifdef USE_SVD
    cv::Mat A; A.create(6, 4, CV_64F);
#endif
    
    for(std::size_t i=0; i<point4D.size(); ++i){
      
#ifdef USE_SVD
      utils::reco::_recoSVD(pt1[i], prjMat1, pt2[i], prjMat2, point4D[i], A);
#else
      utils::reco::_reco(pt1[i], prjMat1, pt2[i], prjMat2, point4D[i]);
#endif

    }
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T3D>
  void reconstruct(const std::vector<T2D> & pt1, const double * prjMat1, const std::vector<T2D> & pt2, const double * prjMat2, const std::vector<T2D> & pt3, const double * prjMat3, std::vector<T3D> & point3D) {
    
    if(!(pt1.size() == pt2.size() && pt2.size() == pt3.size())){
      fprintf(stderr, "ss\n");
      abort();
    }
    
    point3D.resize(pt1.size());

#ifdef USE_SVD
    cv::Mat A; A.create(6, 4, CV_64F);
#endif
    
    for(std::size_t i=0; i<point3D.size(); ++i){
   
#ifdef USE_SVD   
      utils::reco::_recoSVD(pt1[i], prjMat1, pt2[i], prjMat2, pt3[i], prjMat3, point3D[i], A);
#else
      utils::reco::_reco(pt1[i], prjMat1, pt2[i], prjMat2, pt3[i], prjMat3, point3D[i]);
#endif

    }
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T3D>
  void reconstruct(const std::vector<T2D> & pt1, const double * prjMat1, const std::vector<T2D> & pt2, const double * prjMat2, std::vector<T3D> & point3D) {
    
    if(!(pt1.size() == pt2.size())){
      fprintf(stderr, "ss\n");
      abort();
    }

    point3D.resize(pt1.size());
    
#ifdef USE_SVD
    //cv::Mat A; A.create(6, 4, CV_64F);
#endif
    
    for(std::size_t i=0; i<point3D.size(); ++i){
      
#ifdef USE_SVD
      utils::reco::_recoSVD(pt1[i], prjMat1, pt2[i], prjMat2, point3D[i]);
#else      
      utils::reco::_reco(pt1[i], prjMat1, pt2[i], prjMat2, point3D[i]);
#endif 

    }
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T4D>
  void reconstruct(const std::vector<T2D> & pt1, const cv::Mat & prjMat1, const std::vector<T2D> & pt2, const cv::Mat & prjMat2, const std::vector<T2D> & pt3, const cv::Mat & prjMat3, std::vector<cv::Point4_<T4D>> & point4D) {
    
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, pt3, (double*)prjMat3.data, point4D);
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T4D>
  void reconstruct(const std::vector<T2D> & pt1, const cv::Mat & prjMat1, const std::vector<T2D> & pt2, const cv::Mat & prjMat2, std::vector<cv::Point4_<T4D>> & point4D) {
    
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, point4D);
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T3D>
  void reconstruct(const std::vector<T2D> & pt1, const cv::Mat & prjMat1, const std::vector<T2D> & pt2, const cv::Mat & prjMat2, const std::vector<T2D> & pt3, const cv::Mat & prjMat3, std::vector<T3D> & point3D) {
    
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, pt3, (double*)prjMat3.data, point3D);
    
  }
  
  /*****************************************************************************/
  // reconstruct
  /*****************************************************************************/
  template <typename T2D, typename T3D>
  void reconstruct(const std::vector<T2D> & pt1, const cv::Mat & prjMat1, const std::vector<T2D> & pt2, const cv::Mat & prjMat2, std::vector<T3D> & point3D) {
    
    reconstruct(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, point3D);
    
  }
  
  
  
  /*****************************************************************************/
  // reconstruction
  /*****************************************************************************/
  namespace reconstruction {
    
    /*****************************************************************************/
    // error
    /*****************************************************************************/
    template <typename T2D>
    double error(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, const T2D & pt3, const double * prjMat3, double maxError = std::numeric_limits<double>::max()) {
      
      //TODO: check matrix
      
      cv::Point3d point3D;
      
#ifdef USE_SVD
      utils::reco::_recoSVD(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point3D);
#else
      utils::reco::_reco(pt1, prjMat1, pt2, prjMat2, pt3, prjMat3, point3D);
#endif
       
      cv::Point2d _pt1; reproject(point3D, _pt1, prjMat1); double error1 = cv::norm(pt1, _pt1);
      
      if(error1 > maxError) return std::numeric_limits<double>::max();
      
      cv::Point2d _pt2; reproject(point3D, _pt2, prjMat2); double error2 = cv::norm(pt2, _pt2);
      
      if(error2 > maxError) return std::numeric_limits<double>::max();
      
      cv::Point2d _pt3; reproject(point3D, _pt3, prjMat3); double error3 = cv::norm(pt3, _pt3);
      
      if(error3 > maxError) return std::numeric_limits<double>::max();
      
      return (error1 + error2 + error3) / 3.0;
      
    }
  
    /*****************************************************************************/
    // error
    /*****************************************************************************/
    template <typename T2D>
    double error(const T2D & pt1, const double * prjMat1, const T2D & pt2, const double * prjMat2, double maxError = std::numeric_limits<double>::max()) {
      
      //TODO: check matrix
      
      cv::Point3d point3D;
      
#ifdef USE_SVD
      utils::reco::_recoSVD(pt1, prjMat1, pt2, prjMat2, point3D);
#else
      utils::reco::_reco(pt1, prjMat1, pt2, prjMat2, point3D);
#endif
       
      cv::Point2d _pt1; reproject(point3D, _pt1, prjMat1); double error1 = cv::norm(pt1, _pt1);
      
      if(error1 > maxError) return std::numeric_limits<double>::max();
      
      cv::Point2d _pt2; reproject(point3D, _pt2, prjMat2); double error2 = cv::norm(pt2, _pt2);
      
      if(error2 > maxError) return std::numeric_limits<double>::max();
      
      return (error1 + error2) / 2.0;
      
    }
  
    /*****************************************************************************/
    // error
    /*****************************************************************************/
    template <typename T2D>
    inline double error(const T2D & pt1, const cv::Mat prjMat1, const T2D & pt2, const cv::Mat prjMat2, const T2D & pt3, const cv::Mat prjMat3, double maxError = std::numeric_limits<double>::max()) {
      
      return error(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, pt3, (double*)prjMat3.data, maxError);
      
    }
    
    /*****************************************************************************/
    // error
    /*****************************************************************************/
    template <typename T2D>
    inline double error(const T2D & pt1, const cv::Mat prjMat1, const T2D & pt2, const cv::Mat prjMat2, double maxError = std::numeric_limits<double>::max()) {
      
      return error(pt1, (double*)prjMat1.data, pt2, (double*)prjMat2.data, maxError);
      
    }
    
    /*****************************************************************************/
    // error
    /*****************************************************************************/
    template <typename T2D>
    double error(const std::vector<T2D> & pt1, const double * prjMat1, const std::vector<T2D> & pt2, const double * prjMat2, const std::vector<T2D> & pt3, const double * prjMat3, double maxError = std::numeric_limits<double>::max()) {
    
      if(!(pt1.size() == pt2.size() && pt1.size() == pt3.size())){
        fprintf(stderr, "error vector must be same size\n");
        abort();
      }
      
      double errorTot = 0.0;
      
      for(size_t i=0; i<pt1.size(); ++i) {
        errorTot += error(pt1[i], prjMat1, pt2[i], prjMat2, pt3[i], prjMat3, maxError);
      }
      
      return (errorTot / (double)pt1.size());
      
    }
    
    /*****************************************************************************/
    // error
    /*****************************************************************************/
    template <typename T2D>
    inline double error(const std::vector<T2D> & pt1, const cv::Mat prjMat1, const std::vector<T2D> & pt2, const cv::Mat prjMat2, const std::vector<T2D> & pt3, const cv::Mat prjMat3, double maxError = std::numeric_limits<double>::max()) {
      
      if(!(pt1.size() == pt2.size() && pt1.size() == pt3.size())){
        fprintf(stderr, "error vector must be same size\n");
        abort();
      }
      
      double errorTot = 0.0;
      
      for(size_t i=0; i<pt1.size(); ++i) {
        errorTot += error(pt1[i], (double*)prjMat1.data, pt2[i], (double*)prjMat2.data, pt3[i], (double*)prjMat3.data, maxError);
      }
      
      return (errorTot / (double)pt1.size());
      
    }
    
  } /* namespace reconstruction */
  
  
} /* namespace vision */



#endif /* _H_MPL_VISION_RECONSTRUCTION_H_ */


