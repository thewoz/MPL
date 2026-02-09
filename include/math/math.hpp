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

#ifndef _H_MPL_MATH_H_
#define _H_MPL_MATH_H_

#include <cmath>

#include <vector>
#include <algorithm>
#include <random>
#include <limits>

#include <opencv2/opencv.hpp>

#include <mpl/mat.hpp>


#define SVD_OPENCV
//#define SVD_GSL
#ifdef SVD_GSL
  #include <gsl/gsl_linalg.h>
#endif


#define EIGEN_OPENCV_SVD
//#define EIGEN_OPENCV_EIGEN
//#define EIGEN_GSL
#ifdef EIGEN_GSL
  #include <gsl/gsl_eigen.h>
#endif

//****************************************************************************/
// namespace mpl::math
//****************************************************************************
namespace mpl::math {
  
  template <class T>
  inline double norm(const cv::Point_<T> & a) {
    
    return std::sqrt((a.x*a.x) + (a.y*a.y));
    
  }
  
  template <class T>
  inline double norm(const cv::Point3_<T> & a) {
    
    return std::sqrt((a.x*a.x) + (a.y*a.y) + (a.z*a.z));
    
  }
  
  template <class T>
  inline double norm(const cv::Point_<T> & a, const cv::Point_<T> & b) {
    
    return std::sqrt(((a.x-b.x) * (a.x-b.x)) + ((a.y-b.y) * (a.y-b.y)));
    
  }
  
  template <class T>
  inline double norm(const cv::Point3_<T> & a, const cv::Point3_<T> & b) {
    
    return std::sqrt(((a.x-b.x) * (a.x-b.x)) + ((a.y-b.y) * (a.y-b.y)) + ((a.z-b.z) * (a.z-b.z)));
    
  }
  
  template <class T>
  inline double norm(const std::vector<T> & A, const std::vector<T> & B){
    
    if(A.size() != B.size()) {
      fprintf(stderr, "error in mpl::norm() vector must be of the same size\n");
      abort();
    }
    
    double result = 0;
    
    for(size_t i=0; i<A.size(); ++i)
      result += cv::norm(A[i], B[i]);
    
    result /= (double) A.size();
    
    return result;
    
  }
  
  
  //****************************************************************************
  // combinations()
  //****************************************************************************
  void combinations(size_t n, size_t k, std::vector<std::vector<size_t>> & combinations, size_t maxCombinationsNum = std::numeric_limits<size_t>::max(), bool random = false) {
    
    static std::random_device rd;
    static std::mt19937 g(rd());
    
    std::vector<bool> v(n, false);
    
    std::fill(v.end() - k, v.end(), true);
    
    size_t counter = 0;
    
    std::vector<size_t> index(n);
    for(size_t i=0; i<n; ++i) index[i] = i;
    
    if(random) {
    
      combinations.resize(maxCombinationsNum, std::vector<size_t>(k));

      for(int i=0; i<maxCombinationsNum; ++i) {
        
        std::shuffle(index.begin(), index.end(), g);

        for(size_t j=0; j<k; ++j) {
          combinations[i][j] = index[j];
        }
 
      }

    } else {
      
      std::shuffle(index.begin(), index.end(), g);
      
      do {
        
        combinations.resize(combinations.size()+1, std::vector<size_t>(k));
        
        std::vector<size_t> & combination = combinations.back();
        
        for(size_t i=0, j=0; i<n; ++i) {
          
          if(v[i]) combination[j++] = index[i];
          
        }
        
        //for(size_t i=0; i <combination.size(); ++i) std::cout << combination[i] << " "; std::cout << std::endl;
        
        counter++;
        
      } while (std::next_permutation(v.begin(), v.end()) && counter < maxCombinationsNum);
      
    }
    
  }
  
  
  //****************************************************************************
  // polySolve()
  //****************************************************************************
  size_t polySolve(const std::vector<double> & coeff, std::vector<double> & sol) {
    
    std::vector<cv::Complex<double>> roots;

    cv::solvePoly(coeff, roots);

    for(size_t i=0; i<roots.size(); i++) {

      if(roots[i].im == 0) sol.push_back(roots[i].re);
      
    }
    
    return sol.size();
    
  }

  //****************************************************************************
  // polySolve()
  //****************************************************************************
  size_t polySolveAll(const std::vector<double> & coeff, std::vector<double> & sol) {
    
    std::vector<cv::Complex<double>> roots;
    
    cv::solvePoly(coeff, roots);
    
    for(size_t i=0; i<roots.size(); i++) {
      
      sol.push_back(roots[i].re);
      
    }
    
    return sol.size();
    
  }
  
  //****************************************************************************
  // solveCubic()
  //****************************************************************************
  size_t solveCubic(const std::vector<double> & coeff, std::vector<double> & sol) {
    
    std::vector<double> roots;

    cv::solveCubic(coeff, roots);

    for(size_t i=0; i<roots.size(); i++) {
      if(roots[i] != 0) sol.push_back(roots[i]);
    }
    
    return sol.size();
    
  }

  
  //****************************************************************************/
  // svd()
  //****************************************************************************/
  // FIXME: Passo A per copia e non per referenza
  void svd(cv::Mat & A, cv::Mat & W, cv::Mat & U, cv::Mat & V, int flags = 0) {
    
#ifdef SVD_OPENCV
    
    // NOTE:
    //     cv::SVD::MODIFY_A e' ignorato in OpenCV
    //     cv::SVD::FULL_UV nel 99% dei casi non ci serve
    //     cv::SVD::MODIFY_A | cv::SVD::FULL_UV
    cv::SVDecomp(A, W, U, V, flags);

#endif
    
#ifdef SVD_GSL
    
    // NOTE:
    //     cv::SVD::MODIFY_A e' ignorata per colpa mia
    //     cv::SVD::FULL_UV e' ignorata per colpa mia
    //     cv::SVD::MODIFY_A | cv::SVD::FULL_UV
    
    gsl_matrix * a = gsl_matrix_alloc(A.rows, A.cols);
    gsl_matrix * v = gsl_matrix_alloc(A.cols, A.cols);
        
    memcpy(a->data, A.ptr<double>(0), sizeof(double) * A.rows * A.cols);
    
    gsl_vector * s = gsl_vector_alloc(A.cols);
    gsl_vector * t = gsl_vector_alloc(A.cols);
    
    gsl_linalg_SV_decomp(a, v, s, t);
    
    U = cv::Mat((int)a->size1, (int)a->size2, CV_64FC1);
    memcpy(U.ptr<double>(0), a->data, sizeof(double) * a->size1 * a->size2);
    
    V = cv::Mat((int)v->size1, (int)v->size2, CV_64FC1);
    for(int i = 0; i < v->size1; i++)
      for(int j = 0; j < v->size2; j++)
        V.at<double>(j,i) = gsl_matrix_get(v, i, j);
    
    W = cv::Mat((int)s->size, 1, CV_64FC1);
    for(int i = 0; i < s->size; i++)
      W.at<double>(i) = gsl_vector_get(s, i);
    
    gsl_vector_free(s);
    gsl_vector_free(t);
    
    gsl_matrix_free(a);
    gsl_matrix_free(v);
    
#endif
    
  }
  
  
  //****************************************************************************
  // eigen()
  //****************************************************************************
  // FIXME: Passo A per copia e non per referenza
  void eigen(mpl::Mat A, mpl::Vec & eigenvalues, mpl::Mat & eigenvectors) {

    if(A.rows != A.cols) {
      fprintf(stderr, "error mpl::eigen() - matrix must be symmetric\n");
      abort();
    }

    int size = A.rows;

    eigenvalues.resize(size);

    eigenvectors.resize(size, size);

  #ifdef EIGEN_OPENCV_SVD

    // Trovo gli autovalori e gli auto vettori
    cv::Mat W,U,V;
    cv::SVDecomp(A, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    for(int i=0; i<size; ++i) {
      eigenvalues(i) = W.at<double>(i);
      V.row(i).copyTo(eigenvectors.row(i));
    }
    

  #endif

  #ifdef EIGEN_OPENCV_EIGEN

    // Trovo gli autovalori e gli auto vettori
    cv::eigen(A, eigenvalues, eigenvectors);

  #endif

  #if defined(EIGEN_OPENCV_SVD) || defined(EIGEN_OPENCV_EIGEN)

    // Alloco lo spazio per le soluzioni
    std::vector<std::pair<double,cv::Mat>> solution(eigenvalues.rows);
    
    // Sorto in base agli autovalori trovati
    for(int i=0; i<size; ++i) {
      solution[i] = std::pair<double,cv::Mat>(fabs(eigenvalues(i)), eigenvectors.row(i).clone());
    }
      
    // Ordino gli autovalori
    std::sort(solution.begin(), solution.end(), [](const auto & a, const auto & b) { return a.first < b.first; });

    // Ricopio ordinando
    for(int i=0; i<size; ++i) {
      eigenvalues(i)      = solution[i].first;
      solution[i].second.copyTo(eigenvectors.row(i));
    }
    
  #endif


  #ifdef EIGEN_GSL

    gsl_matrix_view m = gsl_matrix_view_array(A.ptr<double>(0), size, size);

    gsl_vector * eval = gsl_vector_alloc(size);
    gsl_matrix * evec = gsl_matrix_alloc(size, size);

    gsl_eigen_symmv_workspace * w = gsl_eigen_symmv_alloc(size);

    gsl_eigen_symmv(&m.matrix, eval, evec, w);

    gsl_eigen_symmv_free(w);

    gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    //FIXME: sta merda!
    for(int i=0; i<size; ++i) {

      eigenvalues(i) = gsl_vector_get(eval, i);

      gsl_vector_const_view evec_i = gsl_matrix_const_column(evec, i);

      const gsl_vector * evec_ii = &evec_i.vector;
       
      //gsl_vector_fprintf(stdout, evec_ii, "%f");

      for(int j=0; j<size; ++j)
        eigenvectors(i,j) = gsl_vector_get(evec_ii, j);
    }

    gsl_vector_free(eval);

    gsl_matrix_free(evec);

  #endif

  }

  //****************************************************************************
  // normal_pdf()
  //****************************************************************************
  double normal_pdf(double x, double mean, double sigma) {
    
    return (1.0/(sigma*std::sqrt(2.0*M_PI))) * std::exp(-0.5 * std::pow((x-mean) / sigma, 2));
    
  }

  //****************************************************************************
  // multivariate_normal_pdf()
  //****************************************************************************
  double multivariate_normal_pdf(const cv::Point2d & point, const cv::Point2d & mean, const cv::Mat & covariance) {
    double det = cv::determinant(covariance);
    double norm = 1.0 / (2.0 * M_PI * std::sqrt(det));
    cv::Mat centered = cv::Mat_<float>(point - mean);
    double exponent = cv::Mat(-0.5 * centered.t() * covariance.inv() * centered).at<float>(0);
    return norm * std::exp(exponent);
  }

} /* namespace mpl::math */


#endif /* _H_MPL_MATH_H_ */
