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

#ifndef _H_MPL_MATH_H_
#define _H_MPL_MATH_H_

#include <cmath>

#include <vector>
#include <algorithm>
#include <random>
#include <limits>

#include <opencv2/opencv.hpp>

#include <mpl/mat.hpp>

/*****************************************************************************/
// namespace mpl::math
/*****************************************************************************/
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
  
  /*****************************************************************************/
  // combinations()
  /*****************************************************************************/
  void combinations(size_t n, size_t k, std::vector<std::vector<size_t>> & combinations, size_t maxCombinationsNum = std::numeric_limits<size_t>::max()) {
    
    static std::random_device rd;
    static std::mt19937 g(rd());
    
    std::vector<bool> v(n, false);
    
    std::fill(v.end() - k, v.end(), true);
    
    size_t counter = 0;
    
    std::vector<size_t> index(n);
    for(size_t i=0; i<n; ++i) index[i] = i;
    
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
  
  size_t polySolve(const std::vector<double> & coeff, std::vector<double> & sol) {
    
    std::vector<cv::Complex<double>> roots;

    cv::solvePoly(coeff, roots);

    for(size_t i=0; i<roots.size(); i++) {

      if(roots[i].im == 0) sol.push_back(roots[i].re);
    }
    
    return sol.size();
    
  }

  size_t solveCubic(const std::vector<double> & coeff, std::vector<double> & sol) {
    
    std::vector<double> roots;

    cv::solveCubic(coeff, roots);

    for(size_t i=0; i<roots.size(); i++) {
      if(roots[i] != 0) sol.push_back(roots[i]);
    }
    
    return sol.size();
    
  }


  #define EIGEN_OPENCV_SVD
  //#define EIGEN_OPENCV_EIGEN
  //#define EIGEN_GSL

  #ifdef EIGEN_GSL
    #include <gsl/gsl_eigen.h>
  #endif

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


  
  
} /* namespace mpl::math */


#endif /* _H_MPL_MATH_H_ */
