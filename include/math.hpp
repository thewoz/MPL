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
  void svd(cv::Mat & A, cv::Mat & W, cv::Mat & U, cv::Mat & V, int flags = 0) {

    // NOTE:
    //     cv::SVD::MODIFY_A e' ignorato in OpenCV
    //     cv::SVD::FULL_UV nel 99% dei casi non ci serve
    //     cv::SVD::MODIFY_A | cv::SVD::FULL_UV
    cv::SVDecomp(A, W, U, V, flags);
    
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
    
    // Trovo gli autovalori e gli auto vettori
    cv::Mat W,U,V;
    cv::SVDecomp(A, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    for(int i=0; i<size; ++i) {
      eigenvalues(i) = W.at<double>(i);
      V.row(i).copyTo(eigenvectors.row(i));
    }

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
