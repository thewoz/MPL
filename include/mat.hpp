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

#ifndef _H_MPL_MAT_H_
#define _H_MPL_MAT_H_

#include <opencv2/opencv.hpp>

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
/*****************************************************************************/
// Classe per una matrice generica
/*****************************************************************************/
class Mat : public cv::Mat {

  public:
  
    Mat() : cv::Mat() { }  

    Mat(const cv::MatExpr & mat) : cv::Mat(mat) { }
  
    Mat(const cv::Mat & mat) : cv::Mat(mat) { }

    Mat(uint32_t row, uint32_t col) : cv::Mat(row, col, CV_64FC1, cv::Scalar(0)) { }
  
    void resize(uint32_t row, uint32_t col) {
      if(empty()) create(cv::Size(col, row), CV_64FC1);
      else cv::resize(*this, *this, cv::Size(col, row));
    }

    //cv::Mat row(int i) { return this->at<cv::Mat>(i); }

    double   operator () (int i, int j) const { return this->at<double>(i,j); }
    double & operator () (int i, int j)       { return this->at<double>(i,j); }

};

/*****************************************************************************/
// Classe per una vettore generica
/*****************************************************************************/
class Vec : public cv::Mat {
  
  public:
  
    Vec() { }
  
    Vec(Vec & vec) : cv::Mat(vec) { }

    Vec(uint32_t size) : cv::Mat(size, 1, CV_64FC1, cv::Scalar(0)) { }
    
    /*****************************************************************************/
    // resize() - 
    /*****************************************************************************/
    void resize(uint32_t size) {
      if(empty()) create(cv::Size(1, size), CV_64FC1);
      else cv::resize(*this, *this, cv::Size(1, size));
    }

    /*****************************************************************************/
    // operator ()
    /*****************************************************************************/
    // NOTE: forse non serve lo 0 inziale
    double   operator () (int i) const { return this->at<double>(i); }
    double & operator () (int i)       { return this->at<double>(i); }
  
    /*****************************************************************************/
    // operator =
    /*****************************************************************************/
    Vec & operator = (const cv::Mat & mat) {
    
      if(mat.channels() != 1) {
        fprintf(stderr, "Matrix must have only one chanel\n");
        abort();
      }
    
      // if(mat.depth() != CV_64F) {
      //   fprintf(stderr, "Matrix must be a 64bit float one\n");
      //   abort();
      // }

      if(mat.rows != 1 && mat.cols != 1) {
        fprintf(stderr, "Matrix must be a 1xN or a Nx1 matrix\n");
        abort();
      } 
    
      if(mat.rows == 1) {

        if(empty()) create(cv::Size(1, mat.cols), CV_64FC1);
        else cv::resize(*this, *this, cv::Size(1, mat.cols));
      
        cv::Mat tmp = mat.t();
      
        tmp.col(0).copyTo(this->col(0));
     
      }
    
      if(mat.cols == 1) {

        if(empty()) create(cv::Size(1, mat.rows), CV_64FC1);
        else cv::resize(*this, *this, cv::Size(1, mat.rows));

        cv::Mat tmp = mat.t();
      
        mat.col(0).copyTo(this->col(0));

      }
    
      return *this;

    }
  
};

/*****************************************************************************/
// Classe per una matrice 3x3 double (64bit)
/*****************************************************************************/
class Mat3 : public cv::Mat {
  
  public:
  
    Mat3() : cv::Mat(3, 3, CV_64FC1, cv::Scalar(0)) { }
  
    double   operator () (int i, int j) const { return this->at<double>(i,j); }
    double & operator () (int i, int j)       { return this->at<double>(i,j); }
  
};
    
/*****************************************************************************/
// Classe per una matrice 4x4 double (64bit)
/*****************************************************************************/
class Mat4 : public cv::Mat {
  
  public:

    Mat4() : cv::Mat(4, 4, CV_64FC1, cv::Scalar(0)) { }
  
  
  double   operator () (int i, int j) const { return this->at<double>(i,j); }
  double & operator () (int i, int j)       { return this->at<double>(i,j); }
  
};

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


//#define EIGEN_OPENCV_SVD
#define EIGEN_OPENCV_EIGEN
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
    solution[i] = std::pair(fabs(eigenvalues(i)), eigenvectors.row(i).clone());
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


  
} /* namespace mpl */


/*****************************************************************************/
// Classe per una matrice double (64bit)
///*****************************************************************************/
//template <>
//class Mat<void> : public cv::Mat {
//
//  public:
//
//    Mat(uint32_t row, uint32_t col) : cv::Mat(3, 3, CV_64FC1, 0) { }
//
//};
//
//typedef Mat<void> Mat;


///*****************************************************************************/
//// namespace mpl
///*****************************************************************************/
//  namespace cv {
//
//  #if(0)
//
//
//
//
//
//  /*****************************************************************************/
//  // Classe per una matrice 4x4 double (64bit)
//  /*****************************************************************************/
//  class Mat5 : public cv::Mat {
//
//    public:
//
//      Mat5() : cv::Mat(5, 5, CV_64FC1, 0) { }
//
//  };
//
////  typedef cv::Matx<double, 3, 3> Mat3;
////  typedef cv::Matx<double, 4, 4> Mat4;
////  typedef cv::Vec<double, 3> Vec3;
//
//  #endif
//
//} /* namespace cv */


  
  template <typename T>
  cv::Point3_<T> operator * (const cv::Mat & M, const cv::Point3_<T> & p) {

    return cv::Mat(M * cv::Vec<T,3>(p.x, p.y, p.z)).at<cv::Point3_<T>>(0);
    
   // return cv::Mat(M * cv::Mat(p, false)).at<cv::Point3_<T>>(0);
        
  }


//template<typename _Tp> static inline
//Point4_<_Tp> operator * (const Matx<_Tp, 4, 4>& a, const Point4_<_Tp>& b)
//{
//    Matx<_Tp, 3, 1> tmp = a * Vec<_Tp,3>(b.x, b.y, b.z);
//    return Point4_<_Tp>(tmp.val[0], tmp.val[1], tmp.val[2]);
//}
  
//  cv::Mat M(3, 3, CV_64FC1);
//  cv::Point3d p;
//
//  cv::Point3d r = M * p;
  
#endif /* _H_MPL_MAT_H_ */
