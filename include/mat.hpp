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

    double   operator () (uint32_t i, uint32_t j) const { return this->at<double>(i,j); }
    double & operator () (uint32_t i, uint32_t j)       { return this->at<double>(i,j); }

};

/*****************************************************************************/
// Classe per una vettore generica
/*****************************************************************************/
class Vec : public cv::Mat {
  
  public:
  
    Vec() { }
  
    Vec(const Vec & vec) : cv::Mat(vec) { }
  
    Vec(const cv::Mat & mat) { *this = mat; }

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
    double   operator () (uint32_t i) const { return this->at<double>(i); }
    double & operator () (uint32_t i)       { return this->at<double>(i); }
  
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
  
    Mat3(const cv::MatExpr & mat) : cv::Mat(mat) { }
  
    Mat3(const cv::Mat & mat) : cv::Mat(mat) { }

    double   operator () (uint32_t i, uint32_t j) const { return this->at<double>(i,j); }
    double & operator () (uint32_t i, uint32_t j)       { return this->at<double>(i,j); }
  
};
    
/*****************************************************************************/
// Classe per una matrice 4x4 double (64bit)
/*****************************************************************************/
class Mat4 : public cv::Mat {
  
  public:

    Mat4() : cv::Mat(4, 4, CV_64FC1, cv::Scalar(0)) { }
  
    Mat4(const cv::MatExpr & mat) : cv::Mat(mat) { }
  
    Mat4(const cv::Mat & mat) : cv::Mat(mat) { }
  
  double   operator () (uint32_t i, uint32_t j) const { return this->at<double>(i,j); }
  double & operator () (uint32_t i, uint32_t j)       { return this->at<double>(i,j); }
  
};
  
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
