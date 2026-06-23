/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017-2026
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

#ifndef _H_MPL_MATH_MATRIX_H_
#define _H_MPL_MATH_MATRIX_H_

#include <opencv2/opencv.hpp>

//*****************************************************************************/
// namespace mpl
//*****************************************************************************/
namespace mpl {
  
//*****************************************************************************/
// Classe per una matrice generica
//*****************************************************************************/
class Mat : public cv::Mat {

  public:
  
    Mat() : cv::Mat() { }  

    Mat(const cv::MatExpr & mat) : cv::Mat(mat) { }
  
    Mat(const cv::Mat & mat) : cv::Mat(mat) { }

    Mat(int row, int col) : cv::Mat(row, col, CV_64FC1, cv::Scalar(0)) { }

    void resize(int row, int col) {
      if(empty()) create(cv::Size(col, row), CV_64FC1);
      else cv::resize(*this, *this, cv::Size(col, row));
    }

    //cv::Mat row(int i) { return this->at<cv::Mat>(i); }

    double   operator () (int i, int j) const { return this->at<double>(i,j); }
    double & operator () (int i, int j)       { return this->at<double>(i,j); }

};

//*****************************************************************************/
// Classe per una vettore generica
//*****************************************************************************/
class Vec : public cv::Mat {
  
  public:
  
    Vec() { }
  
    Vec(const Vec & vec) : cv::Mat(vec) { }
  
    Vec(const cv::Mat & mat) { *this = mat; }

    Vec(int size) : cv::Mat(size, 1, CV_64FC1, cv::Scalar(0)) { }

    //*****************************************************************************/
    // resize() -
    //*****************************************************************************/
    void resize(int size) {
      if(empty()) create(cv::Size(1, size), CV_64FC1);
      else cv::resize(*this, *this, cv::Size(1, size));
    }

    //*****************************************************************************/
    // operator ()
    //*****************************************************************************/
    // NOTE: forse non serve lo 0 inziale
    double   operator () (int i) const { return this->at<double>(i); }
    double & operator () (int i)       { return this->at<double>(i); }
  
    //*****************************************************************************/
    // operator =
    //*****************************************************************************/
    Vec & operator = (const cv::Mat & mat) {
    
      if(mat.channels() != 1) {
        fprintf(stderr, "mpl::Vec::operator=() error: matrix must have only one channel\n");
        abort();
      }
    
      // if(mat.depth() != CV_64F) {
      //   fprintf(stderr, "Matrix must be a 64bit float one\n");
      //   abort();
      // }

      if(mat.rows != 1 && mat.cols != 1) {
        fprintf(stderr, "mpl::Vec::operator=() error: matrix must be a 1xN or Nx1 matrix\n");
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

        mat.col(0).copyTo(this->col(0));

      }
    
      return *this;

    }
  
};

  //*****************************************************************************/
  // Classe per una matrice 3x3 double (64bit)
  //*****************************************************************************/
  class Mat3 : public cv::Mat {
    
    public:
    
      Mat3() : cv::Mat(3, 3, CV_64FC1, cv::Scalar(0)) { }
    
      Mat3(const cv::MatExpr & mat) : cv::Mat(mat) { }
    
      Mat3(const cv::Mat & mat) : cv::Mat(mat) { }

      double   operator () (int i, int j) const { return this->at<double>(i,j); }
      double & operator () (int i, int j)       { return this->at<double>(i,j); }

  };

  //*****************************************************************************/
  // Classe per una matrice 4x4 double (64bit)
  //*****************************************************************************/
  class Mat4 : public cv::Mat {

    public:

      Mat4() : cv::Mat(4, 4, CV_64FC1, cv::Scalar(0)) { }

      Mat4(const cv::MatExpr & mat) : cv::Mat(mat) { }

      Mat4(const cv::Mat & mat) : cv::Mat(mat) { }

    double   operator () (int i, int j) const { return this->at<double>(i,j); }
    double & operator () (int i, int j)       { return this->at<double>(i,j); }
    
  };
  
} // namespace mpl


  //*****************************************************************************/
  // operator *
  //*****************************************************************************/
  template <typename T>
  cv::Point3_<T> operator * (const cv::Mat & M, const cv::Point3_<T> & p) {

    return cv::Mat(M * cv::Vec<T,3>(p.x, p.y, p.z)).at<cv::Point3_<T>>(0);
            
  }

  //*****************************************************************************/
  // namespace cv::mat
  //*****************************************************************************/
  namespace cv::mat {

    //*****************************************************************************/
    // printMat()
    //*****************************************************************************/
    void printMat(const cv::Mat & A) {
      
      for(int i=0; i<A.rows; ++i) {
        for(int j=0; j<A.cols; ++j)
          printf("%g ", A.at<double>(i,j));
        printf("\n");
      }
      
    }
  
    //*****************************************************************************/
    // println()
    //*****************************************************************************/
    void println(const cv::Mat & A) {
      
      printMat(A);
      
      printf("\n");
      
    }

    //*****************************************************************************/
    // println()
    //*****************************************************************************/
    void println(const std::string & str, const cv::Mat & A) {
      
      printf("%s", str.c_str());
      
      printMat(A);
      
      printf("\n");
      
    }

  } // namespace cv::mat

#endif // _H_MPL_MATH_MATRIX_H_
