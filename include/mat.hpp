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
  
  private:
  
 // std::vector<std::vector<T>> data;
  
  public:
  
//    Mat(uint32_t row, uint32_t col) : cv::Mat(row, col, CV_64FC1, 0) {
//
//     // if constexpr (std::is_same<T, double>::value) {
//    //    data = std::vector<std::vector<T>>(row, std::vector<T>(col));
//    //  } else {
//  //     *this(row, col, CV_64FC1, 0);
//   //   }
//
//    }

    Mat() : cv::Mat() { }  

    Mat(const cv::MatExpr & mat) : cv::Mat(mat) { }
  
    Mat(const cv::Mat & mat) : cv::Mat(mat) { }

    Mat(uint32_t row, uint32_t col) : cv::Mat(row, col, CV_64FC1, cv::Scalar(0)) {
  
    //  if constexpr (std::is_same<T, double>::value) {
   //     data = std::vector<std::vector<T>>(row, std::vector<T>(col, value));
   //   } else {
   //     *this(row, col, CV_64FC1, value);
   //   }

    }
  
  Mat(const Mat & mat) : cv::Mat(mat) { }
  
 // Mat(uint32_t col) : cv::Mat(1, col, CV_64FC1, 0) { }
  
  //Mat(uint32_t col, const double & value = 0) : cv::Mat(1, col, CV_64FC1, value) { }

  
  double   operator () (int i, int j) const { return this->at<double>(i,j); }
  double & operator () (int i, int j)       { return this->at<double>(i,j); }
  
//  Mat & operator = (const cv::MatExpr & mat) {
//
//    cv::Mat::operator=(mat);
//
//    return *this;
//
//  }
  

};

/*****************************************************************************/
// Classe per una vettore generica
/*****************************************************************************/
class Vec : public cv::Mat {
  
  public:
  
  Vec() { }
  
    //Vec(uint32_t size) : Mat(1, size) { }
    
  Vec(Vec & vec) : cv::Mat(vec) { }

    Vec(uint32_t size) : cv::Mat(size, 1, CV_64FC1, cv::Scalar(0)) { }
  
  // NOTE: forse non serve lo 0 inziale
  double   operator () (int i) const { return this->at<double>(i); }
  double & operator () (int i)       { return this->at<double>(i); }
  
  Vec & operator = (const cv::Mat & mat) {
    
    if(mat.channels() != 1) {
      fprintf(stderr, "Matrix must have only one chanel\n");
      abort();
    }
    
 //   if(mat.depth() != CV_64F) {
 //     fprintf(stderr, "Matrix must be a 64bit float one\n");
 //     abort();
  //  }
    
    printf("mat.rows: %d\n", mat.rows);
    printf("mat.cols: %d\n", mat.cols);

    if(mat.rows != 1 && mat.cols != 1) {
      fprintf(stderr, "Matrix must be a 1xN or a Nx1 matrix\n");
      abort();
    }
    
    if(mat.rows == 1) {

      cv::resize(*this, *this, cv::Size(1, mat.cols));
      
      std::cout << "row: " << mat.row(0) << std::endl;
      
      cv::Mat tmp = mat.t();
      
      tmp.col(0).copyTo(this->col(0));
      
      std::cout << "row: " << this->col(0) << std::endl;

      
    }
    
    if(mat.cols == 1) {

      cv::resize(*this, *this, cv::Size(1, mat.rows));
      
      std::cout << "col: " << mat.col(0) << std::endl;
      
      cv::Mat tmp = mat.t();
      
      mat.col(0).copyTo(this->col(0));

      std::cout << "col: " << this->col(0) << std::endl;

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

    return cv::Mat(M * cv::Mat(p, false)).at<cv::Point3_<T>>(1);
    
    //return ret;
    
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