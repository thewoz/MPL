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

#ifndef _H_COBBS_OPENCV_H_
#define _H_COBBS_OPENCV_H_

#include <cstdlib>
#include <cstdio>

#include <string>
#include <ostream>

#include <opencv2/opencv.hpp>

#include <mpl/stdio.hpp>
#include <mpl/stdlib.hpp>


namespace cv {
  
  template<typename _Tp1, typename _Tp2>
  static inline Point_<_Tp1> operator - (const Point_<_Tp1> & a, const Point_<_Tp2> & b) {
    return Point_<_Tp1>( saturate_cast<_Tp1>(a.x - b.x), saturate_cast<_Tp1>(a.y - b.y));
  }
  
  
  template<typename _Tp1, typename _Tp2>
  static inline void operator /= (Point_<_Tp1> & a, const _Tp2 & b) {
    
    a.x /= b;
    a.y /= b;

  }

  template<typename _Tp1, typename _Tp2>
  static inline void operator /= (Point3_<_Tp1> & a, const _Tp2 & b) {
    
    a.x /= b;
    a.y /= b;
    a.z /= b;

  }
  
  /*****************************************************************************/
  // operator +=
  /*****************************************************************************/
  template<typename _TpA, typename _TpB> static inline
  Point_<_TpA>& operator += (Point_<_TpA>& a, const Point_<_TpB>& b)
  {
    a.x += b.x;
    a.y += b.y;
    return a;
  }

  /*****************************************************************************/
  // operator +=
  /*****************************************************************************/
  template<typename _TpA, typename _TpB> static inline
  Point3_<_TpA>& operator += (Point3_<_TpA>& a, const Point3_<_TpB>& b)
  {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
  }

  
  /*****************************************************************************/
  // operator /=
  /*****************************************************************************/
  //template<typename T, typename V>
  //inline void operator /= (cv::Point2_<T> & point, const V & value) {
   // point.x /= (double)value;
    //point.y /= (double)value;
  //}

  
}

//template<typename _Tp>
//inline cv::Point3_<_Tp> & cv::Point3_<_Tp>::operator = (const cv::Mat & mat){ }
//
//template<typename _Tp>
//inline cv::Mat::operator cv::Point3_<_Tp>() const { }
//
//template<> void cv::Point3d::operator = (const cv::Mat & mat) { }
//
//template<> void cv::Point3d::operator = (const cv::Mat & mat) { }


/*****************************************************************************/
// opencv
/*****************************************************************************/
namespace cv {
  
  /*****************************************************************************/
  // util
  /*****************************************************************************/
  namespace util {
    
    typedef uint32_t digit_t[5][3];
    
    const digit_t zero = {
      {1, 1, 1},
      {1, 0, 1},
      {1, 0, 1},
      {1, 0, 1},
      {1, 1, 1}
    };
    
    const digit_t uno = {
      {1, 0, 0},
      {1, 0, 0},
      {1, 0, 0},
      {1, 0, 0},
      {1, 0, 0}
    };
    
    const digit_t due = {
      {1, 1, 1},
      {0, 0, 1},
      {1, 1, 1},
      {1, 0, 0},
      {1, 1, 1}
    };
    
    const digit_t tre = {
      {1, 1, 1},
      {0, 0, 1},
      {0, 1, 1},
      {0, 0, 1},
      {1, 1, 1}
    };
    
    const digit_t quattro = {
      {1, 0, 0},
      {1, 0, 0},
      {1, 0, 1},
      {1, 1, 1},
      {0, 0, 1}
    };
    
    const digit_t cinque = {
      {1, 1, 1},
      {1, 0, 0},
      {1, 1, 1},
      {0, 0, 1},
      {1, 1, 1}
    };
    
    const digit_t sei = {
      {1, 0, 0},
      {1, 0, 0},
      {1, 1, 1},
      {1, 0, 1},
      {1, 1, 1}
    };
    
    const digit_t sette = {
      {1, 1, 1},
      {0, 0, 1},
      {0, 0, 1},
      {0, 0, 1},
      {0, 0, 1}
    };
    
    const digit_t otto = {
      {1, 1, 1},
      {1, 0, 1},
      {1, 1, 1},
      {1, 0, 1},
      {1, 1, 1}
    };
    
    const digit_t nove = {
      {1, 1, 1},
      {1, 0, 1},
      {1, 1, 1},
      {0, 0, 1},
      {0, 0, 1}
    };
    
    const digit_t * digits[10] = { &zero, &uno, &due, &tre, &quattro, &cinque, &sei, &sette, &otto, &nove };
    
    /*****************************************************************************/
    // drawDigit
    /*****************************************************************************/
    void drawDigit(cv::Mat & dst, uint32_t number, const cv::Point & pos, const cv::Vec3b & color) {
      
      const digit_t & digit = *digits[number];
      
      int32_t hTo = pos.y - 5;
      if(hTo < 0) hTo = 0;
      
      int32_t wTo = pos.x + 3;
      if(wTo >= dst.cols) wTo = dst.cols;
      
      for(uint32_t h=hTo, i=0; h<pos.y; ++h, ++i) {
        
        for(uint32_t w=pos.x, j=0; w<wTo; ++w, ++j) {
          
          if(digit[i][j] == 1) dst.at<cv::Vec3b>(h, w) = color;
          
        }
        
      }
      
    }
    
  } // namespace unit
  
  
#ifdef __APPLE__

#define CV_KEY_ARROW_UP     0
#define CV_KEY_ARROW_RIGHT  3
#define CV_KEY_ARROW_DOWN   1
#define CV_KEY_ARROW_LEFT   2
#define CV_KEY_RETURN      13
#define CV_KEY_ESC         27

#else
  
#define CV_KEY_ARROW_UP    82
#define CV_KEY_ARROW_RIGHT 83
#define CV_KEY_ARROW_DOWN  84
#define CV_KEY_ARROW_LEFT  81
#define CV_KEY_RETURN      13
#define CV_KEY_ESC         27

#endif
  
#define CV_KEY_C 99
  
  
  /*****************************************************************************/
  // drawString
  /*****************************************************************************/
  void drawString(cv::Mat & dst, const char * str, cv::Point pos, const cv::Vec3b & color){
    
    if(dst.channels() != 3){
      fprintf(stderr, "The destination image is not a three channel image.\n");
      exit(EXIT_FAILURE);
    }
    
    uint32_t size = (uint32_t) strlen(str);
    
    //printf("DDDD %s (%d,%d)\n", str, pos.x, pos.y);
    
    for(uint32_t i=0; i<size; ++i){
      
      util::drawDigit(dst, str[i]-48, pos, color);
      
      pos.x += 4;
      
    }
    
  }
  
  /*****************************************************************************/
  // drawString
  /*****************************************************************************/
  void drawString(cv::Mat & dst, uint32_t number, const cv::Point & pos, const cv::Vec3b & color){
    
    char str[PATH_MAX];
    
    std::itoa(number, str);
    
    drawString(dst, str, pos, color);
    
  }
  
  
  /*****************************************************************************/
  // save
  /*****************************************************************************/
  void save(cv::Mat & image, const char * format, ...){
    
    char str[PATH_MAX];
    
    va_list ap;
    
    va_start(ap, format);
    
    vsprintf(str, format, ap);
    
    va_end(ap);
    
    mpl::io::expandPath(str);
    
    cv::imwrite(str, image);
    
  }

  /*****************************************************************************/
  // save
  /*****************************************************************************/
  void save(cv::Mat & image, const std::string & str){ save(image, str.c_str()); }
  
  /*****************************************************************************/
  // open
  /*****************************************************************************/
  cv::Mat open(uint32_t mode, const char * format, ...){
    
    char str[PATH_MAX];
    
    va_list ap;
    
    va_start(ap, format);
    
    vsprintf(str, format, ap);
    
    va_end(ap);
    
    mpl::io::expandPath(str);
    
    cv::Mat image = cv::imread(str, mode);
    
    if(image.data==NULL){
      fprintf(stderr, "error in opening the image '%s'\n", str);
      exit(EXIT_FAILURE);
    }
    
    return image;
    
  }

  /*****************************************************************************/
  // open
  /*****************************************************************************/
  cv::Mat open(const char * format, ...){
    
    char str[PATH_MAX];
    
    va_list ap;
    
    va_start(ap, format);
    
    vsprintf(str, format, ap);
    
    va_end(ap);
    
    mpl::io::expandPath(str);
    
    return open(cv::IMREAD_UNCHANGED, str);
  
  }

  /*****************************************************************************/
  // open
  /*****************************************************************************/
  cv::Mat open(uint32_t mode, const std::string & str){ return open(mode, str.c_str()); }
  cv::Mat open(const std::string & str){ return open(cv::IMREAD_UNCHANGED, str.c_str()); }

  /*****************************************************************************/
  // median
  /*****************************************************************************/
  //TODO: testare con minvAlue ecc
  double median(cv::Mat src, uint32_t minValue = 0, uint32_t maxValue = 256) {
    
    int histSize = 256;
    
    float range[] = { static_cast<float>(minValue), static_cast<float>(maxValue) };
    
    const float* histRange = { range };
    
    cv::Mat hist;
    
    cv::calcHist( &src, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);
    
    double m = (src.rows*src.cols) / 2.0;
    
    uint32_t bin = 0;
    
    double med = -1.0;
    
    for(uint32_t i = 0; i < histSize && med < 0.0; ++i) {
      
      bin += cvRound(hist.at<float>(i));
      
      if(bin>m && med<0.0) med = i;
      
    }
    
    return med;
    
  }
  
//  std::ostream & operator << (std::ostream & os, const cv::Point2f & point) {
//    os << point.x << ',' << point.y;
//    return os;
//  }
//
//  std::ostream & operator << (std::ostream & os, const cv::Point3f & point) {
//    os << point.x << ',' << point.y << ',' << point.x;
//    return os;
//  }
//
//  cv::Point2f & operator = (const std::string & str) {
//
//    stfd::vector<std::string> tokens;
//
//    std::parse(str, ",", tokens);
//
//    if(tokens.szie() != 2) {
//      fprintf(strerr, "eerror\n");
//      abort();
//    }
//
//    return cv::Point2f(atof(tokens[0]), atof(tokens[1]));
//
//  }
//
//  cv::Point3f & operator = (const std::string & str) {
//
//    stfd::vector<std::string> tokens;
//
//    std::parse(str, ",", tokens);
//
//    if(tokens.szie() != 3) {
//      fprintf(strerr, "eerror\n");
//      abort();
//    }
//
//    return cv::Point3f(atof(tokens[0]), atof(tokens[1]), atof(tokens[3]));
//
//  }
  
} /* namespace opencv */

#endif /* _H_COBBS_OPENCV_H_ */


/*****************************************************************************/
// save
/*****************************************************************************/
/*
void asOpencv::save(const char * filename, cv::Mat & img){
  
  std::vector<int> p(2, 0);
  
  const char * ext = extension(filename);

  if(strcmp(ext, "png")==0){
  
    p[0] = CV_IMWRITE_PNG_COMPRESSION;
    p[1] = png_compression;

  }

  if(strcmp(ext, "jpg")==0){

    p[0] = CV_IMWRITE_JPEG_QUALITY;
    p[1] = jpg_quality;

  }

  cv::imwrite(filename, img, p);
  
}
 
 */
