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

#ifndef _H_MPL_OPENCV_H_
#define _H_MPL_OPENCV_H_

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
  
  template<typename _TpA, typename _TpB> static inline
  Point_<_TpA>& operator += (Point_<_TpA>& a, const Point_<_TpB>& b)
  {
    a.x += b.x;
    a.y += b.y;
    return a;
  }

 
  template<typename _TpA, typename _TpB> static inline
  Point3_<_TpA>& operator += (Point3_<_TpA> & a, const Point3_<_TpB> & b)
  {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
  }

  template<typename TA, typename TB>
  static inline Point_<TA> operator + (const Point_<TA> & a, const TB & b) {

    Point_<TA> point = a;

    point.x += b;
    point.y += b;

    return point;

  }
  
  
  template <class T1, class T2>
  inline double norm(const cv::Point_<T1> & a, const cv::Point_<T2> & b) {
    
    return std::sqrt(((a.x-b.x) * (a.x-b.x)) + ((a.y-b.y) * (a.y-b.y)));
    
  }
  
  template <class T1, class T2>
  inline double norm(const cv::Point3_<T1> & a, const cv::Point3_<T2> & b) {
    
    return std::sqrt(((a.x-b.x) * (a.x-b.x)) + ((a.y-b.y) * (a.y-b.y)) + ((a.z-b.z) * (a.z-b.z)));
    
  }
  

  // NOTE: in opencv the color are in the BGR order
  const cv::Vec3b Red(0, 0, 255);
  const cv::Vec3b Blue(255, 0, 0);
  const cv::Vec3b Green(0, 255, 0);
  const cv::Vec3b Yellow(0, 255, 255);
  const cv::Vec3b Magenta(255, 0, 255);
  const cv::Vec3b White(255, 255, 255);
  const cv::Vec3b Black(0, 0, 0);
  const cv::Vec3b LightGrey(211, 211, 211);

  const cv::Scalar Zero(0, 0, 0);
  const cv::Scalar One(255, 255, 255);

#ifdef __APPLE__

#define CV_KEY_ARROW_UP    126
#define CV_KEY_ARROW_RIGHT 124
#define CV_KEY_ARROW_DOWN  125
#define CV_KEY_ARROW_LEFT  123
#define CV_KEY_RETURN       13
#define CV_KEY_ESC          27
#define CV_KEY_SPACE        32
#define CV_KEY_DELETE        8

#else
  
#define CV_KEY_ARROW_UP    82
#define CV_KEY_ARROW_RIGHT 83
#define CV_KEY_ARROW_DOWN  84
#define CV_KEY_ARROW_LEFT  81
#define CV_KEY_RETURN      13
#define CV_KEY_ESC         27
#define CV_KEY_SPACE       32
#define CV_KEY_DELETE       8

#endif
  
#define CV_KEY_0           48
#define CV_KEY_1           49
#define CV_KEY_2           50
#define CV_KEY_3           51
#define CV_KEY_4           52
#define CV_KEY_5           53
#define CV_KEY_6           54
#define CV_KEY_7           55
#define CV_KEY_8           56
#define CV_KEY_9           57

#define CV_KEY_A           97
#define CV_KEY_B           98
#define CV_KEY_C           99
#define CV_KEY_D          100
#define CV_KEY_E          101
#define CV_KEY_F          102
#define CV_KEY_G          103
#define CV_KEY_H          104
#define CV_KEY_I          105
#define CV_KEY_J          106
#define CV_KEY_K          107
#define CV_KEY_L          108
#define CV_KEY_M          109
#define CV_KEY_N          110
#define CV_KEY_O          111
#define CV_KEY_P          112
#define CV_KEY_Q          113
#define CV_KEY_R          114
#define CV_KEY_S          115
#define CV_KEY_T          116
#define CV_KEY_U          117
#define CV_KEY_V          118
#define CV_KEY_W          119
#define CV_KEY_X          120
#define CV_KEY_Y          121
#define CV_KEY_Z          122

  //*****************************************************************************/
  // util
  //*****************************************************************************/
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
      {0, 1, 0},
      {0, 1, 0},
      {0, 1, 0},
      {0, 1, 0},
      {0, 1, 0}
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
    
    //*****************************************************************************/
    // drawDigit
    //*****************************************************************************/
    void drawDigit(cv::Mat & dst, uint32_t number, const cv::Point & pos, const cv::Vec3b & color) {

      const digit_t & digit = *digits[number];
      
      int32_t hTo = pos.y - 5;
      int32_t hFrom = pos.y;

      if(hTo < 0) hTo = 0;
      if(hFrom > dst.rows) hFrom = dst.rows;

      int32_t wTo = pos.x + 3;
      if(wTo >= dst.cols) wTo = dst.cols;
      
      for(int32_t h=hTo, i=0; h<hFrom; ++h, ++i) {
        
        for(int32_t w=pos.x, j=0; w<wTo; ++w, ++j) {
          
          if(h < 0 || h > dst.rows) continue;
          if(w < 0 || w > dst.cols) continue;

          if(digit[i][j] == 1) dst.at<cv::Vec3b>(h, w) = color;
          
        }
        
      }
      
    }
    
  } // namespace util
  
  
  //*****************************************************************************/
  // drawString
  //*****************************************************************************/
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
  
  //*****************************************************************************/
  // drawNumber
  //*****************************************************************************/
  void drawNumber(cv::Mat & dst, uint32_t number, const cv::Point & pos, const cv::Vec3b & color){
    
    char str[PATH_MAX];
    
    std::itoa(number, str);
    
    drawString(dst, str, pos, color);
    
  }

  //*****************************************************************************/
  // drawParabola
  //*****************************************************************************/
  void drawParabola(cv::Mat & dst, cv::Vec3d & coeffs, const cv::Vec3b & color, const cv::Point2d & offset = cv::Point2d(0,0)) {
    
     for(int x=0; x<dst.cols; ++x) {
       
       double y = coeffs[0] * x * x + coeffs[1] * x + coeffs[2];
       
       cv::Point pt(x, static_cast<int>(y));
       
       if(pt.y >= 0 && pt.y < dst.rows){
         if(dst.channels() == 1) dst.at<uchar>(pt) = color[0];
         if(dst.channels() == 3) dst.at<Vec3b>(pt) = color;
       }
         
       
     }
    
  }

  //*****************************************************************************/
  // cross
  //*****************************************************************************/
  void cross(cv::Mat & dst, const cv::Point & pos, const cv::Vec3b & color) {
    cv::circle(dst, pos, 1, color);
  }

  //*****************************************************************************/
  // minEnclosureRoi
  //*****************************************************************************/
  cv::Rect minEnclosureRoi(const cv::Rect & roi1, const cv::Rect & roi2) {
    
    cv::Rect roi;
    
    roi.x = (roi1.x < roi2.x) ? roi1.x : roi2.x;
    roi.y = (roi1.y < roi2.y) ? roi1.y : roi2.y;

    cv::Point p1 = cv::Point(roi1.x + roi1.width - roi.x, roi1.y + roi1.height - roi.y);
    cv::Point p2 = cv::Point(roi2.x + roi2.width - roi.x, roi2.y + roi2.height - roi.y);

    roi.width  = (p1.x > p2.x) ? p1.x : p2.x;
    roi.height = (p1.y > p2.y) ? p1.y : p2.y;

    return roi;
    
  }

  //*****************************************************************************/
  // getRandomColor
  //*****************************************************************************/
  cv::Vec3b getRandomColor() {

    cv::Vec3b color;

    color.val[0] = (uchar)((rand() / (double) RAND_MAX) * 255);
    color.val[1] = (uchar)((rand() / (double) RAND_MAX) * 255);
    color.val[2] = (uchar)((rand() / (double) RAND_MAX) * 255);

    return color;
        
  }
 
  //*****************************************************************************/
  // RGBtoHex:
  // Function to convert the RGB code to Hex color code
  //*****************************************************************************/
  std::string RGBtoHex(int R, int G, int B) {
    
    if((R >= 0 && R <= 255) && (G >= 0 && G <= 255) && (B >= 0 && B <= 255)) {
   
      std::string hexCode = "#";
                  hexCode += std::decToHex(R);
                  hexCode += std::decToHex(G);
                  hexCode += std::decToHex(B);
   
      return hexCode;
      
    } else return "-1";
    
  }


  //*****************************************************************************/
  // save
  //*****************************************************************************/
  bool save(cv::Mat & image, std::string str){
    
    mpl::io::expandPath(str);

    int status = cv::imwrite(str, image);
    
    if(!status) {
      fprintf(stderr, "error in opening the file '%s'\n", str.c_str());
      abort();
    }
    
    return true;
    
  }
    
  //*****************************************************************************/
  // open
  //*****************************************************************************/
  cv::Mat open(std::string str, int mode = cv::IMREAD_UNCHANGED){
    
    mpl::io::expandPath(str);
    
    cv::Mat image = cv::imread(str, mode);
    
    if(image.data==NULL){
      fprintf(stderr, "error in opening the image '%s'\n", str.c_str());
      exit(EXIT_FAILURE);
    }
    
    return image;
    
  }

  //*****************************************************************************/
  // open
  //*****************************************************************************/
//  void open(std::string str, cv::Mat & image, uint32_t mode = cv::IMREAD_UNCHANGED){
//    
//    mpl::io::expandPath(str);
//    
//    cv::imread(str, image, mode);
//    
//    if(image.data==NULL){
//      fprintf(stderr, "error in opening the image '%s'\n", str.c_str());
//      exit(EXIT_FAILURE);
//    }
//      
//  }

  
  
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
#endif


  //****************************************************************************/
  // histogram
  //****************************************************************************/
  void histogram(const cv::Mat & src, cv::Mat & histo, cv::Mat & mask, int valueToSkip = -1) {
    
    if(src.channels() > 1){
      fprintf(stderr, "The input image must have one channel.\n");
      exit(EXIT_FAILURE);
    }
    
    int histSize = 256;
    float range[] = { 0, 256 };
    const float * histRange = { range };
    
    if(valueToSkip != 1) {
      
      if(mask.empty()) mask = cv::Mat::ones(src.size(), CV_8UC1);
      
      for(int i=0; i<src.rows; ++i)
        for(int j=0; j<src.cols; ++j)
          if(src.at<uchar>(i,j) == valueToSkip) mask.at<uchar>(i,j) = 0;
      
    }
    
    cv::calcHist(&src, 1, 0, mask, histo, 1, &histSize, &histRange);
    
  }

  //****************************************************************************/
  // histogram
  //****************************************************************************/
  void histogram(const cv::Mat & src, cv::Mat & histo, int valueToSkip = -1) {
    
    cv::Mat mask = cv::Mat();
    
    histogram(src, histo, mask, valueToSkip);
    
  }



//// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
//// Returns a positive value, if OAB makes a counter-clockwise turn,
//// negative for clockwise turn, and zero if the points are collinear.
template <typename T>
T cross(const cv::Point_<T> & O, const cv::Point_<T> & A, const cv::Point_<T> & B)
{
  return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

template <typename T>
bool operator < (const cv::Point_<T> & p1, const cv::Point_<T> & p2) {
  
    return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
  
}
  

/////////////////////////////// Point4_ ////////////////////////////////

/** @brief Template class for 4D points specified by its coordinates `x`, `y` , `z` and  `w`.

An instance of the class is interchangeable with the C structure CvPoint2D32f . Similarly to
Point_ , the coordinates of 4D points can be converted to another type. The vector arithmetic and
comparison operations are also supported.

The following Point4_\<\> aliases are available:
@code
    typedef Point4_<int> Point4i;
    typedef Point4_<float> Point4f;
    typedef Point4_<double> Point4d;
@endcode
@see cv::Point4i, cv::Point4f and cv::Point4d
*/
template<typename _Tp> class Point4_
{
public:
    typedef _Tp value_type;

    //! default constructor
    Point4_();
    Point4_(_Tp _x, _Tp _y, _Tp _z, _Tp _w);
    Point4_(const Point4_& pt);
    Point4_(Point4_&& pt) CV_NOEXCEPT;
    explicit Point4_(const Point_<_Tp>& pt);
  explicit Point4_(const Point3_<_Tp>& pt);
    Point4_(const Vec<_Tp, 4>& v);

    Point4_& operator = (const Point4_& pt);
    Point4_& operator = (Point4_&& pt) CV_NOEXCEPT;
    //! conversion to another data type
    template<typename _Tp2> operator Point4_<_Tp2>() const;
    //! conversion to cv::Vec<>
    operator Vec<_Tp, 4>() const;

    //! dot product
    _Tp dot(const Point4_& pt) const;
    //! dot product computed in double-precision arithmetics
    double ddot(const Point4_& pt) const;
    //! cross product of the 2 4D points
    Point4_ cross(const Point4_& pt) const;
    _Tp x; //!< x coordinate of the 4D point
    _Tp y; //!< y coordinate of the 4D point
    _Tp z; //!< z coordinate of the 4D point
    _Tp w; //!< w coordinate of the 4D point

};

typedef Point4_<int> Point4i;
typedef Point4_<float> Point4f;
typedef Point4_<double> Point4d;

template<typename _Tp> class DataType< Point4_<_Tp> >
{
public:
    typedef Point4_<_Tp>                               value_type;
    typedef Point4_<typename DataType<_Tp>::work_type> work_type;
    typedef _Tp                                        channel_type;

    enum { generic_type = 0,
           channels     = 4,
           fmt          = traits::SafeFmt<channel_type>::fmt + ((channels - 1) << 8)
#ifdef OPENCV_TRAITS_ENABLE_DEPRECATED
           ,depth        = DataType<channel_type>::depth
           ,type         = CV_MAKETYPE(depth, channels)
#endif
         };

    typedef Vec<channel_type, channels> vec_type;
};

namespace traits {
template<typename _Tp>
struct Depth< Point4_<_Tp> > { enum { value = Depth<_Tp>::value }; };
template<typename _Tp>
struct Type< Point4_<_Tp> > { enum { value = CV_MAKETYPE(Depth<_Tp>::value, 4) }; };
} // namespace

//////////////////////////////// 3D Point ///////////////////////////////

template<typename _Tp> inline
Point4_<_Tp>::Point4_()
    : x(0), y(0), z(0), w(0) {}

template<typename _Tp> inline
Point4_<_Tp>::Point4_(_Tp _x, _Tp _y, _Tp _z, _Tp _w)
    : x(_x), y(_y), z(_z), w(_w) {}

template<typename _Tp> inline
Point4_<_Tp>::Point4_(const Point4_& pt)
    : x(pt.x), y(pt.y), z(pt.z), w(pt.w) {}

template<typename _Tp> inline
Point4_<_Tp>::Point4_(Point4_&& pt) CV_NOEXCEPT
    : x(std::move(pt.x)), y(std::move(pt.y)), z(std::move(pt.z)), w(std::move(pt.w)) {}

template<typename _Tp> inline
Point4_<_Tp>::Point4_(const Point_<_Tp>& pt)
    : x(pt.x), y(pt.y), z(_Tp()), w(_Tp()) {}

template<typename _Tp> inline
Point4_<_Tp>::Point4_(const Point3_<_Tp>& pt)
    : x(pt.x), y(pt.y), z(pt.z), w(_Tp()) {}
                          
template<typename _Tp> inline
Point4_<_Tp>::Point4_(const Vec<_Tp, 4>& v)
    : x(v[0]), y(v[1]), z(v[2]), w(v[3]) {}

template<typename _Tp> template<typename _Tp2> inline
Point4_<_Tp>::operator Point4_<_Tp2>() const
{
    return Point4_<_Tp2>(saturate_cast<_Tp2>(x), saturate_cast<_Tp2>(y), saturate_cast<_Tp2>(z), saturate_cast<_Tp2>(w));
}

template<typename _Tp> inline
Point4_<_Tp>::operator Vec<_Tp, 4>() const
{
    return Vec<_Tp, 4>(x, y, z, w);
}

template<typename _Tp> inline
Point4_<_Tp>& Point4_<_Tp>::operator = (const Point4_& pt)
{
    x = pt.x; y = pt.y; z = pt.z; w = pt.w;
    return *this;
}

template<typename _Tp> inline
Point4_<_Tp>& Point4_<_Tp>::operator = (Point4_&& pt) CV_NOEXCEPT
{
    x = std::move(pt.x); y = std::move(pt.y); z = std::move(pt.z); w = std::move(pt.w);
    return *this;
}

template<typename _Tp> inline
_Tp Point4_<_Tp>::dot(const Point4_& pt) const
{
    return saturate_cast<_Tp>(x*pt.x + y*pt.y + z*pt.z + w*pt.w);
}

template<typename _Tp> inline
double Point4_<_Tp>::ddot(const Point4_& pt) const
{
    return (double)x*pt.x + (double)y*pt.y + (double)z*pt.z + (double)w*pt.w;
}

//template<typename _Tp> inline
//Point4_<_Tp> Point4_<_Tp>::cross(const Point4_<_Tp>& pt) const
//{
//    return Point4_<_Tp>(y*pt.z - z*pt.y, z*pt.x - x*pt.z, x*pt.y - y*pt.x);
//}


template<typename _Tp> static inline
Point4_<_Tp>& operator += (Point4_<_Tp>& a, const Point4_<_Tp>& b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
    return a;
}

template<typename _Tp> static inline
Point4_<_Tp>& operator -= (Point4_<_Tp>& a, const Point4_<_Tp>& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
  a.w -= b.w;
    return a;
}

template<typename _Tp> static inline
Point4_<_Tp>& operator *= (Point4_<_Tp>& a, int b)
{
    a.x = saturate_cast<_Tp>(a.x * b);
    a.y = saturate_cast<_Tp>(a.y * b);
    a.z = saturate_cast<_Tp>(a.z * b);
    a.w = saturate_cast<_Tp>(a.w * b);
    return a;
}

template<typename _Tp> static inline
Point4_<_Tp>& operator *= (Point4_<_Tp>& a, float b)
{
    a.x = saturate_cast<_Tp>(a.x * b);
    a.y = saturate_cast<_Tp>(a.y * b);
    a.z = saturate_cast<_Tp>(a.z * b);
    a.w = saturate_cast<_Tp>(a.w * b);
    return a;
}

template<typename _Tp> static inline
Point4_<_Tp>& operator *= (Point4_<_Tp>& a, double b)
{
    a.x = saturate_cast<_Tp>(a.x * b);
    a.y = saturate_cast<_Tp>(a.y * b);
    a.z = saturate_cast<_Tp>(a.z * b);
    a.w = saturate_cast<_Tp>(a.w * b);
    return a;
}

template<typename _Tp> static inline
Point4_<_Tp>& operator /= (Point4_<_Tp>& a, int b)
{
    a.x = saturate_cast<_Tp>(a.x / b);
    a.y = saturate_cast<_Tp>(a.y / b);
    a.z = saturate_cast<_Tp>(a.z / b);
    a.w = saturate_cast<_Tp>(a.w / b);
    return a;
}

template<typename _Tp> static inline
Point4_<_Tp>& operator /= (Point4_<_Tp>& a, float b)
{
    a.x = saturate_cast<_Tp>(a.x / b);
    a.y = saturate_cast<_Tp>(a.y / b);
    a.z = saturate_cast<_Tp>(a.z / b);
    a.w = saturate_cast<_Tp>(a.w / b);
    return a;
}

template<typename _Tp> static inline
Point4_<_Tp>& operator /= (Point4_<_Tp>& a, double b)
{
    a.x = saturate_cast<_Tp>(a.x / b);
    a.y = saturate_cast<_Tp>(a.y / b);
    a.z = saturate_cast<_Tp>(a.z / b);
    a.w = saturate_cast<_Tp>(a.w / b);
    return a;
}

template<typename _Tp> static inline
double norm(const Point4_<_Tp>& pt)
{
    return std::sqrt((double)pt.x*pt.x + (double)pt.y*pt.y + (double)pt.z*pt.z + (double)pt.w*pt.w);
}

template<typename _Tp> static inline
bool operator == (const Point4_<_Tp>& a, const Point4_<_Tp>& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

template<typename _Tp> static inline
bool operator != (const Point4_<_Tp>& a, const Point4_<_Tp>& b)
{
    return a.x != b.x || a.y != b.y || a.z != b.z || a.w != b.w;
}

template<typename _Tp> static inline
Point4_<_Tp> operator + (const Point4_<_Tp>& a, const Point4_<_Tp>& b)
{
    return Point4_<_Tp>( saturate_cast<_Tp>(a.x + b.x), saturate_cast<_Tp>(a.y + b.y), saturate_cast<_Tp>(a.z + b.z), saturate_cast<_Tp>(a.w + b.w));
}

template<typename _Tp> static inline
Point4_<_Tp> operator - (const Point4_<_Tp>& a, const Point4_<_Tp>& b)
{
    return Point4_<_Tp>( saturate_cast<_Tp>(a.x - b.x), saturate_cast<_Tp>(a.y - b.y), saturate_cast<_Tp>(a.z - b.z), saturate_cast<_Tp>(a.w - b.w));
}

template<typename _Tp> static inline
Point4_<_Tp> operator - (const Point4_<_Tp>& a)
{
    return Point4_<_Tp>( saturate_cast<_Tp>(-a.x), saturate_cast<_Tp>(-a.y), saturate_cast<_Tp>(-a.z), saturate_cast<_Tp>(-a.w) );
}

template<typename _Tp> static inline
Point4_<_Tp> operator * (const Point4_<_Tp>& a, int b)
{
    return Point4_<_Tp>( saturate_cast<_Tp>(a.x*b), saturate_cast<_Tp>(a.y*b), saturate_cast<_Tp>(a.z*b), saturate_cast<_Tp>(a.w*b));
}

template<typename _Tp> static inline
Point4_<_Tp> operator * (int a, const Point4_<_Tp>& b)
{
    return Point4_<_Tp>( saturate_cast<_Tp>(b.x * a), saturate_cast<_Tp>(b.y * a), saturate_cast<_Tp>(b.z * a), saturate_cast<_Tp>(b.w * a) );
}

template<typename _Tp> static inline
Point4_<_Tp> operator * (const Point4_<_Tp>& a, float b)
{
    return Point4_<_Tp>( saturate_cast<_Tp>(a.x * b), saturate_cast<_Tp>(a.y * b), saturate_cast<_Tp>(a.z * b), saturate_cast<_Tp>(a.w * b) );
}

template<typename _Tp> static inline
Point4_<_Tp> operator * (float a, const Point4_<_Tp>& b)
{
    return Point4_<_Tp>( saturate_cast<_Tp>(b.x * a), saturate_cast<_Tp>(b.y * a), saturate_cast<_Tp>(b.z * a), saturate_cast<_Tp>(b.w * a) );
}

template<typename _Tp> static inline
Point4_<_Tp> operator * (const Point4_<_Tp>& a, double b)
{
    return Point4_<_Tp>( saturate_cast<_Tp>(a.x * b), saturate_cast<_Tp>(a.y * b), saturate_cast<_Tp>(a.z * b), saturate_cast<_Tp>(a.w * b) );
}

template<typename _Tp> static inline
Point4_<_Tp> operator * (double a, const Point4_<_Tp>& b)
{
    return Point4_<_Tp>( saturate_cast<_Tp>(b.x * a), saturate_cast<_Tp>(b.y * a), saturate_cast<_Tp>(b.z * a), saturate_cast<_Tp>(b.w * a) );
}

//template<typename _Tp> static inline
//Point4_<_Tp> operator * (const Matx<_Tp, 4, 3>& a, const Point4_<_Tp>& b)
//{
//    Matx<_Tp, 3, 1> tmp = a * Vec<_Tp,3>(b.x, b.y, b.z);
//    return Point4_<_Tp>(tmp.val[0], tmp.val[1], tmp.val[2]);
//}
//
//template<typename _Tp> static inline
//Matx<_Tp, 4, 1> operator * (const Matx<_Tp, 4, 4>& a, const Point4_<_Tp>& b)
//{
//    return a * Matx<_Tp, 4, 1>(b.x, b.y, b.z, 1);
//}

template<typename _Tp> static inline
Point4_<_Tp> operator / (const Point4_<_Tp>& a, int b)
{
    Point4_<_Tp> tmp(a);
    tmp /= b;
    return tmp;
}

template<typename _Tp> static inline
Point4_<_Tp> operator / (const Point4_<_Tp>& a, float b)
{
    Point4_<_Tp> tmp(a);
    tmp /= b;
    return tmp;
}

template<typename _Tp> static inline
Point4_<_Tp> operator / (const Point4_<_Tp>& a, double b)
{
    Point4_<_Tp> tmp(a);
    tmp /= b;
    return tmp;
}

//template<typename _Tp> inline
//const void Point3_<_Tp>::operator = (const Point4_<_Tp> & pt4d)
//{
//  x = pt4d.x / pt4d.w;
//  y = pt4d.y / pt4d.w;
//  z = pt4d.z / pt4d.w;
//}
//  


} /* namespace opencv */

#endif /* _H_MPL_OPENCV_H_ */
