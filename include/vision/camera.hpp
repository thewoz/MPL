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


#ifndef _H_COBBS_VISION_CAMERA_H_
#define _H_COBBS_VISION_CAMERA_H_

#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include <mpl/vision/vision.hpp>

/*****************************************************************************/
// namespace vision
/*****************************************************************************/
namespace vision {
  
  /*****************************************************************************/
  // camera_t
  /*****************************************************************************/
  class camera_t {
    
  public:
    
    // internal parameter of the camera
    cv::Mat K;
    
    // projection matrix of the camera
    cv::Mat P;
    
    // rotation angles
    cv::Vec3d R;
    
     // traslation vector
    cv::Vec3d T;
    
    // distortion coefficents
    std::vector<double> distCoeffs;

    // sensor size (in pixel unit)
    cv::Size sensor;
    
    // sensor pixel size (in meter)
    double pixelSize;

    // lens focal lenght (in meter)
    cv::Point2d focalLenght;

    // openGL near and far clipping planes
    double farClippingPlan;
    double nearClippingPlan;
    
    // openGL model view matrix
    cv::Mat glModelViewMatrix;
    
    // openGL projection matrix
    cv::Mat glProjectionMatrix;
    
    //****************************************************************************//
    // update
    //****************************************************************************//
    inline void update() {
    
      cv::Mat rotationalMatrix;
      cv::Mat traslationVector;

      cv::decomposeProjectionMatrix(P, K, rotationalMatrix, traslationVector);

      T[0] = traslationVector.at<double>(0);
      T[1] = traslationVector.at<double>(1);
      T[2] = traslationVector.at<double>(2);

      cv::Rodrigues(rotationalMatrix, R);

      double * _R = (double *) rotationalMatrix.data;
      double * _T = (double *) &T[0];
      
      // update GL model view matrix
      float * _MM = (float *) glModelViewMatrix.data;

      _MM[0]  = _R[0];   _MM[1]  = _R[3];   _MM[2]  = _R[6];   _MM[3]  = 0.0f;
      _MM[4]  = _R[1];   _MM[5]  = _R[4];   _MM[6]  = _R[7];   _MM[7]  = 0.0f;
      _MM[8]  = _R[2];   _MM[9]  = _R[5];   _MM[10] = _R[8];   _MM[11] = 0.0f;
      _MM[12] = _T[0];   _MM[13] = _T[1];   _MM[14] = _T[2];   _MM[15] = 1.0f;
      
      // update GL projection matrix
      double width  = sensor.width  * pixelSize; /* lunghezza sensore in metri */
      double height = sensor.height * pixelSize; /* altezza sensore in metri */
      
      float n2n  = 2.0 * nearClippingPlan;
      float n2nf = n2n * farClippingPlan;
      float nmf  = nearClippingPlan - farClippingPlan;
      float npf  = nearClippingPlan + farClippingPlan;

      float * _MP = (float *) glProjectionMatrix.data;

      _MP[0]  = n2n/width;    _MP[1]  = 0.0f;           _MP[2]  = 0.0f;       _MP[3]  = 0.0f;
      _MP[4]  = 0.0f;         _MP[5]  = n2nf/height;    _MP[6]  = 0.0f;       _MP[7]  = 0.0f;
      _MP[8]  = 0.0f;         _MP[9]  = 0.0f;           _MP[10] = npf/nmf;    _MP[11] = n2nf/nmf;
      _MP[12] = 0.0f;         _MP[13] = 0.0f;           _MP[14] = -1.0f;      _MP[15] = 0.0f;
      
      // update projection matrix
//      double * _P = (double *) P.data;
//      double * _K = (double *) K.data;
//      
//      _P[0] = _K[0]*_R[0] + _K[2]*_R[6];   _P[1] = _K[0]*_R[1] + _K[2]*_R[7];   _P[2]  = _K[0]*_R[2] + _K[2]*_R[8];   _P[3]  = _K[0]*_T[0] + _K[2]*_T[2];
//      _P[4] = _K[4]*_R[3] + _K[5]*_R[6];   _P[5] = _K[4]*_R[4] + _K[5]*_R[7];   _P[6]  = _K[4]*_R[5] + _K[5]*_R[8];   _P[7]  = _K[4]*_T[1] + _K[5]*_T[2];
//      _P[8] = _R[6];                       _P[9] = _R[7];                       _P[10] = _R[8];                       _P[11] = _T[2];
//
//      std::cout << "P" << P << std::endl;
//      std::cout << "K" << K << std::endl;

      
    }
      
  public:
    
    //****************************************************************************//
    // camera_t
    //****************************************************************************//
    camera_t() {
    
      K.create(3, 3, CV_64F);
      
      K.at<double>(2,2) = 1;
    
      P.create(3, 4, CV_64F);
      
      distCoeffs.resize(8);
      
      pixelSize = 1.0;
      
      glModelViewMatrix.create(4, 4, CV_32F);
      
      glProjectionMatrix.create(4, 4, CV_32F);
      
    }
    
    
    //****************************************************************************//
    // set/get focal lenght
    //****************************************************************************//
    inline void setFocalLenght(double value) {
      
      ((double*)K.data)[0] = ((double*)K.data)[4] = value / pixelSize;
      
      focalLenght.x = focalLenght.y = value;
      
      update();
    
    }
    
    inline void setFocalLenght(double valueX, double valueY) {
      
      ((double*)K.data)[0] = valueX / pixelSize;
      ((double*)K.data)[4] = valueY / pixelSize;
      
      focalLenght.x = valueX; focalLenght.y = valueY;
    
      update();
      
    }

    template<typename type> inline type getFocalLenght() const {
      
      static_assert(std::is_same<type, cv::Point_<float>   >::value ||
                    std::is_same<type, cv::Point_<double>  >::value ||
                    std::is_same<type, cv::Vec<float,2>    >::value ||
                    std::is_same<type, cv::Vec<double,2>   >::value, "use a undefine specialization");
      
      // NOTE: from C++17 change in
      // if constexpr(true) ;
      
      return type(focalLenght.x, focalLenght.y);
      
    }
    
    template <typename type> inline void getFocalLenght(type & valueX, type & valueY) const { valueX = focalLenght.x; valueY = focalLenght.y; }
    
    
    //****************************************************************************//
    // set/get omega
    //****************************************************************************//
    inline void setOmega(double value) {
      
      ((double*)K.data)[0] = ((double*)K.data)[4] = value;
      
      focalLenght.x = focalLenght.y = value * pixelSize;
      
      update();
      
    }
    
    inline void setOmega(double valueX, double valueY) {
      
      ((double*)K.data)[0] = valueX;
      ((double*)K.data)[4] = valueY;
      
      focalLenght.x = valueX * pixelSize; focalLenght.y = valueY * pixelSize;
      
      update();
      
    }
    
    template<typename type> inline type getOmega() const {
      
      static_assert(std::is_same<type, cv::Point_<float>   >::value ||
                    std::is_same<type, cv::Point_<double>  >::value ||
                    std::is_same<type, cv::Vec<float,2>    >::value ||
                    std::is_same<type, cv::Vec<double,2>   >::value, "use a undefine specialization");
      
      // NOTE: from C++17 change in
      // if constexpr(true) ;
      
      return type(((double*)K.data)[0], ((double*)K.data)[4]);
      
    }
    
    template <typename type> inline void getOmega(type & valueX, type & valueY) const { valueX = ((double*)K.data)[0]; valueY = ((double*)K.data)[4]; }
    
    //****************************************************************************//
    // set/get sensor (size and pixelsize)
    //****************************************************************************//
    inline void setSensor(int width, int height, double pixelsize, bool initOpticalCenter = true){
      
      sensor.width  = width;
      sensor.height = height;

      pixelSize = pixelsize;
      
      // aggiorno la focal lenght in K che deve essere espressa in pixel unit
      ((double*)K.data)[0] = focalLenght.x / pixelSize;
      ((double*)K.data)[4] = focalLenght.y / pixelSize;
      
      if(initOpticalCenter){
        ((double*)K.data)[2] = width  * 0.5;
        ((double*)K.data)[5] = height * 0.5;
      }
      
      update();
      
    }
    
                             inline cv::Size getSensorSize()                            const { return sensor; }
                             inline void     getSensorSize(cv::Size size)               const { size = sensor; }
    template <typename type> inline void     getSensorSize(type & width, type & height) const { width  = sensor.width; height = sensor.height; }
    

    inline double getPixelSize()                   const { return pixelSize;       }
    inline void   getPixelSize(double & pixelsize) const { pixelsize = pixelSize; }
    inline void   getPixelSize(float  & pixelsize) const { pixelsize = pixelSize; }

    template <typename T1, typename T2> inline void getSensor(T1 & width, T1 & height, T2 & pixelsize) const { getSensorSize(width, height); getPixelSize(pixelsize); }
    template <typename type>            inline void getSensor(cv::Size & size, type & pixelsize)       const { getSensorSize(size); getPixelSize(pixelsize); }
    
    
    /*****************************************************************************/
    // set/get camera optical center
    /*****************************************************************************/
    inline void setOpticalCenter(double cx, double cy) { ((double*)K.data)[2] = cx; ((double*)K.data)[5] = cy; update(); }

    template<typename type> inline type getOpticalCenter() const {
      
      static_assert(std::is_same<type, cv::Point_<float>   >::value ||
                    std::is_same<type, cv::Point_<double>  >::value ||
                    std::is_same<type, cv::Vec<float,2>    >::value ||
                    std::is_same<type, cv::Vec<double,2>   >::value, "use a undefine specialization");
      
      // NOTE: from C++17 change in
      // if constexpr(true) ;
      
      return type(((double*)K.data)[2], ((double*)K.data)[5]);
      
    }
    
    template <typename type> inline void getOpticalCenter(type & valueX, type & valueY) const { valueX =  ((double*)K.data)[2]; valueY =  ((double*)K.data)[5]; }

    
    //****************************************************************************//
    // set/get distortion coefficients
    //****************************************************************************//
    inline void setDistortionCoefficients(double k1, double k2, double p1, double p2, double k3 = 0.0, double k4 = 0.0, double k5 = 0.0, double k6 = 0.0){
      distCoeffs[0] = k1; distCoeffs[1] = k2; distCoeffs[2] = p1; distCoeffs[3] = p2;
      distCoeffs[4] = k3; distCoeffs[5] = k4; distCoeffs[6] = k5; distCoeffs[7] = k6;
    }
    
    inline const std::vector<double> & getDistortionCoefficients() const { return distCoeffs; }
    inline void getDistortionCoefficients(std::vector<double> & arg) const { arg = distCoeffs; }

    
    //****************************************************************************//
    // set/update/get translation vector
    //****************************************************************************//
    inline void setTranslation(double x, double y, double z)  {                         T[0] = x;      T[1] = y;      T[2] = z;      update(); }
    template<typename type> inline void setTranslation(const cv::Point3_<type> & arg) { T[0] = arg.x;  T[1] = arg.y;  T[2] = arg.z;  update(); }
    template<typename type> inline void setTranslation(const cv::Vec<type,3>   & arg) { T[0] = arg[0]; T[1] = arg[1]; T[2] = arg[2]; update(); }

                            inline void updateTranslation(double x, double y, double z)  { T[0] += x;      T[1] += y;      T[2] += z;      update(); }
    template<typename type> inline void updateTranslation(const cv::Point3_<type> & arg) { T[0] += arg.x;  T[1] += arg.y;  T[2] += arg.z;  update(); }
    template<typename type> inline void updateTranslation(const cv::Vec<type,3>   & arg) { T[0] += arg[0]; T[1] += arg[1]; T[2] += arg[2]; update(); }
    
    inline void getTranslation(double & x, double & y, double & z) const { x = T[0]; y = T[1]; z = T[2]; }
    inline void getTranslation(float  & x, float  & y, float  & z) const { x = T[0]; y = T[1]; z = T[2]; }

    template<typename type> inline void getTranslation(cv::Point3_<type> & arg) const { arg.x  = T[0]; arg.y  = T[1]; arg.z  = T[2]; }
    template<typename type> inline void getTranslation(cv::Vec<type,3>   & arg) const { arg[0] = T[0]; arg[1] = T[1]; arg[2] = T[2]; }

    template<typename type> inline type getTranslation() const {
      
      static_assert(std::is_same<type, cv::Point3_<float>  >::value ||
                    std::is_same<type, cv::Point3_<double> >::value ||
                    std::is_same<type, cv::Vec<float,3>    >::value ||
                    std::is_same<type, cv::Vec<double,3>   >::value, "use a undefine specialization");
      
      // NOTE: from C++17 change in
      // if constexpr(true) ;
     
      return type(T[0], T[1], T[2]);

    }
    
    
    //****************************************************************************//
    // set/update/get rotational angle
    //****************************************************************************//
                            inline void setRotationAngle(double x, double y, double z)  { R[0] = x;      R[1] = y;      R[2] = z;      update(); }
    template<typename type> inline void setRotationAngle(const cv::Point3_<type> & arg) { R[0] = arg.x;  R[1] = arg.y;  R[2] = arg.z;  update(); }
    template<typename type> inline void setRotationAngle(const cv::Vec<type,3>   & arg) { R[0] = arg[0]; R[1] = arg[1]; R[2] = arg[2]; update(); }

                            inline void updateRotationAngle(double x, double y, double z)  { R[0] += x;      R[1] += y;      R[2] += z;      update(); }
    template<typename type> inline void updateRotationAngle(const cv::Point3_<type> & arg) { R[0] += arg.x;  R[1] += arg.y;  R[2] += arg.z;  update(); }
    template<typename type> inline void updateRotationAngle(const cv::Vec<type,3>   & arg) { R[0] += arg[0]; R[1] += arg[1]; R[2] += arg[2]; update(); }
    
    inline void getRotation(double & x, double & y, double & z) const { x = R[0]; y = R[1]; z = R[2]; }
    inline void getRotation(float  & x, float  & y, float  & z) const { x = R[0]; y = R[1]; z = R[2]; }
    
    template<typename type> inline void getRotationAngle(cv::Point3_<type> & arg) const { arg.x  = R[0]; arg.y  = R[1]; arg.z  = R[2]; }
    template<typename type> inline void getRotationAngle(cv::Vec<type,3>   & arg) const { arg[0] = R[0]; arg[1] = R[1]; arg[2] = R[2]; }
    
    template<typename type> inline type getRotationAngle() const {
      
      static_assert(std::is_same<type, cv::Point3_<float>  >::value ||
                    std::is_same<type, cv::Point3_<double> >::value ||
                    std::is_same<type, cv::Vec<float,3>    >::value ||
                    std::is_same<type, cv::Vec<double,3>   >::value, "use a undefine specialization");
      
      // NOTE: from C++17 change in
      // if constexpr(true) ;
      
      return type(R[0], R[1], R[2]);
      
    }
    
    
    //****************************************************************************//
    // set/update camera position
    //****************************************************************************//
    inline void setPosition   (double x, double y, double z, double ro, double pi, double ya) { updateTranslation(x, y, z); updateRotationAngle(ro, pi, ya); update(); }
    inline void updatePosition(double x, double y, double z, double ro, double pi, double ya) { updateTranslation(x, y, z); updateRotationAngle(ro, pi, ya); update(); }
    
    template<typename T1, typename T2> inline void setPosition   (const T1 & argT, const T2 & argR) { updateTranslation(argT); updateRotationAngle(argR); update(); }
    template<typename T1, typename T2> inline void updatePosition(const T1 & argT, const T2 & argR) { updateTranslation(argT); updateRotationAngle(argR); update(); }

    
    //****************************************************************************//
    // set/get camera
    //****************************************************************************//
    inline void setCameraMatrix(const cv::Mat & matrix, int width, int height, double pixelsize) {
      
      //TODO:
      //assert();
      
      if(((double*)matrix.data)[0] != ((double*)matrix.data)[4]) {
        fprintf(stderr, "");
        abort();
      }
      
      K = matrix;

      pixelSize     = pixelsize;

      // che salvo in metri
      focalLenght.x   = ((double*)K.data)[0] / pixelSize;
      focalLenght.y   = ((double*)K.data)[4] / pixelSize;

      sensor.width  = width;
      sensor.height = height;
  
      update();
      
    }
    
    inline const cv::Mat getCameraMatrix()                 const { return   K.clone(); }
    inline void          getCameraMatrix(cv::Mat & matrix) const { matrix = K.clone(); }
    
    
    //****************************************************************************//
    // set/get projection matrix
    //****************************************************************************//
    inline void setProjectionMatrix(const cv::Mat & matrix, int width, int height, double pixelsize){
      
      // TODO:
      // assert

      P = matrix;
      
      cv::Mat rotMat;
      
      cv::decomposeProjectionMatrix(P, K, rotMat, T);

      setCameraMatrix(K, width, height, pixelSize);
      
      cv::Rodrigues(rotMat, R);
      
      update();
      
    }
    
    inline const cv::Mat getProjectionalMatrix()                 const { return   P.clone(); }
    inline void          getProjectionalMatrix(cv::Mat & matrix) const { matrix = P.clone(); }

    
    /*****************************************************************************/
    // set/get rotational matrix
    /*****************************************************************************/
    inline void setRotationalMatrix(const cv::Mat & matrix){
      
      //TODO:
      //assert and if rotational matrix

      cv::Rodrigues(matrix, R);
    
      update();
      
    }

    inline const cv::Mat getRotationalMatrix()                 const { cv::Mat matrix; cv::Rodrigues(R, matrix); return matrix; }
    inline void          getRotationalMatrix(cv::Mat & matrix) const { cv::Rodrigues(R, matrix); }
    
    
    //****************************************************************************//
    // get openGL projection matrix
    //****************************************************************************//
    inline void          getGlProjection(cv::Mat & matrix)  const { matrix =           glProjectionMatrix.clone(); }
    inline void          getGlProjection(const float * ptr) const { ptr    = (float *) glProjectionMatrix.data;    }
    inline const float * getGlProjection()                  const { return   (float *) glProjectionMatrix.data;    }

    
    //****************************************************************************//
    // get openGL modelView matrix
    //****************************************************************************//
    inline void          getGlModelView(cv::Mat & matrix)  const { matrix =           glModelViewMatrix.clone(); }
    inline void          getGlModelView(const float * ptr) const { ptr    = (float *) glModelViewMatrix.data;    }
    inline const float * getGlModelView()                  const { return   (float *) glModelViewMatrix.data;    }
    
    
    /*****************************************************************************/
    // initCamera
    /*****************************************************************************/
    void initCamera(const char * string) {
      
      std::stringstream iss(string);
      
      double value;
      
      uint32_t valueRead = 0;
      
      K.create(3, 3, CV_64F);
      
      while(iss.good()){
        
        iss >> value;
        
        if(valueRead < 4) {
          if(valueRead == 0) ((double*)K.data)[0] = value;
          if(valueRead == 1) ((double*)K.data)[2] = value;
          if(valueRead == 2) ((double*)K.data)[4] = value;
          if(valueRead == 3) ((double*)K.data)[5] = value;
        } else if(valueRead < 7){
          if(valueRead == 4) sensor.width  = value;
          if(valueRead == 5) sensor.height = value;
          if(valueRead == 6) pixelSize     = value;
        } else distCoeffs.push_back(value);
        
        ++valueRead;
        
      }
      
      if(valueRead < 11){
        fprintf(stderr, "error less parameter in init camera matrix\n");
        abort();
      }
   
      update();
      
    }

    
    /*****************************************************************************/
    // loadCamera
    /*****************************************************************************/
    void loadCamera(FILE * file, uint32_t line = 0) {
      
      std::size_t lineRead = 0;
      
      char str[PATH_MAX];
      
      // ciclo su tutte le linee del file
      while(fgets(str, PATH_MAX, file)){
        
        if(lineRead == line)
          initCamera(str);
        
        ++lineRead;
        
      }
      
      if(lineRead < line){
        fprintf(stderr, "error less line in single camera matrix \n");
        abort();
      }
      
      if(lineRead > line){
        fprintf(stderr, "warming more line in single camera matrix \n");
      }

    }
    
    /*****************************************************************************/
    // loadCamera
    /*****************************************************************************/
    void loadCamera(const char * string, uint32_t line = 0) {
      
      FILE * input = mpl::io::open(string, "r");
      
      loadCamera(input, line);
      
      mpl::io::close(input);
      
    }
    
    /*****************************************************************************/
    // loadProjectionMatrix
    /*****************************************************************************/
    inline void initProjectionMatrix(const char * string){ vision::initProjectionMatrix(string, P); update(); }
    inline void loadProjectionMatrix(FILE * file, std::size_t line = 0){ vision::loadProjectionMatrix(file, P, line); }
    inline void loadProjectionMatrix(const char * file, std::size_t line = 0){ vision::loadProjectionMatrix(file, P, line); }
    
    /*****************************************************************************/
    // loadCameraMatrix
    /*****************************************************************************/
    inline void initCameraMatrix(const char * string){ vision::initCameraMatrix(string, K); update(); }
    inline void loadCameraMatrix(FILE * file, std::size_t line = 0){ vision::loadCameraMatrix(file, K, line); }
    inline void loadCameraMatrix(const char * file, std::size_t line = 0){ vision::loadCameraMatrix(file, K, line); }


  }; /* class camera_t */
  
  
} /* namespace vision */



#endif /* _H_COBBS_VISION_CAMERA_H_ */

