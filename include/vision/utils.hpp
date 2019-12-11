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

#ifndef _H_MPL_VISION_UTILS_H_
#define _H_MPL_VISION_UTILS_H_

#include <cstdlib>
#include <cstdio>

#include <climits>

#include <sstream>

#include <opencv2/opencv.hpp>

#include <mpl/stdio.hpp>
#include <mpl/math.hpp>


#define GET_OMEGA_X(K) K.at<double>(0,0)
#define GET_OMEGA_Y(K) K.at<double>(1,1)

#define GET_OPTICALCENTER_X(K) K.at<double>(1,2)
#define GET_OPTICALCENTER_Y(K) K.at<double>(0,2)

/*****************************************************************************/
// namespace vision
/*****************************************************************************/
namespace mpl::vision {

  /*****************************************************************************/
  // initProjectionMatrix
  /*****************************************************************************/
  void initProjectionMatrix(const char * string, cv::Mat & projectionMatrix){
    
    std::stringstream iss(string);
    
    double value;
    
    uint32_t valueRead = 0;
    
    projectionMatrix.create(3, 4, CV_64F);
    
    while(iss.good()){
      
      iss >> value;
      
      if(valueRead < 12) ((double*)projectionMatrix.data)[valueRead] = value;
      
      ++valueRead;

      if(valueRead == 12) break;
      
    }

    
    if(valueRead < 12){
      fprintf(stderr, "error less value '%s'\n", string);
      abort();
    }
    
    if(valueRead > 12){
      fprintf(stderr, "warming more value\n");
    }
    
    //return projectionMatrix;
    
  }

  /*****************************************************************************/
  // loadProjectionMatrix
  /*****************************************************************************/
  void loadProjectionMatrix(FILE * file, cv::Mat & projectionMatrix, std::size_t line = 0){
    
    std::size_t lineRead = 0;
    
    char str[PATH_MAX];
    
    // ciclo su tutte le linee del file
    while(fgets(str, PATH_MAX, file)){
      
      if(lineRead == line) initProjectionMatrix(str, projectionMatrix);
      
      ++lineRead;
      
    }
    
    if(lineRead < line){
      fprintf(stderr, "error less line\n");
      abort();
    }
    
    if(lineRead > line){
      fprintf(stderr, "warming more lines\n");
    }
    
  }

  /*****************************************************************************/
  // loadProjectionMatrix
  /*****************************************************************************/
  void loadProjectionMatrix(const char * file, cv::Mat & projectionMatrix, std::size_t line = 0){
    
    FILE * input = mpl::io::open(file, "r");
    
    loadProjectionMatrix(input, projectionMatrix, line);
    
    mpl::io::close(input);
    
  }

  /*****************************************************************************/
  // initCameraMatrix
  /*****************************************************************************/
  void initCameraMatrix(const char * string, cv::Mat & cameraMatrix){
    
    std::stringstream iss(string);
    
    double value;
    
    uint32_t valueRead = 0;
    
    cameraMatrix.create(3, 3, CV_64F);
    
    while(iss.good()){
      
      iss >> value;
      
      if(valueRead < 4){
        if(valueRead == 0) ((double*)cameraMatrix.data)[0] = value;
        if(valueRead == 1) ((double*)cameraMatrix.data)[2] = value;
        if(valueRead == 2) ((double*)cameraMatrix.data)[4] = value;
        if(valueRead == 3) ((double*)cameraMatrix.data)[5] = value;
      }
      
      ++valueRead;
      
      if(valueRead == 4) break;
      
    }
    
    if(valueRead < 4){
      fprintf(stderr, "error less value in camera matrix\n");
      abort();
    }
    
    if(valueRead > 4){
      fprintf(stderr, "warming more value in camera matrix\n");
    }
    
    ((double*)cameraMatrix.data)[8] = 1.0;
    
    //return projectionMatrix;
    
  }

  /*****************************************************************************/
  // loadCameraMatrix
  /*****************************************************************************/
  void loadCameraMatrix(FILE * file, cv::Mat & cameraMatrix, std::size_t line = 0){
    
    std::size_t lineRead = 0;
    
    char str[PATH_MAX];
    
    // ciclo su tutte le linee del file
    while(fgets(str, PATH_MAX, file)){
      
      if(lineRead == line)
        initCameraMatrix(str, cameraMatrix);
      
      ++lineRead;
      
    }
    
    if(lineRead < line){
      fprintf(stderr, "error less line in camera matrix\n");
      abort();
    }
    
    if(lineRead > line){
      fprintf(stderr, "warming moer line in camera matrix\n");
    }
    
  }


  /*****************************************************************************/
  // loadCameraMatrix
  /*****************************************************************************/
  void loadCameraMatrix(const char * file, cv::Mat & cameraMatrix, std::size_t line = 0){
    
    FILE * input = mpl::io::open(file, "r");
    
    loadCameraMatrix(input, cameraMatrix, line);
    
    mpl::io::close(input);
    
  }
  
  /*****************************************************************************/
  // getCameraCenter() - Multiple View Geometry p. 156-159
  /*****************************************************************************/
  template <class T>
  void getCameraCenter(const cv::Mat & P, cv::Point3_<T> & center) {

    // FIXME: si puo semplificare
    cv::Mat M  = cv::Mat(3, 3, CV_64F);
    cv::Mat MC = cv::Mat(3, 1, CV_64F);

    for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
         M.at<double>(i,j) = -P.at<double>(i,j);
          
    for(int i=0; i<3; i++)
      MC.at<double>(i) = P.at<double>(i,3);
          
    cv::Mat MInv = M.inv();
          
    cv::Mat C = MInv * MC;

    center.x = C.at<double>(0);
    center.y = C.at<double>(1);
    center.z = C.at<double>(2);

    //std::cout << "P " << P << std::endl;
    //std::cout << "P4 " << P.col(3) << std::endl;
    //std::cout << "MInv " << MInv << std::endl;
    //std::cout << "C " << C << std::endl;

    
  }
  
  /*****************************************************************************/
  // findPlanePassingBy
  /*****************************************************************************/
  template<class T>
  void findPlanePassingBy(const cv::Point3_<T> & c0, const cv::Point3_<T> & c1, const cv::Point3_<T> & c2, double * coef) {
   
    #if(1)
      
      cv::Mat a = cv::Mat(2, 2, CV_64FC1);
      cv::Mat b = cv::Mat(2, 2, CV_64FC1);
      cv::Mat c = cv::Mat(2, 2, CV_64FC1);

      a.at<double>(0,0) = c1.y - c0.y; a.at<double>(0,1) = c1.z - c0.z;
      a.at<double>(1,0) = c2.y - c0.y; a.at<double>(1,1) = c2.z - c0.z;

      b.at<double>(0,0) = c1.x - c0.x; b.at<double>(0,1) = c1.z - c0.z;
      b.at<double>(1,0) = c2.x - c0.x; b.at<double>(1,1) = c2.z - c0.z;
      
      c.at<double>(0,0) = c1.x - c0.x; c.at<double>(0,1) = c1.y - c0.y;
      c.at<double>(1,0) = c2.x - c0.x; c.at<double>(1,1) = c2.y - c0.y;

      coef[0] =  cv::determinant(a);
      coef[1] = -cv::determinant(b);
      coef[2] =  cv::determinant(c);
      coef[3] =  -coef[0]*c0.x - coef[1]*c0.y - coef[2]*c0.z;

    #else
      
      cv::Mat A = centers.inv();

      for(int i=0; i<3; i++)
        coef[i] = -(A.at<double>(i,0) + A.at<double>(i,1) + A.at<double>(i,2));

      coef[3] = 1;
      
    #endif
  
  }
  
  /*****************************************************************************/
  // fundamentalFromProjections
  /*****************************************************************************/
  template<typename TT>
  void fundamentalFromProjections(const cv::Mat_<TT> & P1, const cv::Mat_<TT> & P2, cv::Mat_<TT> F) {
    
    
    double T[4][4] = { 0.0, };

    cv::Mat A; A.create(3, 3, CV_64F);
    
    // copio P2 in U
    P2(cv::Rect(0,0,3,3)).copyTo(A);
    
    cv::Mat B; B.create(3, 1, CV_64F);
    
    B = - P2.col(3);
    
   // std::cout << "B :" << B << std::endl;
    
    cv::Mat U,W,V;
    
    //gsl_linalg_SV_decomp(U,V,S,W);
    //cv::SVDecomp(A, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
   //
    //gsl_linalg_SV_decomp(U,V,S,W);
    
    cv::Mat X;
    
    //gsl_linalg_SV_solve(U,V,S,B,X);
    
    cv::solve(A, B, X);
    
   // std::cout << "X :" << X << std::endl;

    
    for(int i=0; i<3; ++i){
      
      const int j1 = (i + 1) % 3;
      const int j2 = (i + 2) % 3;
      
      const double h[3] = { P2(j1,1) * P2(j2,2) - P2(j1,2) * P2(j2,1),
                            P2(j1,2) * P2(j2,0) - P2(j1,0) * P2(j2,2),
                            P2(j1,0) * P2(j2,1) - P2(j1,1) * P2(j2,0) };
      
      const double z = h[0] * P2(i,0) + h[1] * P2(i,1) + h[2] * P2(i,2);
      
      for(int j=0; j<3; ++j) T[j][i] = h[j] / z;
      
      T[i][3] = X.at<double>(i);
      
    }

    T[3][3] = 1.0;
    
    /*
    printf("T: ");
    
    for(int i=0; i<4; ++i) {
      for(int j=0; j<4; ++j)
        printf("%e ", T[i][j]);
      printf("\n");

    }
    */
    double C[3][3] = { 0.0, };
    
    double t[3];
    
    for(int i=0; i<3; ++i) {
      
      for(int j=0; j<3; ++j) {
        
        C[i][j] = 0.0;
        
        for(int k=0; k<4; ++k) C[i][j] += P1(i,k)*T[k][j];
        
      }
      
      t[i] = 0.0;
      
      for(int k=0; k<4; ++k) t[i] += P1(i,k)*T[k][3];
      
    }
    
    
    
    double tx[3][3] = { {0.0, -t[2], t[1]}, {t[2], 0.0, -t[0]}, {-t[1], t[0], 0.0} };
    
    for(int i=0; i<3; ++i){
      
      for(int j=0; j<3; ++j){
        F(i,j) = 0.0;
        for(int k=0; k<3; ++k) F(i,j) += tx[i][k]*C[k][j];
      }
      
    }

    
    
    
    /*
    cv::Mat C;
    
    findProjectionsCenter(P1, C);
    
    std::cout << "C " << C << std::endl;
    
    cv::Mat e = P2 * C.t();
        
    cv::Mat ex; ex.create(3, 3, CV_64F);
    
    ex.at<double>(0,0) =             0.0; ex.at<double>(0,1)  = -C.at<double>(2); ex.at<double>(0,2) =  C.at<double>(1);
    ex.at<double>(1,0) =  C.at<double>(2); ex.at<double>(1,1) =              0.0; ex.at<double>(1,2) = -C.at<double>(0);
    ex.at<double>(2,0) = -C.at<double>(1); ex.at<double>(2,1) =  C.at<double>(0); ex.at<double>(2,2) =              0.0;
    
    F = e * P2 * P1.inv();
    */
    
    /*
    cv::Mat_<T> X[3];
    
    cv::vconcat(P1.row(1), P1.row(2), X[0]);
    cv::vconcat(P1.row(2), P1.row(0), X[1]);
    cv::vconcat(P1.row(0), P1.row(1), X[2]);
    
    cv::Mat_<T> Y[3];
    
    cv::vconcat(P2.row(1), P2.row(2), Y[0]);
    cv::vconcat(P2.row(2), P2.row(0), Y[1]);
    cv::vconcat(P2.row(0), P2.row(1), Y[2]);

   // for(int i = 0; i < 3; ++i)  std::cout << "X " << X[i] << std::endl;
    
   // for(int i = 0; i < 3; ++i) std::cout << "Y " << Y[i] << std::endl;

    cv::Mat_<T> XY;
    
    for(int i = 0; i < 3; ++i){
      for(int j = 0; j < 3; ++j){
        cv::vconcat(X[j], Y[i], XY);
       // std::cout << "XY " << XY << std::endl;
       // printf("determinant %e : ", cv::determinant(XY));
        F(i, j) = cv::determinant(XY);
       // printf("%e\n", F(i, j));
      }
    }
    
  //  printf("\n");
     
     */

  }
  
  /*****************************************************************************/
  // fundamentalFromProjections
  /*****************************************************************************/
  void fundamentalFromProjections(cv::InputArray _P1, cv::InputArray _P2, cv::OutputArray _F) {
    
    const cv::Mat P1 = _P1.getMat(), P2 = _P2.getMat();
    
    const int depth = P1.depth();
    
    CV_Assert((P1.cols == 4 && P1.rows == 3) && P1.rows == P2.rows && P1.cols == P2.cols);
    CV_Assert((depth == CV_32F || depth == CV_64F) && depth == P2.depth());
    
    _F.create(3, 3, depth);
    
    //cv::Mat F = _F.getMat();
    
    if(depth == CV_32F) fundamentalFromProjections<float> (P1, P2, _F.getMat());
    else                fundamentalFromProjections<double>(P1, P2, _F.getMat());
    
  }

  
  /*****************************************************************************/
  // epipolarLine
  /*****************************************************************************/
  template <class T2D>
  inline void epipolarLine(const T2D & pt, cv::Vec3f & line, const cv::Mat & funMat) {
    
    //TODO: check matrix
    
    double * matFun = ((double*)funMat.data);
    
    line[0] = pt.x * matFun[0] + pt.y * matFun[1] + matFun[2];
    line[1] = pt.x * matFun[3] + pt.y * matFun[4] + matFun[5];
    line[2] = pt.x * matFun[6] + pt.y * matFun[7] + matFun[8];
    
  }
  
  /*****************************************************************************/
  // epipolarLine
  /*****************************************************************************/
  template <class T2D>
  inline cv::Vec3f epipolarLine(const T2D & pt, const cv::Mat & funMat) {
    
    cv::Vec3f line;
    
    epipolarLine(pt, line, funMat);
    
    return line;
    
    
  }

  /*****************************************************************************/
  // alignment
  /*****************************************************************************/
  template <class T>
  double alignment(const cv::Point_<T> & p1, const cv::Point_<T> & p2, const cv::Point_<T> & p3) {

    cv::Point_<T> p12 = p1 - p2;
    cv::Point_<T> p13 = p1 - p3;
    cv::Point_<T> p23 = p2 - p3;

    std::vector<double> a(3);

    a[0] = std::abs((p12.x*p13.y - p13.x*p12.y) / (cv::norm(p12) * cv::norm(p13)));
    a[1] = std::abs((p12.x*p23.y - p23.x*p12.y) / (cv::norm(p12) * cv::norm(p23)));
    a[2] = std::abs((p13.x*p23.y - p23.x*p13.y) / (cv::norm(p23) * cv::norm(p13)));

    return *std::min_element(a.begin(),a.end());

  }

  /*****************************************************************************/
  // alignment
  /*****************************************************************************/
  template <class T>
  double alignment(const std::vector<cv::Point_<T>> & points) {

    if(points.size() != 6) {
      fprintf(stderr, "mpl::isAlignment() error - input points size must be 6\n");
      abort();
    }

    std::vector<double> alignments(4);

    alignments[0] = mpl::vision::alignment(points[2], points[3], points[4]);
    alignments[1] = mpl::vision::alignment(points[2], points[3], points[5]);
    alignments[2] = mpl::vision::alignment(points[2], points[4], points[5]);
    alignments[3] = mpl::vision::alignment(points[3], points[4], points[5]);

    return *std::min_element(alignments.begin(),alignments.end());

  }
  
  /*****************************************************************************/
  // isOnConic
  /*****************************************************************************/
  template <class T>
  double isOnConic(const std::vector<cv::Point_<T>> & points) {
    
    if(points.size() != 6) {
      fprintf(stderr, "mpl::isOnConic() error - input points size must be 6\n");
      abort();
    }
    
    mpl::Mat A(6,6);
    
    for(int i=0; i<6; ++i) {
      A(i,0) = points[i].x * points[i].x;
      A(i,1) = points[i].x * points[i].y;
      A(i,2) = points[i].y * points[i].y;
      A(i,3) = points[i].x;
      A(i,4) = points[i].y;
      A(i,5) = 1.0;
    }
    
    cv::Mat AA = A.t() * A;
    
    cv::Mat W,U,V;
    
    cv::SVDecomp(AA, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    
    return W.at<double>(5);
    
  }

  template <class T>
  inline void normalize(cv::Point3_<T> & point) {

    point.x /= point.z;
    point.y /= point.z;
    point.z  = 1.0;

  }

  template <class T>
  inline void normalize(const cv::Point3_<T> & point, cv::Point3_<T> & pointNorm) {

    pointNorm.x = point.x / point.z;
    pointNorm.y = point.y / point.z;
    pointNorm.z = 1.0;

  }

  template <class T>
  inline void normalize(const cv::Point3_<T> & point, cv::Point_<T> & pointNorm) {

    pointNorm.x = point.x / point.z;
    pointNorm.y = point.y / point.z;

  }

  /*****************************************************************************/
  // fromProjectionMatricesToFondamentalMatrix
  /*****************************************************************************/
  // See Multiple View Geometry in Computer Vision p. 246 and 254
  void fromProjectionMatricesToFondamentalMatrix(const cv::Mat & Pl, const cv::Mat & Pr, cv::Mat & F) {
    
    cv::Mat H = cv::Mat::zeros(cv::Size(4,4), CV_64FC1);
    H.at<double>(3,0) = 1;
    H.at<double>(3,1) = 1;
    H.at<double>(3,2) = 1;
    H.at<double>(3,3) = 1;
    
    cv::Mat R = cv::Mat::zeros(cv::Size(3,3), CV_64FC1);
    //  Pl(cv::Rect(0,0,3,3)).copyTo(R);
    
    for(size_t i=0; i<3; ++i){
      for(size_t j=0; j<3; ++j){
        
        R.at<double>(i,j) = Pl.at<double>(i,j);
        
      }
    }
    
    // std::cout << "R " << R << std::endl;
    cv::Mat RInv = R.inv();
    
    cv::Mat T = cv::Mat::zeros(cv::Size(1,3), CV_64FC1);
    T.at<double>(0,0) = -Pl.at<double>(0,3);
    T.at<double>(1,0) = -Pl.at<double>(1,3);
    T.at<double>(2,0) = -Pl.at<double>(2,3);
    
    //T.col(0) = -Pl.col(3);
    
    cv::Mat h = cv::Mat::zeros(cv::Size(1,3), CV_64FC1);
    
    //std::cout << "T " << T << std::endl;
    for(size_t i=0; i<3; ++i){
      
      cv::Mat T_tmp = cv::Mat::zeros(cv::Size(1,3), CV_64FC1);
      //T.copyTo(T_tmp(cv::Rect(0,0,1,3)));
      T_tmp.at<double>(0,0) = T.at<double>(0,0);
      T_tmp.at<double>(1,0) = T.at<double>(1,0);
      T_tmp.at<double>(2,0) = T.at<double>(2,0);
      
      T_tmp.at<double>(i,0) += 1;
      
      h = RInv * T_tmp;
      H.at<double>(0,i) = h.at<double>(0,0);
      H.at<double>(1,i) = h.at<double>(1,0);
      H.at<double>(2,i) = h.at<double>(2,0);
      
    }
    
    
    h = RInv * T;
    H.at<double>(0,3) = h.at<double>(0,0);
    H.at<double>(1,3) = h.at<double>(1,0);
    H.at<double>(2,3) = h.at<double>(2,0);
    
    
    //std::cout << "H " << H << std::endl;
    //std::cout << "PH " << Pl*H << std::endl;
    
    //printf("fine di fromPtoCan\n"); fflush(stdout);
    
    /*
     cv::Mat Plp = cv::Mat::zeros(cv::Size(4, 4), CV_64FC1);
     
     Plp.at<double>(3,3) = 1;
     
     Pl.copyTo(Plp(cv::Rect(0,0,4,3)));
     
     std::cout << "Pl " << Pl << std::endl;
     
     std::cout << "Plp " << Plp << std::endl;
     
     cv::Mat H = Plp.inv(); //Plp.inv(cv::DECOMP_SVD);
     
     std::cout << "H " << H << std::endl;
     */
    
    cv::Mat Prl = Pr * H;
    
    cv::Mat M = Prl(cv::Rect(0,0,3,3));
    
    mpl::Mat m(3,3);
    
    m(0,1) = -Prl.at<double>(2,3); m(0,2) =  Prl.at<double>(1,3);
    m(1,0) =  Prl.at<double>(2,3); m(1,2) = -Prl.at<double>(0,3);
    m(2,0) = -Prl.at<double>(1,3); m(2,1) =  Prl.at<double>(0,3);
    
    //std::cout << "Prl " << Prl << std::endl;
    //std::cout << "M " << M << std::endl;
    //std::cout << "m " << m << std::endl;
    //cv::Mat(M * m).copyTo(F);
    F = m * M;
    //std::cout << "F " << F << std::endl;
    
    
  }
  
  /*****************************************************************************/
  // epipolarLines
  /*****************************************************************************/
  // Retta ax + by + c = 0
  cv::Vec3d epipolarLine(const cv::Point2d & point, cv::Mat F) {
    
    cv::Vec3d lineParameters;
    
    //std::cout << F << std::endl;
    
    lineParameters[0] = point.x * F.at<double>(0,0) + point.y * F.at<double>(0,1) + 1.0 * F.at<double>(0,2);
    lineParameters[1] = point.x * F.at<double>(1,0) + point.y * F.at<double>(1,1) + 1.0 * F.at<double>(1,2);
    lineParameters[2] = point.x * F.at<double>(2,0) + point.y * F.at<double>(2,1) + 1.0 * F.at<double>(2,2);
    
    //printf("%f %f %f\n", lineParameters[0], lineParameters[1], lineParameters[2]);
    
    return lineParameters;
    
  }

  
  
} /* namespace mpl::vision */

#endif /* vision_h */
