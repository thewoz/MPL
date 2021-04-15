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

#ifndef _H_MPL_VISION_UTILS_H_
#define _H_MPL_VISION_UTILS_H_

#include <cstdlib>
#include <cstdio>

#include <climits>

#include <sstream>

#include <opencv2/opencv.hpp>

#include <mpl/stdio.hpp>
#include <mpl/math.hpp>

#include <mpl/vision/normalization.hpp>
#include <mpl/vision/reconstruction.hpp>

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
            
    cv::Mat X;
        
    cv::solve(A, B, X);
        
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

  }
  
  //****************************************************************************
  // fundamentalFromProjections
  //****************************************************************************
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

  //***************************************************************************************
  // utilsOptimalTriangulation
  //***************************************************************************************
  namespace utilsOptimalTriangulation {
    
    double cost(double a, double b, double c, double d, double f, double s, double t) {
      
      return (t*t/(1+f*f*t*t))+((c*t+d)*(c*t+d)/((a*t+b)*(a*t+b)+s*s*(c*t+d)*(c*t+d)));
      
    }
    
    cv::Vec3d closestPoint(double lambda, double mu, double nu) {
      
      cv::Vec3d point;
      
      point[0] = -lambda * nu;
      point[1] = -mu * nu;
      point[2] = lambda*lambda + mu*mu;
      
      return point;
      
    }
    
  }
  
  //***************************************************************************************
  // optimalTriangulation
  //***************************************************************************************
  // Multiple View Geometry in Computer Vision
  // Pag 318 Alg 12.1
  template<typename P2D, typename P3D>
  void optimalTriangulation(const P2D & point1, const P2D & point2, cv::Mat & fundamentalMatrix, P3D & point3D = NULL) {
    
    mpl::Mat3 T1; mpl::Mat3 T2;
    
    T1(0,0) = 1; T1(1,1) = 1; T1(2,2) = 1;
    T2(0,0) = 1; T2(1,1) = 1; T2(2,2) = 1;
    
    T1(0,2) = -point1.x; T1(1,2) = -point1.y;
    T2(0,2) = -point2.x; T2(1,2) = -point2.y;
    
    cv::Mat F = T2.inv().t() * fundamentalMatrix * T1.inv();
        
    cv::Mat W,U; mpl::Mat V;
    mpl::math::svd(F, W, U, V, cv::SVD::FULL_UV);
    
    //std::cout << "W: \n"  << W << "\n\n";
    //std::cout << "U: \n"  << U << "\n\n";
    //std::cout << "V: \n"  << V << "\n\n";
    
    cv::Mat E1; cv::Mat E2;
    E1 = V.row(2);
    E2 = U.col(2).t();
    
    double normE1 = sqrt(E1.at<double>(0) * E1.at<double>(0) + E1.at<double>(1) * E1.at<double>(1));
    double normE2 = sqrt(E2.at<double>(0) * E2.at<double>(0) + E2.at<double>(1) * E2.at<double>(1));
    
    E1.at<double>(0) /= normE1; E1.at<double>(1) /= normE1; E1.at<double>(2) /= normE1;
    E2.at<double>(0) /= normE2; E2.at<double>(1) /= normE2; E2.at<double>(2) /= normE2;
    
    mpl::Mat3 R1; mpl::Mat3 R2;
    
    R1(0,0) =  E1.at<double>(0); R1(0,1) = E1.at<double>(1);
    R1(1,0) = -E1.at<double>(1); R1(1,1) = E1.at<double>(0);
    R1(2,2) = 1;
    
    R2(0,0) =  E2.at<double>(0); R2(0,1) = E2.at<double>(1);
    R2(1,0) = -E2.at<double>(1); R2(1,1) = E2.at<double>(0);
    R2(2,2) = 1;
    
    F = R2 * F * R1.t();
        
    {
      double a = F.at<double>(1,1);
      double b = F.at<double>(1,2);
      double c = F.at<double>(2,1);
      double d = F.at<double>(2,2);
      double f = E1.at<double>(2);
      double s = E2.at<double>(2);
      
      double A = a*a+s*s*c*c;
      double B = 2*(a*b+s*s*c*d);
      double C = b*b+s*s*d*d;
      double D = a*d+b*c;
      double E = b*d;
      double F = a*c;
      double G = a*d-b*c;
      
      double Ap = A*A;
      double Bp = 2*A*B;
      double Cp = B*B+2*A*C;
      double Dp = 2*B*C;
      double Ep = C*C;
      
      double As = f*f*f*f*G*F;
      double Bs = f*f*f*f*G*D;
      double Cs = G*(f*f*f*f*E+2*f*f*F);
      double Ds = 2*f*f*G*D;
      double Es = G*(2*f*f*E+F);
      double Fs = D*G;
      double Gs = G*E;
      
      std::vector<double> coeff(7);
      
      coeff[0] = As;
      coeff[1] = Ap+Bs;
      coeff[2] = Bp+Cs;
      coeff[3] = Cp+Ds;
      coeff[4] = Dp+Es;
      coeff[5] = Ep+Fs;
      coeff[6] = Gs;
      
      std::vector<double> sol;
      
      mpl::math::polySolveAll(coeff, sol);
      
      sol.push_back((1.0/f*f)+(c*c/(a*a+s*s*c*c)));
            
      double minCost = DBL_MAX;
      int index = -1;
      
      for(int i=0; i<sol.size(); ++i) {
        double value = utilsOptimalTriangulation::cost(a, b, c, d, f, s, sol[i]);
        if(value<minCost) { minCost = value; index = i; }
      }
      
      cv::Vec3d point1 = utilsOptimalTriangulation::closestPoint(sol[index]*f, 1, -sol[index]);
      cv::Vec3d point2 = utilsOptimalTriangulation::closestPoint(-s*(c*sol[index]+d), a*sol[index]+b, c*sol[index]+d);
      
      cv::Mat pt1 = T1.inv() * R1.t() * point1;
      cv::Mat pt2 = T2.inv() * R2.t() * point2;
      
      mpl::Mat3 tmpE;
      
      tmpE(0,1) = -E2.at<double>(2);
      tmpE(0,2) =  E2.at<double>(1);
      tmpE(1,0) =  E2.at<double>(2);
      tmpE(1,2) = -E2.at<double>(0);
      tmpE(2,0) = -E2.at<double>(1);
      tmpE(2,1) =  E2.at<double>(0);
      
      cv::Mat R = tmpE * fundamentalMatrix;
            
      cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64FC1);
      
      P1.at<double>(0,0) = 1; P1.at<double>(1,1) = 1; P1.at<double>(2,2) = 1;
      
      cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64FC1);
      
      P2.at<double>(0,3) = E2.at<double>(0);
      P2.at<double>(1,3) = E2.at<double>(1);
      P2.at<double>(2,3) = E2.at<double>(2);
      
      P2.at<double>(0,0) = R.at<double>(0,0); P2.at<double>(0,1) = R.at<double>(0,1); P2.at<double>(0,2) = R.at<double>(0,2);
      P2.at<double>(1,0) = R.at<double>(1,0); P2.at<double>(1,1) = R.at<double>(1,1); P2.at<double>(1,2) = R.at<double>(1,2);
      P2.at<double>(2,0) = R.at<double>(2,0); P2.at<double>(2,1) = R.at<double>(2,1); P2.at<double>(2,2) = R.at<double>(2,2);
      
      cv::Point2d p1; p1.x = pt1.at<double>(0) / pt1.at<double>(2); p1.y = pt1.at<double>(1) / pt1.at<double>(2);
      cv::Point2d p2; p2.x = pt2.at<double>(0) / pt2.at<double>(2); p2.y = pt2.at<double>(1) / pt2.at<double>(2);
            
      mpl::vision::reconstruct(p1, P1, p2, P2, point3D);
      
    }
    
  }
  

  //****************************************************************************/
  // fundamentalMatrixFromEightPoints
  //****************************************************************************/
  // Multiple View Geometry in Computer Vision
  // Pag 282 Alg 11.1
  template<typename P2D>
  void fundamentalMatrixFromEightPoints(const std::vector<P2D> & points1, const std::vector<P2D> & points2, cv::Mat & fundamentalMatrix) {
    
    if(points1.size() != points2.size()) {
      fprintf(stderr, "error the points set must be of the same size\n");
      abort();
    }
    
    std::vector<cv::Point2d> points1Norm = points1;
    std::vector<cv::Point2d> points2Norm = points2;
    
    // normalizzo i punti in maniera isotropica
    mpl::Mat3 H1Inv = mpl::vision::normalization::isotropic(points1Norm);
    mpl::Mat3 H2Inv = mpl::vision::normalization::isotropic(points2Norm);
    
    mpl::Mat A((int)points1Norm.size(), 9);
    
    for(int i=0; i<points1Norm.size(); ++i) {
      
      A(i,0) = points1Norm[i].x * points2Norm[i].x;
      A(i,1) = points1Norm[i].y * points2Norm[i].x;
      A(i,2) =                    points2Norm[i].x;
      A(i,3) = points1Norm[i].x * points2Norm[i].y;
      A(i,4) = points1Norm[i].y * points2Norm[i].y;
      A(i,5) =                    points2Norm[i].y;
      A(i,6) = points1Norm[i].x;
      A(i,7) = points1Norm[i].y;
      A(i,8) = 1;
      
    }
        
    A = A.t() * A;
    
    mpl::Vec eigenvalues;
    mpl::Mat eigenvectors;
    
    mpl::math::eigen(A, eigenvalues, eigenvectors);
    
    mpl::Mat3 F;
    
    F(0,0) = eigenvectors(0,0);
    F(0,1) = eigenvectors(0,1);
    F(0,2) = eigenvectors(0,2);
    F(1,0) = eigenvectors(0,3);
    F(1,1) = eigenvectors(0,4);
    F(1,2) = eigenvectors(0,5);
    F(2,0) = eigenvectors(0,6);
    F(2,1) = eigenvectors(0,7);
    F(2,2) = eigenvectors(0,8);
    
    cv::Mat W,U; mpl::Mat V;
    
    mpl::math::svd(F, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    
    mpl::Mat3 D;
    
    D(0,0) = W.at<double>(0);
    D(1,1) = W.at<double>(1);
    
    F = U * D * V;
   
    F = H2Inv.t() * F * H1Inv;
        
    fundamentalMatrix = F.clone();
    
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
    
    cv::Mat W, U, V;
    
    mpl::math::svd(AA, W, U, V, cv::SVD::MODIFY_A);
    
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
    
    for(int i=0; i<3; ++i){
      for(int j=0; j<3; ++j){
        R.at<double>(i,j) = Pl.at<double>(i,j);
      }
    }
    
    // std::cout << "R " << R << std::endl;
    cv::Mat RInv = R.inv();
    
    cv::Mat T = cv::Mat::zeros(cv::Size(1,3), CV_64FC1);
    T.at<double>(0,0) = -Pl.at<double>(0,3);
    T.at<double>(1,0) = -Pl.at<double>(1,3);
    T.at<double>(2,0) = -Pl.at<double>(2,3);
        
    cv::Mat h = cv::Mat::zeros(cv::Size(1,3), CV_64FC1);
    
    //std::cout << "T " << T << std::endl;
    for(int i=0; i<3; ++i){
      
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
    
    cv::Mat Prl = Pr * H;
    
    cv::Mat M = Prl(cv::Rect(0,0,3,3));
    
    mpl::Mat m(3,3);
    
    m(0,1) = -Prl.at<double>(2,3); m(0,2) =  Prl.at<double>(1,3);
    m(1,0) =  Prl.at<double>(2,3); m(1,2) = -Prl.at<double>(0,3);
    m(2,0) = -Prl.at<double>(1,3); m(2,1) =  Prl.at<double>(0,3);
    
    F = m * M;
    
  }
  
  /*****************************************************************************/
  // epipolarLines
  /*****************************************************************************/
  // Retta ax + by + c = 0
  cv::Vec3d epipolarLine(const cv::Point2d & point, cv::Mat F) {
    
    cv::Vec3d lineParameters;
        
    lineParameters[0] = point.x * F.at<double>(0,0) + point.y * F.at<double>(0,1) + 1.0 * F.at<double>(0,2);
    lineParameters[1] = point.x * F.at<double>(1,0) + point.y * F.at<double>(1,1) + 1.0 * F.at<double>(1,2);
    lineParameters[2] = point.x * F.at<double>(2,0) + point.y * F.at<double>(2,1) + 1.0 * F.at<double>(2,2);
        
    return lineParameters;
    
  }

  //****************************************************************************/
  // getRT
  //****************************************************************************/
  cv::Mat getRT(double alpha, double beta, double gamma, double cams_distance, const cv::Point3f & cam_offset, double alphaTime = 0) {
    // define the rotation matrix R as the product of 3 rotations:
    // angle gamma about the z axis
    // angle beta about the x axis
    // angle alpha about the y axis
    // R is the rotation matrix that brings the cam reference frame in the original reference
    
    cv::Mat R(3, 3, CV_64FC1, 0.0);
    R = mpl::geometry::computeRotationalMatrixInY(-alphaTime) *
        mpl::geometry::computeRotationalMatrixInZ(-gamma) *
        mpl::geometry::computeRotationalMatrixInX(-beta) *
        mpl::geometry::computeRotationalMatrixInY(-alpha);
    
    cv::Mat Rt(3, 4, CV_64FC1, 0.0);
    
    //define the first 3x3 part of Rt as the matrix R
    for(int col=0; col<3; col++) {
      for(int row=0; row<3; row++) {
        Rt.at<double>(row,col) = R.at<double>(row,col);
      }
    }
    
    //define the translation vector
    cv::Mat R_Rotor(3, 3, CV_64FC1, 0.0);
    R_Rotor = mpl::geometry::computeRotationalMatrixInY(-alphaTime);
    
    cv::Mat Offset(3,1,CV_64FC1, 0.0);
    Offset.at<double>(0,0) = -cam_offset.x;
    Offset.at<double>(1,0) =  cam_offset.y;
    Offset.at<double>(2,0) =  cam_offset.z;
    
    cv::Mat T_Rotor(3, 1, CV_64FC1, 0.0);
    T_Rotor = R_Rotor * Offset;
    
    cv::Mat R_2Original(3, 3, CV_64FC1, 0.0);
    R_2Original = mpl::geometry::computeRotationalMatrixInY(-alphaTime) *
                  mpl::geometry::computeRotationalMatrixInZ(-gamma) *
                  mpl::geometry::computeRotationalMatrixInX(-beta) *
                  mpl::geometry::computeRotationalMatrixInY(-alpha);
    
    cv::Mat Baseline(3, 1, CV_64FC1, 0.0);
    Baseline.at<double>(0,0) = cams_distance * 0.5;
    Baseline.at<double>(1,0) = 0;
    Baseline.at<double>(2,0) = 0;
    
    cv::Mat T_2Original(3, 1, CV_64FC1, 0.0);
    T_2Original = R_2Original * Baseline;
    
    Rt.at<double>(0,3) = T_2Original.at<double>(0,0) + T_Rotor.at<double>(0,0);
    Rt.at<double>(1,3) = T_2Original.at<double>(1,0) + T_Rotor.at<double>(1,0);
    Rt.at<double>(2,3) = T_2Original.at<double>(2,0) + T_Rotor.at<double>(2,0);
    
    return Rt.clone();
    
  }
  
} /* namespace mpl::vision */

#endif /* vision_h */
