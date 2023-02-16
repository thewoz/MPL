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
#include <mpl/geometry/geometry.hpp>

//*****************************************************************************/
// namespace vision
//*****************************************************************************/
namespace mpl::vision {

  //*****************************************************************************/
  // initProjectionMatrix
  //*****************************************************************************/
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

  //*****************************************************************************/
  // loadProjectionMatrix
  //*****************************************************************************/
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

  //*****************************************************************************/
  // loadProjectionMatrix
  //*****************************************************************************/
  void loadProjectionMatrix(const char * filename, cv::Mat & projectionMatrix, std::size_t line = 0){
    
    FILE * input = mpl::io::open(filename, "r");
    
    loadProjectionMatrix(input, projectionMatrix, line);
    
    mpl::io::close(input);
    
  }

  //*****************************************************************************/
  // initCameraMatrix
  //*****************************************************************************/
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

  //*****************************************************************************/
  // loadCameraMatrix
  //*****************************************************************************/
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

  //*****************************************************************************/
  // loadCameraMatrix
  //*****************************************************************************/
  void loadCameraMatrix(const char * file, cv::Mat & cameraMatrix, std::size_t line = 0){
    
    FILE * input = mpl::io::open(file, "r");
    
    loadCameraMatrix(input, cameraMatrix, line);
    
    mpl::io::close(input);
    
  }

  //*****************************************************************************/
  // getCameraCenter() - Multiple View Geometry p. 156-159
  //*****************************************************************************/
  void getCameraCenter(const cv::Mat & P, cv::Point3d & center) {
    
    cv::Mat M  = cv::Mat(3, 3, CV_64F);
    cv::Mat MC = cv::Mat(3, 1, CV_64F);
    
    // NOTE: si puo semplificare
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
    
  }

  //*****************************************************************************/
  // findPlanePassingBy
  //*****************************************************************************/
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

  //****************************************************************************/
  // projectionMatrixFromEssentialMatrixDecomposition()
  //****************************************************************************/
  // Calculates the camera RT matrix from the SVD of the essential matrix.
  // See pp. 258 H&Z
  cv::Mat RTMatrixFromEssentialMatrixDecomposition(cv::Mat U, cv::Mat W, cv::Mat V, double sign_u3) {
    
    cv::Mat RT(3, 4, CV_64FC1);
    
    cv::Mat R = cv::determinant(U*V) * U * W * V;
    
    // Fill the 3x4 matrix P.
    // The last column is composed of the last column of U, with the sign based on sign_u3
    for(int i = 0; i<3; i++) {
      for(int j = 0; j<4; j++) {
        if(j==3)
          RT.at<double>(i,3) = sign_u3*U.at<double>(i,2);
        else
          RT.at<double>(i,j) = R.at<double>(i,j);
      }
    }
    
    return RT;
    
  }

  //****************************************************************************/
  // RTFromEessential()
  //****************************************************************************/
  // Calculates the camera RT matrix of the P from the SVD of the essential matrix
  // It return only the left RT Matrix since the right one is equalt to [I]
  // See pp. 258 H&Z
  cv::Mat RTFromEessential(cv::Mat E, cv::Mat Kright, cv::Mat Kleft, cv::Point2d pointRight, cv::Point2d pointLeft, double camDist) {
    
    cv::Mat W,U; mpl::Mat V;
    mpl::math::svd(E, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    
    // based on H&Z pp. 258
    cv::Mat Worth = (cv::Mat_<double>(3,3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
    
    // RT non P
    cv::Mat RT1 = mpl::vision::RTMatrixFromEssentialMatrixDecomposition(U, Worth,     V,  camDist);
    cv::Mat RT2 = mpl::vision::RTMatrixFromEssentialMatrixDecomposition(U, Worth,     V, -camDist);
    cv::Mat RT3 = mpl::vision::RTMatrixFromEssentialMatrixDecomposition(U, Worth.t(), V,  camDist);
    cv::Mat RT4 = mpl::vision::RTMatrixFromEssentialMatrixDecomposition(U, Worth.t(), V, -camDist);
    
    cv::Mat RTLeft[4] = {RT1, RT2, RT3, RT4};
    cv::Mat PLeft[4] = {Kleft*RT1, Kleft*RT2, Kleft*RT3, Kleft*RT4};
    
    // RT non P
    cv::Mat PnormRight = (cv::Mat_<double>(3,4) << 1, 0, 0, 0,
                          0, 1, 0, 0,
                          0, 0, 1, 0);
    cv::Mat PRight = Kright*PnormRight;
    
    // Determine which P for cam3 to use should have -ve depth, t_x must be -ve
    cv::Mat M_right(3,3,CV_64FC1);
    cv::Mat M_left(3,3,CV_64FC1);
    
    //cv::Mat thisX_coords(4,1,CV_64FC1);
    
    cv::Point4d thisX_coords;
    
    int left_P_index = -1;
    
    for(int i=0; i<4; ++i) {
      
      mpl::vision::reconstruct(pointRight, PRight, pointLeft, PLeft[i], thisX_coords);
      
      // Get normalized X-coords from triangulation (H&Z pp. 312)
      thisX_coords.x /= thisX_coords.w;
      thisX_coords.y /= thisX_coords.w;
      thisX_coords.z /= thisX_coords.w;
      thisX_coords.w /= thisX_coords.w;
      
      double w_right = (PRight.at<double>(2,1) * thisX_coords.x) +
      (PRight.at<double>(2,1) * thisX_coords.y) +
      (PRight.at<double>(2,2) * thisX_coords.z) +
      (PRight.at<double>(2,3) * thisX_coords.w);
      
      double w_left = (PLeft[i].at<double>(2,1) * thisX_coords.x) +
      (PLeft[i].at<double>(2,1) * thisX_coords.y) +
      (PLeft[i].at<double>(2,2) * thisX_coords.z) +
      (PLeft[i].at<double>(2,3) * thisX_coords.w);
      
      for(int m_row = 0; m_row < 3; m_row++) {
        for(int n_col = 0; n_col < 3; n_col++) {
          M_right.at<double>(m_row,n_col) = PRight.at<double>(m_row,n_col);
          M_left.at<double>(m_row,n_col)  = PLeft[i].at<double>(m_row,n_col);
        }
      }
      
      double detM_right = cv::determinant(M_right);
      double detM_left  = cv::determinant(M_left);
      
      double normM3_right = cv::norm(M_right.row(2));
      double normM3_left  = cv::norm(M_left.row(2));
      
      double depth_right = (detM_right/abs(detM_right))*w_right/(normM3_right);
      
      double depth_left  = (detM_left/abs(detM_left))*w_left/(normM3_left);
      
      // Check whether cam3 is to the "left" of cam2 and the depth is -ve in both cameras
      if((RTLeft[i].at<double>(0,3) > 0) && (depth_right > 0) && (depth_left > 0))
        left_P_index = i;
      
    }
    
    if(left_P_index >= 0)
      return RTLeft[left_P_index];
    else {
      cv::Mat P_null = (cv::Mat_<double>(3,4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      return P_null;
    }
    
  }

  //****************************************************************************/
  // essentialMatrixLinear()
  //****************************************************************************/
  // Calculate the essential matrix for a pair of calibrated cameras.
  // Note _right denotes the camera on the right when behind both cameras, looking at the target.
  // Moreover the _right camera will have P = [I | 0]
  template <class T>
  cv::Mat essentialMatrixLinear(cv::Mat Kright, cv::Mat Kleft, const std::vector<T> & pointsRight, const std::vector<T> & pointsLeft) {
    
    if(pointsRight.size() != pointsLeft.size()){
      fprintf(stderr, "error in essentialMatrixLinear() points must have the same size\n");
      abort();
    }
    
    // matrice per mettere le equazioni lineari
    cv::Mat Elin((int)pointsRight.size(), 9, CV_64FC1);
    
    cv::Mat homoPointRight(3, 1, CV_64FC1);
    cv::Mat homoPointLeft(3, 1, CV_64FC1);
    
    cv::Mat tmpPointRight(3, 1, CV_64FC1);
    cv::Mat tmpPointLeft(1, 3, CV_64FC1);
    
    // Generate Alin * Evec = 0 linear solution for essential matrix
    // Simila method as shown in H&Z pp. 279
    for(int i=0; i<pointsRight.size(); ++i) {
      
      homoPointRight.at<double>(0) = pointsRight[i].x;
      homoPointRight.at<double>(1) = pointsRight[i].y;
      homoPointRight.at<double>(2) = 1;
      
      homoPointLeft.at<double>(0) = pointsLeft[i].x;
      homoPointLeft.at<double>(1) = pointsLeft[i].y;
      homoPointLeft.at<double>(2) = 1;
      
      cv::Mat KleftT = Kleft.t();
      
      // condition points to be used to find essential matrix H&Z pp.257
      tmpPointRight = Kright.inv()  * homoPointRight;
      tmpPointLeft  = homoPointLeft.t() * KleftT.inv();
      
      // Form linear set of equations
      Elin.at<double>(i,0) = tmpPointLeft.at<double>(0) * tmpPointRight.at<double>(0);
      Elin.at<double>(i,1) = tmpPointLeft.at<double>(0) * tmpPointRight.at<double>(1);
      Elin.at<double>(i,2) = tmpPointLeft.at<double>(0) * tmpPointRight.at<double>(2);
      Elin.at<double>(i,3) = tmpPointLeft.at<double>(1) * tmpPointRight.at<double>(0);
      Elin.at<double>(i,4) = tmpPointLeft.at<double>(1) * tmpPointRight.at<double>(1);
      Elin.at<double>(i,5) = tmpPointLeft.at<double>(1) * tmpPointRight.at<double>(2);
      Elin.at<double>(i,6) = tmpPointLeft.at<double>(2) * tmpPointRight.at<double>(0);
      Elin.at<double>(i,7) = tmpPointLeft.at<double>(2) * tmpPointRight.at<double>(1);
      Elin.at<double>(i,8) = tmpPointLeft.at<double>(2) * tmpPointRight.at<double>(2);
      
    }
    
    cv::Mat W,U; mpl::Mat V;
    
    mpl::math::svd(Elin, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    
    cv::Mat Evec = V.row(8);
    
    cv::Mat Einit(3, 3, CV_64FC1);
    
    Einit.at<double>(0,0) = Evec.at<double>(0);
    Einit.at<double>(0,1) = Evec.at<double>(1);
    Einit.at<double>(0,2) = Evec.at<double>(2);
    Einit.at<double>(1,0) = Evec.at<double>(3);
    Einit.at<double>(1,1) = Evec.at<double>(4);
    Einit.at<double>(1,2) = Evec.at<double>(5);
    Einit.at<double>(2,0) = Evec.at<double>(6);
    Einit.at<double>(2,1) = Evec.at<double>(7);
    Einit.at<double>(2,2) = Evec.at<double>(8);
    
    mpl::math::svd(Einit, W, U, V, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    
    cv::Mat diagFix = cv::Mat::zeros(3, 3, CV_64FC1);
    
    diagFix.at<double>(0,0) = diagFix.at<double>(1,1) = (W.at<double>(0) + W.at<double>(1)) * 0.5;
    
    cv::Mat E = U * diagFix * V;
    
    return E;
    
  }

  //****************************************************************************/
  // anglesFromRTMatrix()
  //****************************************************************************/
  void anglesFromRTMatrix(cv::Mat R, double distance, std::vector<double> & angles) {
    
    double alpha, beta, gamma, delta, epsilon;
    double c_beta;
    double s_beta;
    
    if(R.at<double>(1,2)==1.0) {
      fprintf(stderr, "I can't find the angles");
      abort();
    }
    
    beta = asin(-R.at<double>(1,2));
    
    alpha = atan(R.at<double>(0,2)/R.at<double>(2,2));
    if(R.at<double>(2,2)/cos(beta) < 0) alpha = alpha + M_PI;
    
    gamma = atan(R.at<double>(1,0)/R.at<double>(1,1));
    if(R.at<double>(1,1)/cos(beta) < 0) gamma = gamma + M_PI;
    
    c_beta=cos(beta);
    s_beta=sin(beta);
    
    //check for the signs of cos anf sin
    if(c_beta<0.0) {
      if(s_beta<0.0) beta-=M_PI;
      else beta+=M_PI;
    }
    
    epsilon = acos(R.at<double>(0,3)/distance);
    
    if(epsilon == 0) {
      delta = 0;
    } else {
      delta = atan(R.at<double>(1,3)/R.at<double>(2,3));
      if(R.at<double>(2,3)/(distance*sin(epsilon)) < 0)
        delta = delta + M_PI;
    }
    
    angles.resize(5);
    
    angles[0] = alpha;
    angles[1] = beta;
    angles[2] = gamma;
    angles[3] = delta;
    angles[4] = epsilon;
    
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

  //*****************************************************************************/
  // alignment
  //*****************************************************************************/
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

  //*****************************************************************************/
  // alignment
  //*****************************************************************************/
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

  //*****************************************************************************/
  // isOnConic
  //*****************************************************************************/
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

  //  //*****************************************************************************/
  //  // fundamentalFromProjections
  //  //*****************************************************************************/
  //  void fundamentalFromProjections(const cv::Mat & P1, const cv::Mat & P2, cv::Mat & F) {
  //
  //    double T[4][4] = { 0.0, };
  //
  //    cv::Mat A; A.create(3, 3, CV_64F);
  //
  //    // copio P2 in U
  //    P2(cv::Rect(0,0,3,3)).copyTo(A);
  //
  //    cv::Mat B; B.create(3, 1, CV_64F);
  //
  //    B = - P2.col(3);
  //
  //    cv::Mat X;
  //
  //    cv::solve(A, B, X);
  //
  //    for(int i=0; i<3; ++i){
  //
  //      const int j1 = (i + 1) % 3;
  //      const int j2 = (i + 2) % 3;
  //
  //      const double h[3] = { P2.at<double>(j1,1) * P2.at<double>(j2,2) - P2.at<double>(j1,2) * P2.at<double>(j2,1),
  //                            P2.at<double>(j1,2) * P2.at<double>(j2,0) - P2.at<double>(j1,0) * P2.at<double>(j2,2),
  //                            P2.at<double>(j1,0) * P2.at<double>(j2,1) - P2.at<double>(j1,1) * P2.at<double>(j2,0) };
  //
  //      const double z = h[0] * P2.at<double>(i,0) + h[1] * P2.at<double>(i,1) + h[2] * P2.at<double>(i,2);
  //
  //      for(int j=0; j<3; ++j) T[j][i] = h[j] / z;
  //
  //      T[i][3] = X.at<double>(i);
  //
  //    }
  //
  //    T[3][3] = 1.0;
  //
  //    double C[3][3] = { 0.0, };
  //
  //    double t[3];
  //
  //    for(int i=0; i<3; ++i) {
  //
  //      for(int j=0; j<3; ++j) {
  //
  //        C[i][j] = 0.0;
  //
  //        for(int k=0; k<4; ++k) C[i][j] += P1.at<double>(i,k) * T[k][j];
  //
  //      }
  //
  //      t[i] = 0.0;
  //
  //      for(int k=0; k<4; ++k) t[i] += P1.at<double>(i,k) * T[k][3];
  //
  //    }
  //
  //    double tx[3][3] = { {0.0, -t[2], t[1]}, {t[2], 0.0, -t[0]}, {-t[1], t[0], 0.0} };
  //
  //    for(int i=0; i<3; ++i){
  //
  //      for(int j=0; j<3; ++j){
  //        F.at<double>(i,j) = 0.0;
  //        for(int k=0; k<3; ++k) F.at<double>(i,j) += tx[i][k] * C[k][j];
  //      }
  //
  //    }
  //
  //  }
  //
  //  //****************************************************************************
  //  // fundamentalFromProjections
  //  //****************************************************************************
  //  void fundamentalFromProjections(cv::InputArray _P1, cv::InputArray _P2, cv::OutputArray _F) {
  //
  //    const cv::Mat P1 = _P1.getMat(), P2 = _P2.getMat();
  //
  //    const int depth = P1.depth();
  //
  //    CV_Assert((P1.cols == 4 && P1.rows == 3) && P1.rows == P2.rows && P1.cols == P2.cols);
  //    CV_Assert((depth == CV_32F || depth == CV_64F) && depth == P2.depth());
  //
  //    _F.create(3, 3, depth);
  //
  //    if(depth == CV_32F) fundamentalFromProjections<float> (P1, P2, _F.getMat());
  //    else                fundamentalFromProjections<double>(P1, P2, _F.getMat());
  //
  //  }


  //*****************************************************************************/
  // pseudoInverse
  //*****************************************************************************/
  cv::Mat pseudoInverse(const cv::Mat & M) {
    
    return M.t() * (M*M.t()).inv();
    
  }

  //*****************************************************************************/
  // fundamentalFromGeneralProjections
  //*****************************************************************************/
  // See Multiple View Geometry in Computer Vision p. 246 and 254
  // Computation from camera matrices P, P′:
  // General cameras
  // F = [e′]×P′P+, where P+ is the pseudo-inverse of P, and e′ = P′C, with PC = 0.
  void fundamentalFromGeneralProjections(const cv::Mat & P1, const cv::Mat & P2, cv::Mat & F) {
    
    cv::Point3d C;
    
    getCameraCenter(P1, C);
    
    cv::Mat Cm = cv::Mat::zeros(cv::Size(1,4), CV_64FC1);
    
    Cm.at<double>(0) = C.x;
    Cm.at<double>(1) = C.y;
    Cm.at<double>(2) = C.z;
    Cm.at<double>(3) = 1.0;

    cv::Mat e2 = P2 * Cm;
        
    cv::Mat ec = cv::Mat::zeros(cv::Size(3,3), CV_64FC1);
    
    ec.at<double>(0,1) = -e2.at<double>(2); ec.at<double>(0,2) =  e2.at<double>(1);
    ec.at<double>(1,0) =  e2.at<double>(2); ec.at<double>(1,2) = -e2.at<double>(0);
    ec.at<double>(2,0) = -e2.at<double>(1); ec.at<double>(2,1) =  e2.at<double>(0);
    
    cv::Mat P1p = pseudoInverse(P1);
        
    F = ec * P2 * P1p;
    
  }

  //*****************************************************************************/
  // fundamentalFromCanonicalProjections
  //*****************************************************************************/
  // See Multiple View Geometry in Computer Vision p. 246 and 254
  // Computation from camera matrices P, P′:
  // Canonical cameras P=[I|0] P'=[M|m]
  // F = [e′]×M = M−T[e]×, where e′ = m and e = M−1m
  void fundamentalFromCanonicalProjections(const cv::Mat & Pl, const cv::Mat & Pr, cv::Mat & F) {
    
    cv::Mat H = cv::Mat::zeros(cv::Size(4,4), CV_64FC1);
    H.at<double>(3,0) = 1;
    H.at<double>(3,1) = 1;
    H.at<double>(3,2) = 1;
    H.at<double>(3,3) = 1;
    
    cv::Mat R = cv::Mat::zeros(cv::Size(3,3), CV_64FC1);
    
    //NOTE: perche non faccio questo?
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

  //  //*****************************************************************************/
  //  // epipolarLine
  //  //*****************************************************************************/
  //  template <class T2D>
  //  inline void epipolarLine(const T2D & pt, cv::Vec3f & line, const cv::Mat & funMat) {
  //
  //    //TODO: check matrix
  //
  //    double * matFun = ((double*)funMat.data);
  //
  //    line[0] = pt.x * matFun[0] + pt.y * matFun[1] + matFun[2];
  //    line[1] = pt.x * matFun[3] + pt.y * matFun[4] + matFun[5];
  //    line[2] = pt.x * matFun[6] + pt.y * matFun[7] + matFun[8];
  //
  //  }
  //
  //  //*****************************************************************************/
  //  // epipolarLine
  //  //*****************************************************************************/
  //  template <class T2D>
  //  inline cv::Vec3f epipolarLine(const T2D & pt, const cv::Mat & funMat) {
  //
  //    cv::Vec3f line;
  //
  //    epipolarLine(pt, line, funMat);
  //
  //    return line;
  //
  //
  //  }

  //*****************************************************************************/
  // epipolarLines
  //*****************************************************************************/
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
