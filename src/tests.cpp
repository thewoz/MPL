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


#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include "cplex.hpp"

#ifdef TEST_LOADER

#include "loader.hpp"

/*****************************************************************************/
// points3D_t
/*****************************************************************************/
class points3D_t {
//class points3D_t : public loader_t {

    
private:
  
  std::vector<cv::Point3f> points;
  
public:
  
  void filler(const arguments_t & arguments) {
    
    cv::Point3f point;
    
    point.x = arguments.get<double>(0);
    point.y = arguments.get<double>(1);
    point.z = arguments.get<double>(2);
    
    points.push_back(point);
    
  }
  
  void print(){
    
    for(std::size_t i=0; i<points.size(); ++i)
      printf("%f %f %f\n", points[i].x, points[i].y, points[i].z);
    
  }
  
};


/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []){
  
  points3D_t points;
  
  double min, max;

  //points.load(argv[1], "3 4 5");
  
  //points.minmax(argv[1], 3, min, max);  printf("%e %e\n", min, max);
  
  loader_t loader;
  
  loader.load(argv[1], "3 4 5", points, &points3D_t::filler);

  loader.minmax(argv[1], 3, min, max);  printf("%e %e\n", min, max);
  
  points.print();
  
  return EXIT_SUCCESS;
  
}

#endif /* TEST_LOADER */


#ifdef DEPTH_MAP

#include <cobbs/vision/camera.hpp>

/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []){

  vision::camera_t camera1;
  vision::camera_t camera2;

  cv::Mat_<double> cameraMatrix1(3,3); // 3x3 matrix
  cv::Mat_<double> distCoeffs1(5,1);   // 5x1 matrix for five distortion coefficients
  cv::Mat_<double> cameraMatrix2(3,3); // 3x3 matrix
  cv::Mat_<double> distCoeffs2(5,1);   // 5x1 matrix
  cv::Mat_<double> R(3,3);             // 3x3 matrix, rotation left to right camera
  cv::Mat_<double> T(3,1);             // * 3 * x1 matrix, translation left to right proj. center
  // ^^ that's the main diff to your code, (3,1) instead of (4,1)

  cameraMatrix1 << 6654, 0, 1231, 0, 6654, 1037, 0, 0, 1;
  cameraMatrix2 << 6689, 0, 1249, 0, 6689, 991, 0, 0, 1;
  distCoeffs1   << -4.57e-009, 5.94e-017, 3.68e-008, -3.46e-008, 6.37e-023;
  distCoeffs2   << -4.72e-009, 2.88e-016, 6.2e-008, -8.74e-008, -8.18e-024;
  R <<  0.87, -0.003, -0.46, 0.001, 0.999, -0.003, 0.46, 0.002, 0.89;
  T << 228, 0, 0;

  cv::Mat R1,R2,P1,P2,Q;   // you're safe to leave OutpuArrays empty !
  Size imgSize(3000,3000); // wild guess from you cameramat ( not that it matters )

  cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imgSize, R, T, R1, R2, P1, P2, Q);

  cerr << "Q" << Q << endl;

  return 0;
  
}


#endif



#ifdef TEST_VISION

#include <cobbs/vision/cameraSystem.hpp>

#include <cobbs/geometric.hpp>

/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []){
  
  std::vector<cv::Point3d> points;
  
  cv::Mat R, T;
  double S;
  
  geometric::applyRTS3D(points, R, T, S);
  
  vision::camera_t camera;
  
  //camera.setTranslation(cv::Mat());
  //camera.setTranslation(std::vector<double>());

  camera.setTranslation(cv::Point3f());
  camera.setTranslation(cv::Point3d());

  camera.setTranslation(cv::Vec3f());
  camera.setTranslation(cv::Vec3d());
  //camera.setTranslation(cv::Vec3i());

  cv::Point3f p = camera.getRotationAngle<cv::Point3f>();
  //cv::Mat     m = camera.getRotationAngle<cv::Mat>();
  //int i = camera.getTranslation<int>();
  
  int a = 0;
  
  camera.setTranslation(a,a,a);

  cv::Point2f fff = camera.getOmega<cv::Point2f>();
  
  camera.setPosition(cv::Point3f(), cv::Point3d());
  
  cv::Point3f point3d;
  
  cv::Point2f b,c,d;
    
  vision::cameraSystem3_t trifocalSystem;
  
  trifocalSystem.loadProjectionMatrices("ddd");
  
  trifocalSystem.reproject(point3d, b, c, d);

  trifocalSystem.reconstruct(point3d, b, c, d);
  
  cv::Point3d pt3d = trifocalSystem.reconstruct<cv::Point3d>(b, c, d);

  double error = trifocalSystem.reconstructError(1.5, b, c, d);
  
  trifocalSystem.reconstructError(error, 1.5, b, c, d);
  
  return EXIT_SUCCESS;
  
}

#endif /* TEST_VISION */


#ifdef TEST_MUNKRES

#include <opencv2/opencv.hpp>

#include <cobbs/munkres/munkres.hpp>

/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []){
 
  munkres::Matrix<float> matrix(4, 4, FLT_MAX);

  matrix(0,1) = 4.0;
  matrix(1,0) = 4.0;
  matrix(3,2) = 4.0;
  matrix(2,3) = 4.0;

  munkres::Munkres<float> munkres;
  
  std::vector<cv::Vec2i> pairs;
  
  munkres.solve(matrix, pairs);
  
  for(uint32_t i=0; i<pairs.size(); ++i){
    
    printf("%d -> %d (%g)\n", pairs[i][0], pairs[i][1], matrix(pairs[i][0], pairs[i][1]));
    
    
  }
  
  munkres.solve(matrix);

  for(uint32_t i=0; i<4; ++i){
		  
    for(uint32_t j=0; j<4; ++j){
		    
      printf("%d -> %d (%g)", i, j, matrix(i,j));

      if(matrix(i,j) == 0) printf(" ok");
      
      printf("\n");

    }
    
  }
  
  
}

#endif /* TEST_MUNKRES */


#ifdef TESTLAGRANGE
/*****************************************************************************/
// Test Lagrange interpolation
/*****************************************************************************/
fprintf(stderr, "Test Lagrange interpolation:\n\n");

std::vector<double> x(10);
std::vector<double> y(10);

for(int i=0; i<10; ++i){ x[i] = i; y[i] = sin(i); printf("%f %f\n", x[i], y[i]); }

printf("\n");

std::vector<double> coeffs = lagrange::findCoefficients(x, y);

for(int i=0; i<coeffs.size(); ++i){ printf("%.15f, ", coeffs[i]); }

#endif

#ifdef TEST_CPLEX

#include <cstdlib>
#include <cstdio>

#include "cplex.hpp"

/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []){
 
  
  
  return 0;
  
}

#endif /* TEST_CPLEX */


