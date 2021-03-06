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


#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

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
  
  geometric::applyRTS(points, R, T, S);
  
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

#include <cobbs/munkres/munkres.hpp>

/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []){
 
  munkres::Matrix<float> matrix;
  
  matrix.resize(4, 4, FLT_MAX);

  matrix(0,1) = 4.0;
  matrix(1,0) = 4.0;
  matrix(3,2) = 4.0;
  matrix(2,3) = 4.0;

  munkres::Munkres<float> munkres;
  
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

  
  


