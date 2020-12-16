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

#include "mvee.hpp"


/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []){
  
  uint32_t size = 100000;
  uint32_t dims = 3;
  
  printf("Create random %u points whitin an ellipsoid...", size);

  cv::RNG rng;

  /* elipsoid axes */
  cv::Mat A = (cv::Mat_<double>(dims, 1) << 3, 5, 7);//rng.uniform(0.0, 100.0), rng.uniform(0.0, 100.0), rng.uniform(0.0, 100.0));
  
  //std::cout << "A " << A.t() << std::endl;
  
  /* points matrix DxN */
  cv::Mat P = cv::Mat(dims, size, CV_64F);
  
  /* create N random ellipsoid points */
  for(int i=0; i<size; ++i){
    
    /* random numbers */
    cv::Mat r = (cv::Mat_<double>(dims, 1) << rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0));
    
    /* random point coordinates */
    cv::Mat p = 2.0 * A.mul(r);
    
    /* check if the point is inside the ellipsoid */
    if(pow(p.at<double>(0)/A.at<double>(0),2)+pow(p.at<double>(1)/A.at<double>(1),2)+pow(p.at<double>(2)/A.at<double>(2),2) > 1) { --i; continue; }
    
    /* assigned the point */
    P.at<double>(0,i) = p.at<double>(0);
    P.at<double>(1,i) = p.at<double>(1);
    P.at<double>(2,i) = p.at<double>(2);
    
    //std::cout << "p " << p.t() << std::endl;
    
  }
  
  //std::cout << "P " << P << std::endl;
  
  /* elipsoid rotation angle */
  //cv::Mat angle = (cv::Mat_<double>(dims, 1) << -M_PI, 0.0, 0.0);
  cv::Mat angle = (cv::Mat_<double>(dims, 1) << 0.0, M_PI_2, 0.0);
  
  //std::cout << "angle " << angle.t() << std::endl;
  
  /* rotation matrix */
  cv::Mat R;
  
  /* from rotation angle to rotation matrix */
  cv::Rodrigues(angle, R);
  
  /* elipsoid points baricenter */
  cv::Mat p0 = P * cv::Mat(size, 1, CV_64F, cv::Scalar(1.0/size));
  
  /* eplipsoid center */
  cv::Mat C = (cv::Mat_<double>(dims, 1) << 2, 4, 6);//rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0));
  
  //std::cout << "C " << C.t() << std::endl;
  //std::cout << "p0 " << p0.t() << std::endl;
  //std::cout << "R " << R << std::endl;

  /* rotatate eplipsoid points */
  for(int i=0; i<size; ++i){
    
    cv::Mat temp = (R * (P.col(i)-p0)) + (C + p0);
    //cv::Mat temp = (R * P.col(i)) + C;
    
    P.at<double>(0,i) = temp.at<double>(0);
    P.at<double>(1,i) = temp.at<double>(1);
    P.at<double>(2,i) = temp.at<double>(2);

    //std::cout << P.col(i).t() << std::endl;
    
  }
  
  printf("done!\n\n");

  //exit(1);
  
  cv::Mat outC;
  cv::Mat outR;
  cv::Mat outA;
  
  printf("Fitting ellipsoid...");

  double EPS = 1.0e-03;
  
  double runtime;
  
#if(0)
  
  runtime = (double)cv::getTickCount();
  
  mvee::fitV1(P, outC, outR, outA, EPS);
	
  runtime = ((double)cv::getTickCount() - runtime)/cv::getTickFrequency();
    
  printf("done!\n\n");
  
  printf("Check errors: \n\n");
  
  /*
  printf("\tRotational ");
  
  if(cv::norm(cv::Mat::eye(3, 3, outR.type()), (outR.t() * outR)) > FLT_EPSILON){
    
    printf("inf inf inf\n");
    
  } else {
    
    printf("\n");
    
    std::cout << R << std::endl;
    std::cout << outR << std::endl;

    cv::Mat outAngle;
    
    cv::Rodrigues(outR, outAngle);
    
    std::cout << "angle " << angle.t() << " outAngle " << outAngle.t() << std::endl;
    
    for(uint32_t i=0; i<dims; ++i)
      printf("%e ", fabs(angle.at<double>(i)-outAngle.at<double>(i)));
    
    printf("\n");
    
  }
  */
   
  printf("\tBarycenter ");
  
  for(uint32_t i=0; i<dims; ++i)
    printf("%e ", fabs(C.at<double>(i)-outC.at<double>(i)));
 
  printf("\n");

  printf("\tAxes size  ");
  
  for(uint32_t i=0; i<dims; ++i)
    printf("%e ", fabs(A.at<double>(i)-outA.at<double>(i)));
  
  printf("\n\n");
  
  printf("Result founded in %g sec\n\n", runtime);
  
#endif
  
  runtime = (double)cv::getTickCount();
  
  mvee::fitV2(P, outC, outR, outA, EPS);
  
  runtime = ((double)cv::getTickCount() - runtime)/cv::getTickFrequency();
  
  printf("done!\n\n");
  
  printf("Check errors: \n\n");
  
  /*
   printf("\tRotational ");
   
   if(cv::norm(cv::Mat::eye(3, 3, outR.type()), (outR.t() * outR)) > FLT_EPSILON){
   
   printf("inf inf inf\n");
   
   } else {
   
   printf("\n");
   
   std::cout << R << std::endl;
   std::cout << outR << std::endl;
   
   cv::Mat outAngle;
   
   cv::Rodrigues(outR, outAngle);
   
   std::cout << "angle " << angle.t() << " outAngle " << outAngle.t() << std::endl;
   
   for(uint32_t i=0; i<dims; ++i)
   printf("%e ", fabs(angle.at<double>(i)-outAngle.at<double>(i)));
   
   printf("\n");
   
   }
   */
  
  printf("\tBarycenter ");
  
  for(uint32_t i=0; i<dims; ++i)
  printf("%e ", fabs(C.at<double>(i)-outC.at<double>(i)));
  
  printf("\n");
  
  printf("\tAxes size  ");
  
  for(uint32_t i=0; i<dims; ++i)
  printf("%e ", fabs(A.at<double>(i)-outA.at<double>(i)));
  
  printf("\n\n");
  
  printf("Result founded in %g sec\n\n", runtime);

}
