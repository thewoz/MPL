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

#include "include/kabsch.hpp"


/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []){

  uint32_t size = 100000;
  uint32_t dim  = 3;
  
  bool computeOptimalScale = true;

  printf("Create random %u points...", size);

  cv::Mat angle = (cv::Mat_<double>(dim, 1) << M_PI_2, 0.0, 0.0);

  cv::Mat R;

  cv::Rodrigues(angle, R);

  cv::Mat T = (cv::Mat_<double>(dim, 1) << 1, 3, 5);

  double S = 4.3;

  cv::Mat P = cv::Mat(dim, size, CV_64F);
 
  cv::RNG rng;

  rng.fill(P, cv::RNG::UNIFORM, -1000, 1000);
  
  cv::Mat p0 = P * cv::Mat(size, 1, CV_64F, cv::Scalar(1.0/size));
    
  cv::Mat Q = cv::Mat(dim, size, CV_64F);

  for(int i=0; i<size; ++i){
    
    cv::Mat temp = ((R * (P.col(i)-p0)) * S) + (T + p0);
    
    Q.at<double>(0,i) = temp.at<double>(0);
    Q.at<double>(1,i) = temp.at<double>(1);
    Q.at<double>(2,i) = temp.at<double>(2);
    
  }
  
  printf("done!\n\n");

  cv::Mat outR, outT;
  
  double outS = 0.0;
    
  printf("Running Kabsch...");
    
  double runtime = (double)cv::getTickCount();
  
  cv::Mat barycenter;
  
  if(computeOptimalScale) barycenter = kabsch::solve(P, Q, outR, outT, &outS);
  else barycenter = kabsch::solve(P, Q, outR, outT);

  runtime = ((double)cv::getTickCount() - runtime)/cv::getTickFrequency();

  printf("done!\n\n");
 
  printf("Check result: \n\n");

  bool isOk = true;
  
  printf("\tRotational...");
  
  if(cv::norm(cv::Mat::eye(3, 3, outR.type()), (outR.t() * outR)) > FLT_EPSILON){
    
    printf("error!\n");
  
  } else {

    cv::Mat outAngle;

    cv::Rodrigues(outR, outAngle);

    for(uint32_t i=0; i<dim; ++i){
      if(fabs(angle.at<double>(i)-outAngle.at<double>(i)) > FLT_EPSILON){
        printf("error!\n");
        isOk = false;
        break;
      }
    }
    
    if(isOk) printf("ok!\n");

  }
  
  printf("\tTranslation...");

  isOk = true;
  
  for(uint32_t i=0; i<dim; ++i){
    if(fabs(T.at<double>(i)-outT.at<double>(i)) > FLT_EPSILON){
      printf("error!\n");
      isOk = false;
      break;
    }
  }
  
  if(isOk) printf("ok!\n");
  
  isOk = true;
  
  if(computeOptimalScale){
    
    if(fabs(S - outS) > FLT_EPSILON) printf("\tScale...error!\n");
    else printf("\tScale...ok!\n");
    
  }
  
  printf("\tBaricenter...");
  
  isOk = true;
  
  for(uint32_t i=0; i<dim; ++i){
    if(fabs(p0.at<double>(i)-barycenter.at<double>(i)) > FLT_EPSILON){
      printf("error!\n");
      isOk = false;
      break;
    }
  }
  
  if(isOk) printf("ok!\n");

  printf("\n");
  
  printf("Result founded in %g sec\n\n", runtime);

}
