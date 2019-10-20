/*
 * MIT License
 *
 * Copyright (c) 2017 Leonardo Parisi
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
