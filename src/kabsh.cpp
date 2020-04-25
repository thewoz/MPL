//
//  main.cpp
//  Kabsh
//
//  Created by Leonardo Parisi on 25/04/2020.
//  Copyright © 2020 MPL. All rights reserved.
//

#include <cstdlib>
#include <cstdio>

#include <vector>

#include <opencv2/opencv.hpp>

#include "geometry.hpp"

int main(int argc, const char * argv[]) {

  std::vector<cv::Point3d> pointsA;
  std::vector<cv::Point3d> pointsB;

  mpl::geometry::kabsch::info2D_t RTS;
  
  // Mi trovo la migliore rotoscala trasclazione tra i punti
  mpl::geometry::findBestRTS<2>(pointsA, pointsB, RTS.p0, RTS.R, RTS.T, RTS.S, 1.5);
  
  
  return 0;
  
}
