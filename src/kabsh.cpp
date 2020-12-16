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
