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

#include <vector>
#include <array>

#include "loader.hpp"

/*****************************************************************************/
// points3D_t
/*****************************************************************************/
//class points3D_t {
class points3D_t : public mpl::loader {
  
private:
  
  std::vector< std::array<double, 3> > points;
  
public:
  
  void filler(const mpl::loader::data & arguments) {
    
    std::array<double, 3> point;
    
    point[0] = arguments.get<double>(0);
    point[1] = arguments.get<double>(1);
    point[2] = arguments.get<double>(2);
    
    points.push_back(point);
    
  }
  
  void print(){
    
    for(std::size_t i=0; i<points.size(); ++i)
      printf("%f %f %f\n", points[i][0], points[i][1], points[i][2]);
    
  }
  
};

/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []) {

  points3D_t points;
  
  double min, max;
  
  points.load(argv[1], "3 4 5");
  
  points.minmax(argv[1], 3, min, max);  printf("%e %e\n", min, max);
  
  mpl::loader loader;
  
  loader(argv[1], "3 4 5", points, &points3D_t::filler);
 
  loader.minmax(argv[1], 3, min, max);  printf("%e %e\n", min, max);
  
  points.print();
  
  return EXIT_SUCCESS;
  
}
