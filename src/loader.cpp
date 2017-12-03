/*
 * MIT License
 *
 * Copyright © 2017
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
  
  return 0;
  
}
