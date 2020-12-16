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
#include <cmath>

#include "gnuplot.hpp"

struct point_t {
  
  double x;
  double y;
  
};


/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []){
  
  mpl::gnuplot gp;
    
  gp.setTitle("ceppa");
  
  //gp.sendLine("plot [-pi/2:pi] cos(x),-(sin(x) > sin(x+1) ? sin(x) : sin(x+1))");
  
  std::vector<point_t> points(100);
  
  for(size_t i=0; i<points.size(); ++i) {
    points[i].x = i;
    points[i].y = sin(i);
  }
  
  gp.plot(points);


  return 0;
  
}
