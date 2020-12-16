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

#include <mpl/interpolation/lagrange.hpp>

/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []) {
  
  std::vector<double> x(10);
  std::vector<double> y(10);
  
  for(int i=0; i<10; ++i){ x[i] = i; y[i] = sin(i); printf("%f %f\n", x[i], y[i]); }
  
  printf("\n");
  
  std::vector<double> coeffs = mpl::interpolation::lagrange::findCoefficients(x, y);
  
  for(int i=0; i<coeffs.size(); ++i){ printf("%.15f, ", coeffs[i]); }
  
  return 0;
  
}

