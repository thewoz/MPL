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

#ifndef _H_MPL_LAGRANGE_H_
#define _H_MPL_LAGRANGE_H_

#include <cstdlib>
#include <cmath>

#include <vector>
#include <array>

#include <algorithm>

/*****************************************************************************/
// namespace mpl::interpolation::lagrange
/*****************************************************************************/
namespace mpl::interpolation::lagrange {

  /*****************************************************************************/
  // findCoefficients
  /*****************************************************************************/
  // Generate the polynomials coefficients of the lagrange interpolation
  // polynomial for the given points
  /*****************************************************************************/
  template <typename T>
  std::vector<double> findCoefficients(const std::vector<T> & x, const std::vector<T> & y) {
    
    if(x.size() != y.size()){
      fprintf(stderr, "lagrange error: the inputs data must be of the same lenght\n");
      abort();
    }
        
    std::size_t size = x.size();
    
    std::vector<double> coeff(size, 0);
    
    // ciclo su tutti i punti
    for(std::size_t i=0; i<size; ++i) {
      
      // coefficenti temporanei
      std::vector<double> tmpcoeffs(size, 0);
      
      // Start with a constant polynomial
      tmpcoeffs[0] = y[i];
      
      double prod = 1;
      
      // ciclo su tutti i punti
      for(std::size_t j=0; j<size; ++j) {

        if(i == j) continue;
        
        prod *= x[i] - x[j];
        
        double precedent = 0;
        
        for(auto resptr=tmpcoeffs.begin(); resptr<tmpcoeffs.end(); resptr++) {
          // Compute the new coefficient of X^i based on
          // the old coefficients of X^(i-1) and X^i
          double newres = (*resptr) * (-x[j]) + precedent;
          precedent = *resptr;
          *resptr = newres;
        }
        
      }
      
      std::transform(coeff.begin(), coeff.end(), tmpcoeffs.begin(), coeff.begin(), [=] (double oldcoeff, double add) { return oldcoeff+add/prod; } );
      
    }
    
    //printf("f(x)="); for(int i=0; i<coeff.size()-1; ++i) printf("%f*x**%d+", coeff[i],i); printf("%f*x**%d\n",coeff[coeff.size()-1], coeff.size()-1);
    
    return coeff;
    
  }
  
  /*****************************************************************************/
  // interpolate
  /*****************************************************************************/
  double interpolate(double x, const std::vector<double> & coefficients) {
    
    double y = 0;
    
    for(int i=0; i<coefficients.size(); ++i)
      y += coefficients[i] * pow(0,i);

    return y;
    
  }

} /* namespace lagrange */


#endif /* _H_MPL_LAGRANGE_H_ */
