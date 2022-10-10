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


#ifndef _H_MPL_STAT_H_
#define _H_MPL_STAT_H_

#include <cstdlib>
#include <cstdio>

#include <cmath>

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // class stat
  /*****************************************************************************/
  class stat {
    
  public:
    
    stat() : m_n(0) { }
    
    void clear() {
      m_n = 0;
    }
    
    inline void add(double x) {
      
      m_n++;
      
      // See Knuth TAOCP vol 2, 3rd edition, page 232
      if (m_n == 1) {
        m_oldM = m_newM = x;
        m_oldS = 0.0;
      } else {
        m_newM = m_oldM + (x - m_oldM)/m_n;
        m_newS = m_oldS + (x - m_oldM)*(x - m_newM);
        
        // set up for next iteration
        m_oldM = m_newM;
        m_oldS = m_newS;
      }
    }
    
    int size() const {
      return m_n;
    }
    
    double mean() const {
      return (m_n > 0) ? m_newM : 0.0;
    }
    
    double variance() const {
      return ( (m_n > 1) ? m_newS/(m_n - 1) : 0.0 );
    }
    
    double std() const {
      return std::sqrt(variance());
    }
    
  private:
    
    int m_n;
    double m_oldM, m_newM, m_oldS, m_newS;
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_STAT_H_ */
