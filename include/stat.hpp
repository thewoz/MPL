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
    
    inline void push(double x) {
      
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
