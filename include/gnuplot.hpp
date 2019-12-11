/*
 * MIT License
 *
 * Copyright © 2017 COBBS
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

#ifndef _H_MPL_GNUPLOT_H_
#define _H_MPL_GNUPLOT_H_

#include <cstdlib>
#include <cstdio>

#include <string>
#include <vector>

#include <iostream>

#include <mpl/stdlib.hpp>

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {

  /*****************************************************************************/
  // class gnuplot
  /*****************************************************************************/
  class gnuplot {
    
  private:
    
    // Uccido l'operatore copia
    void operator = (gnuplot const & ) = delete;
    
    // pipe per comunicare con gnuplot
    FILE * pipe = NULL;
    
  public:
    
    /*****************************************************************************/
    // gnuplot
    /*****************************************************************************/
    gnuplot() { }
    
    /*****************************************************************************/
    // ~gnuplot
    /*****************************************************************************/
    ~gnuplot(){
      if(pipe) pclose(pipe);
    }
    
    /*****************************************************************************/
    // sendLine() -
    /*****************************************************************************/
    void sendLine(const std::string & text) {
      
      if(!pipe) return;
  
      fputs((text + "\n").c_str(), pipe);
      
    }
    
    void setTitle(const std::string & title) { fputs(("set title \"" + title + "\"\n").c_str(), pipe); }
    
    void setGrid() { fputs("set grid\n", pipe); }
      
    void setXLabel(const std::string & label) { fputs(("set xlabel \"" + label + "\"\n").c_str(), pipe); }
    void setYLabel(const std::string & label) { fputs(("set ylabel \"" + label + "\"\n").c_str(), pipe); }
    
    void setLogX() { fputs("set log x\n", pipe); }
    void setLogY() { fputs("set log y\n", pipe); }

    void unsetLogX() { fputs("unset log x\n", pipe); }
    void unsetLogY() { fputs("unset log y\n", pipe); }

    template <class T>
    void plot(std::vector<T> & points) {
      
      open();
      
      for(size_t i=0; i<points.size(); ++i)
        fprintf(pipe, "%e %e\n", points[i].x, points[i].y);
      
      close();

    }
    
    template <class T>
    inline void add(const T & point) { fprintf(pipe, "%e %e\n", point.x, point.y); }
    
    inline void add(double & x, double & y) { fprintf(pipe, "%e %e\n", x, y); }

    inline void init(bool persist = true) {
        
        pipe = popen(persist ? "gnuplot -persist" : "gnuplot", "w");
        
        if(!pipe) {
          fprintf(stderr, "Error in opening the pipe in mpl::gnuplot()");
          abort();
        }
              
    }
    
    
    inline void open() { fputs("plot '-' w p\n", pipe); }
    
    inline void close() { fputs("e\n", pipe); }
    
    inline void save(std::string name) {
      
      mpl::io::expandPath(name);
      
      fputs("set term png\n", pipe);
      fprintf(pipe, "set output \"%s\"\n", name.c_str());
      fputs("rep\n", pipe);
      fputs("set output\n", pipe);

      
    }

    
    
  };
  
} /* namespace mpl::math */


#endif /* _H_MPL_GNUPLOT_H_ */
