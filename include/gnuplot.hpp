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
