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

#ifndef _H_MPL_HISTO_H_
#define _H_MPL_HISTO_H_

#include <cstdio>
#include <cstdlib>

#include <vector>
#include <map>

#include <iostream>

#include <mpl/stdio.hpp>

//****************************************************************************
// namespace mpl
//****************************************************************************
namespace mpl {

  //****************************************************************************
  // histoInt_t
  //****************************************************************************
  class histoInt_t {
    
  private:
    
    struct data_t {
      
      int rangeMin;
      int rangeMax;
      double rangePer;
      
      int counter;
      double counterPer;

      data_t() : rangeMin(INT_MAX), rangeMax(INT_MIN), counter(0) { }
      
    };
    
    
  public:
    
    std::map<int, int> data;
    
    std::vector<data_t> histo;

    histoInt_t() { }
    
    void add(int value) { data[value]++; }
    
    void clear() { data.clear(); }
    
    //****************************************************************************
    // binning
    //****************************************************************************
    void binning(size_t binNum, size_t norm) {
      
      if(data.size() < binNum) {
        binNum = data.size();
      }
      
      int bins = int(data.size() / (int) binNum);
      
      //printf("%d %lu %lu\n", bins, data.size(), binNum);
      
      histo.resize(binNum);
      
      int counter = 0;
      
      int index = 0;
      
      int totalConter = 0;
      
      for(auto iter=data.begin(); iter!=data.end(); iter++) {
       
        // mi segno da dove inzia il bin
        if(counter == 0) histo[index].rangeMin = iter->first;

        // mi segno
        histo[index].counter += iter->second;
                
        totalConter += iter->second;
        
        if(++counter >= bins) {
          
          histo[index].rangeMax = iter->first;
          
          if(index+1 < histo.size()) {
            ++index;
            counter = 0;
          }
          
        }
        
      }
      
      for(int i=0; i<histo.size(); ++i) {
        histo[i].counterPer = histo[i].counter / (double)totalConter;
        histo[i].rangePer   = ((histo[i].rangeMin + histo[i].rangeMax) * 0.5) / (double) norm;
      }

    }
    
    //****************************************************************************
    // print
    //****************************************************************************
    void print(const char * format, ...) {
      
      char filename[PATH_MAX];
      
      va_list arg;
      
      va_start (arg, format);
      vsprintf (filename, format, arg);
      va_end (arg);
            
      mpl::io::expandPath(filename);
      
      FILE * output = mpl::io::open(filename, "w");
      
      print(output);
      
      mpl::io::close(output);
      
    }
    
    //****************************************************************************
    // print
    //****************************************************************************
    void print(FILE * output = stdout) {
      
      for(int i=0; i<histo.size(); ++i) {
        fprintf(output, "%d %d (%.2f) - %d (%.2f) \n", histo[i].rangeMin, histo[i].rangeMax,  histo[i].rangePer*100, histo[i].counter, histo[i].counterPer*100);
      }
      
    }
    
  };
  
  
  //****************************************************************************//
  // histo_t
  //****************************************************************************//
  class histo_t {
    
  public:
    
    std::vector<int> histo;
    
    histo_t() { histo.resize(11, 0); }
    
    void add(double value) {
      
      if(value == 1.0) { histo[0]++;
      } else if(value < 1.0 && value > 0.9) { histo[1]++;
      } else if(value <= 0.9 && value > 0.8) { histo[2]++;
      } else if(value <= 0.8 && value > 0.7) { histo[3]++;
      } else if(value <= 0.7 && value > 0.6) { histo[4]++;
      } else if(value <= 0.6 && value > 0.5) { histo[5]++;
      } else if(value <= 0.5 && value > 0.4) { histo[6]++;
      } else if(value <= 0.4 && value > 0.3) { histo[7]++;
      } else if(value <= 0.3 && value > 0.2) { histo[8]++;
      } else if(value <= 0.2 && value > 0.1) { histo[9]++;
      } else if(value <= 0.1 && value >= 0.0) { histo[10]++;
      } else { fprintf(stderr, "error in histo\n"); }
      
    }
    
    void clear() { histo.clear(); histo.resize(11, 0); }
    
    void print(double normVal, std::ostream & out = std::cout) {
      
      out << std::setw(8) << std::right << "100%"
      << std::setw(8) << std::right << "90%"
      << std::setw(8) << std::right << "80%"
      << std::setw(8) << std::right << "70%"
      << std::setw(8) << std::right << "60%"
      << std::setw(8) << std::right << "50%"
      << std::setw(8) << std::right << "40%"
      << std::setw(8) << std::right << "30%"
      << std::setw(8) << std::right << "20%"
      << std::setw(8) << std::right << "10%"
      << std::setw(8) << std::right << "0%" << std::endl;
      
      out << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[0]  / normVal
      << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[1]  / normVal
      << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[2]  / normVal
      << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[3]  / normVal
      << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[4]  / normVal
      << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[5]  / normVal
      << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[6]  / normVal
      << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[7]  / normVal
      << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[8]  / normVal
      << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[9]  / normVal
      << std::setw(8) << std::right << std::fixed << std::setprecision(1) << histo[10] / normVal << std::endl;
      
    }
    
  };
  
} /* namespace mpl */
  
#endif /* _H_MPL_HISTO_H_ */
