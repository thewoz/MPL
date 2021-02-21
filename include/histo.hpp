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
#include <iostream>

//****************************************************************************
// namespace mpl
//****************************************************************************
namespace mpl {

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
