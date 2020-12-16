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

#ifndef _H_MPL_ALGORITHM_H_
#define _H_MPL_ALGORITHM_H_

#include <cstdlib>
#include <cstdio>

#include "params.hpp"


/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {

  /*****************************************************************************/
  // algorithm_t
  /*****************************************************************************/
  class algorithm_t {
    
  protected:
    
    //std::vector<algorithm_t *> modules;
    //std::vector<size_t> modules_order;
    
    //std::map<std::string, param_t *> inputs;
    //std::map<std::string, param_t *> outputs;
    
    mpl::params_t inputs;
    mpl::params_t outputs;

    /*****************************************************************************/
    // algorithm_t
    /*****************************************************************************/
    algorithm_t() { /*modules.push_back(this); modules_order.push_back(0);*/ }
    
    /*****************************************************************************/
    // ~algorithm_t
    /*****************************************************************************/
    ~algorithm_t() {
      
      // XXX distruggi map
      
    }
    
  public:
    
    // Nome dell'algoritmo
    std::string name;
    
    // Funzione di esecuzione
    virtual void run() = 0;
    
    // Funzione di inizializazione
    virtual void init() = 0;
    
    template <class T>
    T & input(const std::string & key) {
      
      return inputs.get<T>(key);
      
      
    }
    
  protected:
    
    // funzione che controlla se gli input sono corretti
    virtual void checkInput() = 0;

    
  };
  
  
} /* namespace mpl */

#endif /* _H_MPL_ALGORITHM_H_ */
