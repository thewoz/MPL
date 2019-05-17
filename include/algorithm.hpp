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
