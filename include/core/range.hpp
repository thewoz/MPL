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

#ifndef _H_MPL_RANGE_H_
#define _H_MPL_RANGE_H_

#include <cstdio>
#include <cstdlib>

//****************************************************************************
// namespace mpl
//****************************************************************************
namespace mpl {

  //****************************************************************************
  // class range_t
  //****************************************************************************
  template <class T>
  class range_t {

  public:
    
    T start;
    T end;
        
    range_t() {
      start = std::numeric_limits<T>::min();
      end   = std::numeric_limits<T>::max();
    }
    
    range_t(T _start, T _end = std::numeric_limits<T>::max()) : start(_start), end(_end) { }
    
    T length() const { return (end - start + 1); }
    
    template <class V>
    bool in(V value) const { return (value<=end && value>=start); }
    
    static T inf() { return std::numeric_limits<T>::max(); }

//    //****************************************************************************
//    // operator <
//    //****************************************************************************
//    bool operator < (const range_t & arg) const { return (start<arg.start); }
//    
//    bool operator == (const range_t & arg) const { return ((range_t == match.father) && (child == match.child)); }

  };
  
} /* namespace mpl */
  
#endif /* _H_MPL_RANGE_H_ */
