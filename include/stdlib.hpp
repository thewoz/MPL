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

#ifndef _H_MPL_STDLIB_H_
#define _H_MPL_STDLIB_H_

#include <cstdlib>
#include <cstdio>

#include <cmath>

#include <climits>

#include <vector>
#include <string>

#include <algorithm>

/*****************************************************************************/
// std
/*****************************************************************************/
namespace std {

  /*****************************************************************************/
  // itoa
  /*****************************************************************************/
  char * itoa(int value, char * result, int base = 10) {
    
    // check that the base if valid
    if(base < 2 || base > 36) {
      *result = '\0';
      return result;
    }
    
    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;
    
    do {
      tmp_value = value;
      value /= base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while (value);
    
    // Apply negative sign
    if(tmp_value < 0) *ptr++ = '-';
    
    *ptr-- = '\0';
    
    while (ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr-- = *ptr1;
      *ptr1++ = tmp_char;
    }
    
    return result;
    
  }

  /*****************************************************************************/
  // itoa
  /*****************************************************************************/
  char * itoa(int value, int base = 10) {
   
    static char str[LINE_MAX];
    
    return itoa(value, str, base);
    
  }
  
  /*****************************************************************************/
  // ceilToSignificantFigures
  /*****************************************************************************/
  double ceilToSignificantFigures(double num, int n = 1) {
    
    if(num == 0) {
      return 0;
    }
    
    double d = ceil(log10(num < 0 ? -num: num));
    
    int power = n - (int) d;
    
    double magnitude = pow(10, power);
    
    long shifted = ceil(num*magnitude);
    
    return shifted/magnitude;
    
  }
  
  
  /*****************************************************************************/
  // parse
  /*****************************************************************************/
  void parse(const std::string & str, const std::string & delimiter, std::vector<std::string> & tokens) {
  
    tokens.clear();
    
    std::size_t last = 0;
    std::size_t next = 0;
    
    while((next = str.find(delimiter, last)) != string::npos) {
      
      tokens.push_back(str.substr(last, next-last));
      
      last = next + 1;
      
      }
    
     tokens.push_back(str.substr(last));
    
    }
  
  /*****************************************************************************/
  // parse
  /*****************************************************************************/
  std::vector<std::string> parse(const std::string & str, const std::string & delimiter) {
    
    std::vector<std::string> tokens;
    
    parse(str, delimiter, tokens);
    
    return tokens;
    
  }

  
  /*****************************************************************************/
  // namespace util_set
  /*****************************************************************************/
  namespace util_set{
    
    struct counter_t {
      struct value_type { template<typename T> value_type(const T&) { } };
      void push_back(const value_type&) { ++count; }
      size_t count = 0;
    };
  
  }
  
  /*****************************************************************************/
  // intersection_size
  /*****************************************************************************/
  template<typename T1, typename T2>
  size_t intersection_size(const T1& s1, const T2& s2) {
    util_set::counter_t c;
    std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(c));
    return c.count;
  }
  
  /*****************************************************************************/
  // difference_size
  /*****************************************************************************/
  template<typename T1, typename T2>
  size_t difference_size(const T1& s1, const T2& s2) {
    util_set::counter_t c;
    std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(c));
    return c.count;
  }

  
} /* namespace std */


#endif /* _H_MPL_STDLIB_H_ */
