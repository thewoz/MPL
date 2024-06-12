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

#ifndef _H_MPL_STDLIB_H_
#define _H_MPL_STDLIB_H_

#include <cstdlib>
#include <cstdio>

#include <cmath>

#include <climits>

#include <vector>
#include <string>

#include <algorithm>

//*****************************************************************************/
// std
//*****************************************************************************/
namespace std {

  //*****************************************************************************/
  // itoa
  //*****************************************************************************/
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

  //*****************************************************************************/
  // itoa
  //*****************************************************************************/
  char * itoa(int value, int base = 10) {
   
    static char str[LINE_MAX];
    
    return itoa(value, str, base);
    
  }

  //*****************************************************************************/
  // decToHex():
  // function to convert decimal to hexadecimal
  //*****************************************************************************/
  std::string decToHex(int n) {
    
    // char array to store hexadecimal number
    char hexaDeciNum[2];
   
    // counter for hexadecimal number array
    int i = 0;
    
    while (n != 0) {
   
      // temporary variable to store remainder
      int temp = 0;
   
      // storing remainder in temp variable.
      temp = n % 16;
   
      // check if temp < 10
      if(temp < 10) {
        hexaDeciNum[i] = temp + 48;
        i++;
      } else {
        hexaDeciNum[i] = temp + 55;
        i++;
      }
   
      n = n / 16;
      
    }
   
    std::string hexCode = "";

    if(i == 2) {
      hexCode.push_back(hexaDeciNum[1]);
      hexCode.push_back(hexaDeciNum[0]);
    } else if (i == 1) {
      hexCode.push_back(48);
      hexCode.push_back(hexaDeciNum[0]);
    } else if (i == 0)
      hexCode = "00";
   
    // Return the equivalent hexadecimal color code
    return hexCode;
    
  }
  
  //*****************************************************************************/
  // ceilToSignificantFigures
  //*****************************************************************************/
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
  
  
  //*****************************************************************************/
  // parse
  //*****************************************************************************/
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
  
  //*****************************************************************************/
  // parse
  //*****************************************************************************/
  std::vector<std::string> parse(const std::string & str, const std::string & delimiter) {
    
    std::vector<std::string> tokens;
    
    parse(str, delimiter, tokens);
    
    return tokens;
    
  }
  
  //*****************************************************************************/
  // namespace util_set
  //*****************************************************************************/
  namespace util_set{
    
    struct counter_t {
      struct value_type { template<typename T> value_type(const T&) { } };
      void push_back(const value_type&) { ++count; }
      size_t count = 0;
    };
  
  }
  
  //*****************************************************************************/
  // intersection
  //*****************************************************************************/
  template<typename T1, typename T2>
  size_t intersection(const T1 & s1, const T2 & s2) {
    util_set::counter_t c;
    std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(c));
    return c.count;
  }

  //*****************************************************************************/
  // difference
  //*****************************************************************************/
  template<typename T1, typename T2>
  size_t difference(const T1 & s1, const T2 & s2) {
    util_set::counter_t c;
    std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(), std::back_inserter(c));
    return c.count;
  }

  //*****************************************************************************/
  // equal
  //*****************************************************************************/
  template<typename T1, typename T2>
  bool equal(const T1& s1, const T2& s2) {
    
    if(s1.size() != s2.size()) return false;
    
    size_t size = intersection(s1, s2);
    
    if(size == s1.size()) return true;
    else return false;
    
  }

  //*****************************************************************************/
  // intersection
  //*****************************************************************************/
  template<typename T>
  void intersection(const std::vector<T> & s1, const std::vector<T> & s2, std::vector<T> & r) {
    r.resize(s1.size()+s2.size());
    auto it = std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), r.begin());
    r.resize(it-r.begin());
   }

  template<typename T>
  void intersection(const std::set<T> & s1, const std::set<T> & s2, std::set<T> & r) {
    r.clear();
    std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), std::inserter(r, r.begin()));
   }
  
  //*****************************************************************************/
  // difference
  //*****************************************************************************/
  template<typename T1, typename T2>
  size_t difference(const T1& s1, const T2& s2, T1 & r) {
    r.resize(s1.size()+s2.size());
    auto it = std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(), r.begin());
    r.resize(it-r.begin());
  }
  
} /* namespace std */


#endif /* _H_MPL_STDLIB_H_ */
