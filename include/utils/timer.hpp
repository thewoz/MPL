/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2025
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

#ifndef _H_MPL_TIMER_H_
#define _H_MPL_TIMER_H_

#include <cstdio>
#include <cstdlib>

#include <chrono>

//****************************************************************************
// namespace mpl
//****************************************************************************
namespace mpl {

//****************************************************************************/
// Timer()
//****************************************************************************/
template <class DT = std::chrono::milliseconds, class ClockT = std::chrono::steady_clock>

class timer_t {
    
  using timep_t = typename ClockT::time_point;
    
  timep_t _start = ClockT::now(), _end = { };

public:
  
  void tick() {
    _end = timep_t{};
    _start = ClockT::now();
  }
    
  void tock() { _end = ClockT::now(); }
    
  template <class T = DT>
  auto duration() const {
    return std::chrono::duration_cast<T>(_end - _start);
  }
  
};
  
} /* namespace mpl */
  
#endif /* _H_MPL_TIMER_H_ */
