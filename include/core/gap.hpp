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

#ifndef _H_MPL_GAP_H_
#define _H_MPL_GAP_H_

#include <opencv2/opencv.hpp>

//****************************************************************************
// namespace mpl
//****************************************************************************
namespace mpl {

  //****************************************************************************
  // class gap_t
  //****************************************************************************
  class gap_t : public cv::Range {

  public:
    
    int camera;
    
    gap_t() : cv::Range(-1,-1) { }
    
    gap_t(int _start, int _end, int _camera = -1) : cv::Range(_start,_end), camera(_camera) { }
    
    int length() const { return (end - start + 1); }
    
  };
  
} /* namespace mpl */
  
#endif /* _H_MPL_GAP_H_ */
