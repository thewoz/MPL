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

#ifndef _H_COBBS_OPENGL_H_
#define _H_COBBS_OPENGL_H_

#include <cstdlib>
#include <cstdio>

#include <opencv2/opencv.hpp>

#ifdef __APPLE__
  #include <OpenGL/gl.h>
#else
  #include <GL/gl.h>
#endif


/*****************************************************************************/
// opengl
/*****************************************************************************/
namespace opengl {
  
  
  /*****************************************************************************/
  // glOrto
  /*****************************************************************************/
  template <typename T>
  inline cv::Mat glOrto(T left, T right, T bottom, T top, T znear, T zfar) {
    
    assert(left   != right);
    assert(bottom != top);
    assert(znear  != zfar);
    
    const T p_fn = zfar + znear;
    const T m_nf = znear - zfar; // ~ -m_fn
    
    const T p_rl = right + left;
    const T m_rl = right - left;
    const T p_tb = top + bottom;
    const T m_tb = top - bottom;
    
    const T m_lr = -m_rl;
    const T m_bt = -m_tb;
    
    return ( cv::Mat_<T>(4,4) << T(2)/m_rl,           T(0),           T(0),      p_rl/m_lr,
                                      T(0),      T(2)/m_tb,           T(0),      p_tb/m_bt,
                                      T(0),           T(0),      T(2)/m_nf,      p_fn/m_nf,
                                      T(0),           T(0),           T(0),          T(1));
    
  }
  
  
  /*****************************************************************************/
  // glFrustum
  /*****************************************************************************/
  template <typename T>
  inline cv::Mat glFrustum(T left, T right, T bottom, T top, T znear, T zfar) {
    
    assert(znear > T(0));
    assert(zfar  > T(0));
    assert(left   != right);
    assert(bottom != top);
    assert(znear  != zfar);
    
    const T x_2n = znear + znear;
    const T x_2nf = T(2) * znear * zfar;
    
    const T p_fn = zfar + znear;
    const T m_nf = znear - zfar; // ~ -m_fn
    
    const T p_rl = right + left;
    const T m_rl = right - left;
    const T p_tb = top + bottom;
    const T m_tb = top - bottom;
   
    return ( cv::Mat_<float>(4,4) << x_2n/m_rl,          T(0),     p_rl/m_rl,           T(0),
                                      T(0),     x_2n/m_tb,     p_tb/m_tb,           T(0),
                                      T(0),          T(0),     p_fn/m_nf,     x_2nf/m_nf,
                                      T(0),          T(0),         T(-1),           T(0));
    
  }
  
   
  /*****************************************************************************/
  // gluPerspective
  /*****************************************************************************/
  template <typename T>
  inline cv::Mat gluPerspective(T fovy_deg, T aspect, T znear, T zfar) {
    
    assert(znear > T(0));
    assert(zfar  > T(0));
    assert(aspect != T(0));
    assert(znear  != zfar);
    
    const T half_fovy_rad = T(M_PI) * fovy_deg / T(360);
    
    const T si = sin(half_fovy_rad);
    const T co = cos(half_fovy_rad);
    
    assert(si != T(0));
    
    const T c = co / si; // cotangent
    const T a = aspect;
    
    const T x_2nf = T(2) * znear * zfar;
    const T p_fn = zfar + znear;
    const T m_nf = znear - zfar;
    
    return ( cv::Mat_<T>(4,4) << c/a,    T(0),         T(0),          T(0),
                                T(0),       c,         T(0),          T(0),
                                T(0),    T(0),    p_fn/m_nf,    x_2nf/m_nf,
                                T(0),    T(0),        T(-1),          T(0));
    
  }
  
  
} /* namespace opengl */


#endif /* _H_COBBS_OPENGL_H_ */

