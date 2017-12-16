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

#ifndef _H_MPL_QUATERNION_H_
#define _H_MPL_QUATERNION_H_

#include <cstdlib>
#include <cstdio>

#include <cmath>

/*****************************************************************************/
// namespace mpl::geometry
/*****************************************************************************/
namespace mpl::geometry {
    
    /*****************************************************************************/
    // union quaternion_t
    /*****************************************************************************/
    union quaternion_t {
      
      // q = q4 + iq1 + jq2 + kq3;
      // q4^2 + q1^2 + q2^2 + q3^2 = 1
      // [q4 q1 q2 q3] = [qw qx qy qz]
      
      quaternion_t() { }
      
      quaternion_t(double _q1, double _q2, double _q3, double _q4) { q1 = _q1;  q2 = _q2;  q3 = _q3; q4 = _q4; }
      
      quaternion_t(double pitch, double yaw, double roll) {
        
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        
        x = sp * cy * cr + cp * sy * sr;
        y = cp * sy * cr - sp * cy * sr;
        z = cp * cy * sr + sp * sy * cr;
        w = cp * cy * cr - sp * sy * sr;
        
      }
      
      struct { double x, y, z, w; };
      struct { double q1, q2, q3, q4; };
      double data[4];
      
      double   operator [] (size_t index) const { return data[index]; }
      double & operator [] (size_t index)       { return data[index]; }
      
      void println(FILE * output = stdout) const { fprintf(output, "%f %f %f %f\n", q1, q2, q3, q4); }
      void print  (FILE * output = stdout) const { fprintf(output, "%f %f %f %f",   q1, q2, q3, q4); }
      
      bool check() const { return (fabs((q4*q4)+(q1*q1)+(q2*q2)+(q3*q3)-1) > 1.0e-08) ? true : false; }
      
      /*****************************************************************************/
      // Rotation
      /*****************************************************************************/
      // In homogeneous expression - order XYZ
      void getRotationMAtrix(double matrix[3][3]) const {
        
        double sqw = w*w;
        double sqx = x*x;
        double sqy = y*y;
        double sqz = z*z;
        
        // invs (inverse square length) is only required if quaternion is not already normalised
        double invs = 1 / (sqx + sqy + sqz + sqw);
        
        matrix[0][0] = ( sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
        matrix[1][1] = (-sqx + sqy - sqz + sqw)*invs;
        matrix[2][2] = (-sqx - sqy + sqz + sqw)*invs;
        
        double tmp1 = x*y;
        double tmp2 = z*w;
        
        matrix[1][0] = 2.0 * (tmp1 + tmp2)*invs;
        matrix[0][1] = 2.0 * (tmp1 - tmp2)*invs;
        
        tmp1 = x*z;
        tmp2 = y*w;
        
        matrix[2][0] = 2.0 * (tmp1 - tmp2)*invs;
        matrix[0][2] = 2.0 * (tmp1 + tmp2)*invs;
        
        tmp1 = y*z;
        tmp2 = x*w;
        
        matrix[2][1] = 2.0 * (tmp1 + tmp2)*invs;
        matrix[1][2] = 2.0 * (tmp1 - tmp2)*invs;
        
      }
      
      /*****************************************************************************/
      // getAngles
      /*****************************************************************************/
      void getAngles(double & pitch, double & yaw, double & roll) const {
        
        // pitch (x-axis rotation)
        double sinr = +2.0 * (w * x + y * z);
        double cosr = +1.0 - 2.0 * (x * x + y * y);
        pitch = atan2(sinr, cosr);
        
        // yaw (y-axis rotation)
        double sinp = +2.0 * (w * y - z * x);
        if(fabs(sinp) >= 1)
          yaw = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
          yaw = asin(sinp);
        
        // roll (z-axis rotation)
        double siny = +2.0 * (w * z + x * y);
        double cosy = +1.0 - 2.0 * (y * y + z * z);
        roll = atan2(siny, cosy);
        
      }
      
    };

  
} /* namespace mpl::geometry */

#endif /* _H_MPL_QUATERNION_H_ */





