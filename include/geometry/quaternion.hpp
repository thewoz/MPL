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
      
      quaternion_t() { }
      
      // NOTE: The constructor is like the glm::quat
      quaternion_t(double _q4, double _q1, double _q2, double _q3) { q4 = _q4;  q1 = _q1;  q2 = _q2; q3 = _q3; }
      
      //NOTE: (pitch, yaw, roll) => (x,y,z)
      quaternion_t(double pitch, double yaw, double roll) {
        
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        
        x = sp * cy * cr - cp * sy * sr;
        y = cp * sy * cr + sp * cy * sr;
        z = cp * cy * sr - sp * sy * cr;
        w = cp * cy * cr + sp * sy * sr;

        //printf("mpl q %f %f %f %f\n", x, y,z,w);
        
      }
      
      struct { double w, x, y, z; };
      struct { double q4, q1, q2, q3; };
      
      double data[4];
      
      double   operator [] (size_t index) const { return data[index]; }
      double & operator [] (size_t index)       { return data[index]; }
      
      void println(FILE * output = stdout) const { fprintf(output, "%f %f %f %f\n", q4, q1, q2, q3); }
      void print  (FILE * output = stdout) const { fprintf(output, "%f %f %f %f",   q4, q1, q2, q3); }
      
      bool check() const { return (fabs((q4*q4)+(q1*q1)+(q2*q2)+(q3*q3)-1) > 1.0e-08) ? true : false; }
      
      /*****************************************************************************/
      // Rotation
      /*****************************************************************************/
      // In homogeneous expression - order XYZ
      void getRotationMatrix(double matrix[3][3]) const {
        
        double sqx = x*x;
        double sqy = y*y;
        double sqz = z*z;
        double sqw = w*w;

        // invs (inverse square length) is only required if quaternion is not already normalised
        double invs = 1 / (sqx + sqy + sqz + sqw);
        
        matrix[0][0] = ( sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
        matrix[1][1] = (-sqx + sqy - sqz + sqw)*invs;
        matrix[2][2] = (-sqx - sqy + sqz + sqw)*invs;
        
        double tmp1 = x*y;
        double tmp2 = z*w;
        
        matrix[0][1] = 2.0 * (tmp1 + tmp2)*invs;
        matrix[1][0] = 2.0 * (tmp1 - tmp2)*invs;
        
        tmp1 = x*z;
        tmp2 = y*w;
        
        matrix[0][2] = 2.0 * (tmp1 - tmp2)*invs;
        matrix[2][0] = 2.0 * (tmp1 + tmp2)*invs;
        
        tmp1 = y*z;
        tmp2 = x*w;
        
        matrix[1][2] = 2.0 * (tmp1 + tmp2)*invs;
        matrix[2][1] = 2.0 * (tmp1 - tmp2)*invs;
        
      }
      
      /*****************************************************************************/
      // getAngles
      /*****************************************************************************/
      template <typename T>
      void getAngles(T & pitch, T & yaw, T & roll) const {
        
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





