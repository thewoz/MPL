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


#ifndef _H_MPL_RK4_H_
#define _H_MPL_RK4_H_

#include <cstdlib>
#include <cstdio>

#include <vector>
#include <array>

/*****************************************************************************/
// namespace mpl::integration
/*****************************************************************************/
namespace mpl::integration {
    
    //****************************************************************************
    //
    //  Purpose:
    //
    //    RK4VEC takes one Runge-Kutta step for a vector ODE.
    //
    //  Discussion:
    //
    //    It is assumed that an initial value problem, of the form
    //
    //      du/dt = f (t, u)
    //      u(t0) = u0
    //
    //    is being solved.
    //
    //    If the user can supply current values of t, u, a stepsize dt, and a
    //    function to evaluate the derivative, this function can compute the
    //    fourth-order Runge Kutta estimate to the solution at time t+dt.
    //
    //  Licensing:
    //
    //    This code is distributed under the GNU LGPL license.
    //
    //  Modified:
    //
    //    09 October 2013
    //
    //  Author:
    //
    //    John Burkardt
    //
    //  Parameters:
    //
    //    Input:
    //
    //      t0, the current time.
    //      u0, the solution estimate at the current time.
    //      dt, the time step.
    //
    //     double * F ( double T, int M, double U[] )
    //      a function which evaluates the derivative, or right hand side of the problem.
    //
    //    Output: the fourth-order Runge-Kutta solution estimate at time t0+dt.
    //
    
    template<size_t N>
    /*****************************************************************************/
    // RK4 function via double *
    /*****************************************************************************/
    void rk4(const double * u0, double dt, double * ut, void f(const double i[N], double o[N])) {
      
      double u1[N];
      double u2[N];
      double u3[N];
      
      double k1[N];
      double k2[N];
      double k3[N];
      double k4[N];
      
      //  Get four sample values of the derivative.
      f(u0, k1);
      
      for(int i=0; i<N; ++i)
        u1[i] = u0[i] + dt * k1[i] / 2.0;
      
      f(u1, k2);
      
      for(int i=0; i<N; ++i)
        u2[i] = u0[i] + dt * k2[i] / 2.0;
      
      f(u2, k3);
      
      for(int i=0; i<N; ++i)
        u3[i] = u0[i] + dt * k3[i];
      
      f(u3, k4);
      
      //  Combine them to estimate the solution.
      for(int i=0; i<N; ++i)
        ut[i] = u0[i] + (dt/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
            
    }
    
//#if(0)
//
//    /*****************************************************************************/
//    // RK4 function via double *
//    /*****************************************************************************/
//    template <typename T>
//    const double * rk4(double t0, const double * u0, size_t m, double dt, const double * f(double t, size_t m, const double * u)) {
//
//      double * u = new double[m];
//
//      double * u1 = new double[m];
//      double * u2 = new double[m];
//      double * u3 = new double[m];
//
//      double * k1 = new double[m];
//      double * k2 = new double[m];
//      double * k3 = new double[m];
//      double * k4 = new double[m];
//
//      //  Get four sample values of the derivative.
//      k1 = f(t0, u0);
//
//      for(int i=0; i<m; ++i)
//        u1[i] = u0[i] + dt * k1[i] / 2.0;
//
//      k2 = f((t0 + dt / 2.0), u1);
//
//      for(int i=0; i<m; ++i)
//        u2[i] = u0[i] + dt * k2[i] / 2.0;
//
//      k3 = f((t0 + dt / 2.0), u2);
//
//      for(int i=0; i<m; ++i)
//        u3[i] = u0[i] + dt * k3[i];
//
//      k4 = f((t0 + dt), u3);
//
//      //  Combine them to estimate the solution.
//      for(int i=0; i<m; ++i)
//        u[i] = u0[i] + (dt/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
//
//      //  Free memory.
//      delete [] u1;
//      delete [] u2;
//      delete [] u3;
//
//      delete [] k1;
//      delete [] k2;
//      delete [] k3;
//      delete [] k4;
//
//      return u;
//
//    }
//
//    /*****************************************************************************/
//    // RK4 function via std::vector
//    /*****************************************************************************/
//    template <typename T>
//    std::vector<double> rk4(double t0, const std::vector<T> & u0, double dt, const std::vector<double> f(double t, const std::vector<T> & u)) {
//
//      // the spatial dimension
//      double m = u0.size();
//
//      std::vector<double> u(m);
//
//      std::vector<double> u1(m);
//      std::vector<double> u2(m);
//      std::vector<double> u3(m);
//
//      std::vector<double> k1(m);
//      std::vector<double> k2(m);
//      std::vector<double> k3(m);
//      std::vector<double> k4(m);
//
//      //  Get four sample values of the derivative.
//      k1 = f(t0, u0);
//
//      for(int i=0; i<m; ++i)
//        u1[i] = u0[i] + dt * k1[i] / 2.0;
//
//      k2 = f((t0 + dt / 2.0), u1);
//
//      for(int i=0; i<m; ++i)
//        u2[i] = u0[i] + dt * k2[i] / 2.0;
//
//      k3 = f((t0 + dt / 2.0), u2);
//
//      for(int i=0; i<m; ++i)
//        u3[i] = u0[i] + dt * k3[i];
//
//      k4 = f((t0 + dt), u3);
//
//      //  Combine them to estimate the solution.
//      for(int i=0; i<m; ++i)
//        u[i] = u0[i] + (dt/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
//
//      return u;
//
//    }
//
//    /*****************************************************************************/
//    // RK4 function via std::array
//    /*****************************************************************************/
//    template <typename T, std::size_t N>
//    std::array<double,N> rk4(double t0, const std::array<T,N> & u0, double dt, const std::array<double,N> f(double t, const std::array<T,N> & u)) {
//
//      std::array<double,N> u;
//
//      std::array<double,N> u1;
//      std::array<double,N> u2;
//      std::array<double,N> u3;
//
//      std::array<double,N> k1;
//      std::array<double,N> k2;
//      std::array<double,N> k3;
//      std::array<double,N> k4;
//
//      //  Get four sample values of the derivative.
//      k1 = f(t0, u0);
//
//      for(int i=0; i<N; ++i)
//        u1[i] = u0[i] + dt * k1[i] / 2.0;
//
//      k2 = f((t0 + dt / 2.0), u1);
//
//      for(int i=0; i<N; ++i)
//        u2[i] = u0[i] + dt * k2[i] / 2.0;
//
//      k3 = f((t0 + dt / 2.0), u2);
//
//      for(int i=0; i<N; ++i)
//        u3[i] = u0[i] + dt * k3[i];
//
//      k4 = f((t0 + dt), u3);
//
//      //  Combine them to estimate the solution.
//      for(int i=0; i<N; ++i)
//        u[i] = u0[i] + (dt/6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
//
//      return u;
//
//    }
//
//#endif
//
} /* namespace mpl::integration */

#endif /* _H_MPL_RK4_H_ */
