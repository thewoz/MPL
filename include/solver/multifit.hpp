/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2019
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

#ifndef _H_MPL_SOLVER_MULTIFIT_H_
#define _H_MPL_SOLVER_MULTIFIT_H_

// https://www.gnu.org/software/gsl/doc/html/nls.html?highlight=multifit#c.gsl_multifit_nlinear_fdf

#include <cstdio>
#include <cstdlib>

#include <vector>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlinear.h>

//****************************************************************************//
// namespace mpl::solver::multifit
//****************************************************************************//
namespace mpl::solver::multifit {
  
    //****************************************************************************//
    // class data_t
    //****************************************************************************//
    class data_t {

      public:

        virtual size_t size() const = 0;

    };

    //****************************************************************************//
    // class params_t
    //****************************************************************************//
    class params_t {
      
    private:
      
      std::vector<double> params;
      
      bool isInited;
      
    public:
      
      params_t(size_t size) : isInited(true) { params.resize(size); }
      
      params_t() : isInited(false) { }
      
      double & operator [] (size_t index) {
        
        if(!isInited) {
          fprintf(stderr, "params_t must be inited");
          abort();
        }
        
        return params[index];
        
        
      }
      
      size_t size() const { return params.size(); }
      
    };

    //****************************************************************************//
    // callback
    //****************************************************************************//
    void callback(const size_t iter, void * data, const gsl_multifit_nlinear_workspace * workspace) {
      
      gsl_vector * params = gsl_multifit_nlinear_position(workspace);
      
      printf("%lu) ", iter);
      
      for(size_t i=0; i<params->size; ++i)
        printf("%f ", gsl_vector_get(params, i));
               
      printf("\n");

    }
  

  
  //****************************************************************************//
  // multifit() -
  //****************************************************************************//
  void solve(multifit::data_t & data, multifit::params_t & params, int (* func_f) (const gsl_vector * x, void * params, gsl_vector * f), int (* func_df) (const gsl_vector * x, void * params, gsl_matrix * df), size_t max_iter = 200, double xtol = 1.0e-8, double gtol = 1.0e-8, double ftol = 1.0e-8, bool logMode = false, bool debugMode = false) {
    
    // Alloco lo spazio per i paremetri
    gsl_vector * x = gsl_vector_alloc(params.size());
    
    /* starting point */
    for(size_t i=0; i<params.size(); ++i) gsl_vector_set(x, i, params[i]);

    gsl_multifit_nlinear_parameters fdf_params = gsl_multifit_nlinear_default_parameters();
    fdf_params.trs = gsl_multifit_nlinear_trs_lm;

    gsl_multifit_nlinear_fdf fdf;

    /* define function to be minimized */
    fdf.f      = func_f;
    fdf.df     = func_df;
    fdf.fvv    = NULL;
    fdf.n      = std::max(params.size(), data.size());
    fdf.p      = params.size();
    fdf.params = &data;
    
    const gsl_multifit_nlinear_type * T = gsl_multifit_nlinear_trust;
      
    gsl_multifit_nlinear_workspace * workspace = gsl_multifit_nlinear_alloc(T, &fdf_params, fdf.n, params.size());
      
    gsl_vector * f = gsl_multifit_nlinear_residual(workspace);

    int info;
    
    double chisq0, chisq;
    //double rcond;

    /* initialize solver */
    gsl_multifit_nlinear_init(x, &fdf, workspace);

    /* store initial cost */
    gsl_blas_ddot(f, f, &chisq0);

    void (*callbackFunc)(const size_t iter, void *params, const gsl_multifit_nlinear_workspace *w) = NULL;
    
    if(debugMode) callbackFunc = multifit::callback;
       
    /* iterate until convergence */
    int status = gsl_multifit_nlinear_driver(max_iter, xtol, gtol, ftol, callbackFunc, NULL, &info, workspace);

    /* store final cost */
    gsl_blas_ddot(f, f, &chisq);

    /* store cond(J(x)) */
    //gsl_multifit_nlinear_rcond(&rcond, work);

    /* print summary */
    if(logMode) {
      
      fprintf(stderr, "Summary:\n");
      fprintf(stderr, "Method type: %s/%s\n",  gsl_multifit_nlinear_name(workspace), gsl_multifit_nlinear_trs_name(workspace));
      fprintf(stderr, "Status: %s\n", gsl_strerror(status));
      fprintf(stderr, "Number of iterations: %zu\n", gsl_multifit_nlinear_niter(workspace));
      fprintf(stderr, "Function evaluations: %zu\n", fdf.nevalf);
      fprintf(stderr, "Jacobian evaluations: %zu\n", fdf.nevaldf);
      if(!status)fprintf(stderr, "Reason for stopping: %s\n",   (info == 1) ? "small step size" : "small gradient");
      fprintf(stderr, "Initial |f(x)| = %g\n",       chisq0);
      fprintf(stderr, "Final   |f(x)| = %g\n",       chisq);
      //fprintf(stderr, "XXX = %g\n",                  1.0 / rcond);

      printf("\n\n");
      
    }
    
    gsl_vector * _params = gsl_multifit_nlinear_position(workspace);
    
    for(size_t i=0; i<params.size(); ++i)
      params[i] = gsl_vector_get(_params, i);
   
    gsl_multifit_nlinear_free(workspace);
      
    gsl_vector_free(x);
    
  }
  
} /* namespace mpl::solver::multifit */

#endif /* _H_MPL_SOLVER_MULTIFIT_H_ */
