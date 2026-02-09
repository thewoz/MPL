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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <vector>

#if defined(MPL_MULTIFIT_USE_GSL)
  #include <gsl/gsl_blas.h>
  #include <gsl/gsl_matrix.h>
  #include <gsl/gsl_multifit_nlinear.h>
  #include <gsl/gsl_vector.h>
#else
  #include <opencv2/opencv.hpp>
#endif

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

#if defined(MPL_MULTIFIT_USE_GSL)

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
  // solve() - GSL backend (define MPL_MULTIFIT_USE_GSL)
  //****************************************************************************//
  void solve(multifit::data_t & data, multifit::params_t & params, int (* func_f) (const gsl_vector * x, void * params, gsl_vector * f), int (* func_df) (const gsl_vector * x, void * params, gsl_matrix * df), size_t max_iter = 200, double xtol = 1.0e-8, double gtol = 1.0e-8, double ftol = 1.0e-8, bool logMode = false, bool debugMode = false) {

    gsl_vector * x = gsl_vector_alloc(params.size());

    for(size_t i=0; i<params.size(); ++i) gsl_vector_set(x, i, params[i]);

    gsl_multifit_nlinear_parameters fdf_params = gsl_multifit_nlinear_default_parameters();
    fdf_params.trs = gsl_multifit_nlinear_trs_lm;

    gsl_multifit_nlinear_fdf fdf;

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

    gsl_multifit_nlinear_init(x, &fdf, workspace);

    gsl_blas_ddot(f, f, &chisq0);

    void (*callbackFunc)(const size_t iter, void *params, const gsl_multifit_nlinear_workspace *w) = NULL;

    if(debugMode) callbackFunc = multifit::callback;

    int status = gsl_multifit_nlinear_driver(max_iter, xtol, gtol, ftol, callbackFunc, NULL, &info, workspace);

    gsl_blas_ddot(f, f, &chisq);

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

      printf("\n\n");

    }

    gsl_vector * _params = gsl_multifit_nlinear_position(workspace);

    for(size_t i=0; i<params.size(); ++i)
      params[i] = gsl_vector_get(_params, i);

    gsl_multifit_nlinear_free(workspace);

    gsl_vector_free(x);

  }

#else

    //****************************************************************************//
    // callback
    //****************************************************************************//
    void callback(const size_t iter, const cv::Mat & params) {
      printf("%lu) ", iter);
      for(int i=0; i<params.rows; ++i)
        printf("%f ", params.at<double>(i, 0));
      printf("\n");
    }



  //****************************************************************************//
  // solve() - OpenCV backend (default)
  //****************************************************************************//
  void solve(multifit::data_t & data, multifit::params_t & params, int (* func_f) (const cv::Mat & x, void * params, cv::Mat & f), int (* func_df) (const cv::Mat & x, void * params, cv::Mat & df), size_t max_iter = 200, double xtol = 1.0e-8, double gtol = 1.0e-8, double ftol = 1.0e-8, bool logMode = false, bool debugMode = false) {

    cv::Mat x(static_cast<int>(params.size()), 1, CV_64F);
    for(size_t i=0; i<params.size(); ++i) x.at<double>(static_cast<int>(i), 0) = params[i];

    cv::Mat f;
    if(func_f(x, &data, f) != 0) {
      fprintf(stderr, "multifit: func_f failed\n");
      abort();
    }

    double chisq0 = cv::norm(f, cv::NORM_L2SQR);
    double chisq = chisq0;
    double lambda = 1.0e-3;
    int info = 0;
    int status = 0;

    size_t iter_count = 0;
    for(size_t iter = 0; iter < max_iter; ++iter) {
      iter_count = iter + 1;
      cv::Mat J;
      if(func_df(x, &data, J) != 0) {
        fprintf(stderr, "multifit: func_df failed\n");
        abort();
      }

      cv::Mat JtJ = J.t() * J;
      cv::Mat Jtr = J.t() * f;
      cv::Mat A = JtJ + lambda * cv::Mat::eye(JtJ.rows, JtJ.cols, JtJ.type());
      cv::Mat delta;
      if(!cv::solve(A, -Jtr, delta, cv::DECOMP_CHOLESKY)) {
        if(!cv::solve(A, -Jtr, delta, cv::DECOMP_SVD)) {
          fprintf(stderr, "multifit: unable to solve linear system\n");
          status = -1;
          break;
        }
      }

      double step_norm = cv::norm(delta);
      double grad_norm = cv::norm(Jtr);
      if(step_norm < xtol) { info = 1; status = 0; break; }
      if(grad_norm < gtol) { info = 2; status = 0; break; }

      cv::Mat x_candidate = x + delta;
      cv::Mat f_candidate;
      if(func_f(x_candidate, &data, f_candidate) != 0) {
        fprintf(stderr, "multifit: func_f failed\n");
        abort();
      }

      double chisq_candidate = cv::norm(f_candidate, cv::NORM_L2SQR);
      if(std::abs(chisq - chisq_candidate) < ftol) { info = 3; x = x_candidate; f = f_candidate; chisq = chisq_candidate; status = 0; break; }

      if(chisq_candidate < chisq) {
        x = x_candidate;
        f = f_candidate;
        chisq = chisq_candidate;
        lambda = std::max(lambda * 0.7, 1.0e-12);
      } else {
        lambda = std::min(lambda * 2.0, 1.0e12);
      }

      if(debugMode) {
        multifit::callback(iter, x);
      }
    }

    if(logMode) {
      fprintf(stderr, "Summary:\n");
      fprintf(stderr, "Method type: OpenCV Levenberg-Marquardt\n");
      fprintf(stderr, "Status: %s\n", status == 0 ? "success" : "failure");
      fprintf(stderr, "Number of iterations: %zu\n", iter_count);
      fprintf(stderr, "Function evaluations: n/a\n");
      fprintf(stderr, "Jacobian evaluations: n/a\n");
      if(status == 0) {
        fprintf(stderr, "Reason for stopping: %s\n", (info == 1) ? "small step size" : (info == 2) ? "small gradient" : (info == 3) ? "small residual change" : "max iterations");
      }
      fprintf(stderr, "Initial |f(x)| = %g\n", chisq0);
      fprintf(stderr, "Final   |f(x)| = %g\n", chisq);
      printf("\n\n");
    }

    for(size_t i=0; i<params.size(); ++i)
      params[i] = x.at<double>(static_cast<int>(i), 0);

  }

#endif

} /* namespace mpl::solver::multifit */

#endif /* _H_MPL_SOLVER_MULTIFIT_H_ */
