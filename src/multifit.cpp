#include <iostream>
#include <vector>


//#define MPL_MULTIFIT_USE_GSL
//g++ -std=c++17 ./src/multifit_example.cpp `pkg-config --cflags opencv4` -I/usr/local/include `pkg-config --libs opencv4` -lgsl -rpath /usr/local/lib/
#include <mpl/multifit.hpp>

class line_fit_data_t : public mpl::solver::multifit::data_t {
  public:
    std::vector<double> x;
    std::vector<double> y;

    size_t size() const override {
      return x.size();
    }
};

#if defined(MPL_MULTIFIT_USE_GSL)

int residual_func(const gsl_vector * p, void * params, gsl_vector * f) {
  auto * data = static_cast<line_fit_data_t *>(params);
  const double a = gsl_vector_get(p, 0);
  const double b = gsl_vector_get(p, 1);

  for(size_t i = 0; i < data->size(); ++i) {
    gsl_vector_set(f, i, a * data->x[i] + b - data->y[i]);
  }

  return GSL_SUCCESS;
}

int jacobian_func(const gsl_vector * p, void * params, gsl_matrix * J) {
  (void)p;
  auto * data = static_cast<line_fit_data_t *>(params);

  for(size_t i = 0; i < data->size(); ++i) {
    gsl_matrix_set(J, i, 0, data->x[i]);
    gsl_matrix_set(J, i, 1, 1.0);
  }

  return GSL_SUCCESS;
}

#else

int residual_func(const cv::Mat & p, void * params, cv::Mat & f) {
  auto * data = static_cast<line_fit_data_t *>(params);
  const double a = p.at<double>(0, 0);
  const double b = p.at<double>(1, 0);

  f = cv::Mat::zeros(static_cast<int>(data->size()), 1, CV_64F);
  for(size_t i = 0; i < data->size(); ++i) {
    f.at<double>(static_cast<int>(i), 0) = a * data->x[i] + b - data->y[i];
  }

  return 0;
}

int jacobian_func(const cv::Mat & p, void * params, cv::Mat & J) {
  (void)p;
  auto * data = static_cast<line_fit_data_t *>(params);

  J = cv::Mat::zeros(static_cast<int>(data->size()), 2, CV_64F);
  for(size_t i = 0; i < data->size(); ++i) {
    J.at<double>(static_cast<int>(i), 0) = data->x[i];
    J.at<double>(static_cast<int>(i), 1) = 1.0;
  }

  return 0;
}

#endif

int main() {
  line_fit_data_t data;
  data.x = {0.0, 1.0, 2.0, 3.0, 4.0};
  data.y = {1.1, 2.9, 5.1, 6.8, 9.2};

  mpl::solver::multifit::params_t params(2);
  params[0] = 0.0;
  params[1] = 0.0;

  mpl::solver::multifit::solve(data, params, residual_func, jacobian_func, 200, 1.0e-8, 1.0e-8, 1.0e-8, true, false);

  std::cout << "Stima lineare: y = a*x + b" << std::endl;
  std::cout << "a = " << params[0] << std::endl;
  std::cout << "b = " << params[1] << std::endl;

  return 0;
}
