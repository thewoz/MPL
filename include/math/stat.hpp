/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017-2026
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


#ifndef _H_MPL_MATH_STAT_H_
#define _H_MPL_MATH_STAT_H_

#include <cstdlib>
#include <cstdio>

#include <cmath>

#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

//*****************************************************************************/
// namespace mpl::statistic
//*****************************************************************************/
namespace mpl::statistic {

  //*****************************************************************************/
  //  average
  //*****************************************************************************/
  template <typename T>
  double avg(const T & set, double * stddev = nullptr){

    double sum = 0;
    double temp = 0;

    for(const auto & value : set){
      sum  += value;
      temp += value * value;
    }

    double avg = sum / (double)set.size();

    if(stddev != nullptr)
      *stddev = sqrt((temp - (set.size() * (avg*avg))) / (double)(set.size() - 1));

    return avg;

  }


  //*****************************************************************************/
  //  median
  //*****************************************************************************/
  template <typename T>
  double median(const T & _set, double * stddev = nullptr){

    T set = _set;

    std::size_t halfSize = set.size() * 0.5;

    //std::nth_element(minDist.begin(), minDist.begin()+halfSize+2, minDist.end());

    std::sort(set.begin(), set.end());

    double median = 0;

    if((set.size() % 2) != 0) median = set[halfSize];
    //else median = pow((sqrt(minDist[halfSize-1]) + sqrt(minDist[halfSize])) * 0.5, 2);
    else median = (set[halfSize-1] + set[halfSize]) * 0.5;

    if(stddev != nullptr){

      double sum = 0;

      for(const auto & value : set)
        sum += (value - median) * (value - median);

      *stddev = sqrt(sum / (double)(set.size()-1));

    }

    return median;

  }


  //*****************************************************************************/
  // class stat_t
  //*****************************************************************************/
  class stat_t {

  public:

    stat_t() : m_n(0) { }

    void clear() {
      m_n = 0;
    }

    inline void add(double x) {

      m_n++;

      // See Knuth TAOCP vol 2, 3rd edition, page 232
      if (m_n == 1) {
        m_oldM = m_newM = x;
        m_oldS = 0.0;
      } else {
        m_newM = m_oldM + (x - m_oldM)/m_n;
        m_newS = m_oldS + (x - m_oldM)*(x - m_newM);

        // set up for next iteration
        m_oldM = m_newM;
        m_oldS = m_newS;
      }
    }

    int size() const {
      return m_n;
    }

    double mean() const {
      return (m_n > 0) ? m_newM : 0.0;
    }

    double variance() const {
      return ( (m_n > 1) ? m_newS/(m_n - 1) : 0.0 );
    }

    double std() const {
      return std::sqrt(variance());
    }

  private:

    int m_n;
    double m_oldM, m_newM, m_oldS, m_newS;

  };


  //****************************************************************************/
  // normal_pdf()
  //****************************************************************************/
  double normal_pdf(double x, double mean, double sigma) {

    return (1.0/(sigma*std::sqrt(2.0*M_PI))) * std::exp(-0.5 * std::pow((x-mean) / sigma, 2));

  }

  //****************************************************************************/
  // multivariate_normal_pdf()
  //****************************************************************************/
  double multivariate_normal_pdf(const cv::Point2d & point, const cv::Point2d & mean, const cv::Mat & covariance) {

    double det = cv::determinant(covariance);

    double norm = 1.0 / (2.0 * M_PI * std::sqrt(det));

    cv::Mat centered = cv::Mat_<float>(point - mean);

    double exponent = cv::Mat(-0.5 * centered.t() * covariance.inv() * centered).at<float>(0);

    return norm * std::exp(exponent);

  }

} // namespace mpl::statistic

#endif // _H_MPL_MATH_STAT_H_
