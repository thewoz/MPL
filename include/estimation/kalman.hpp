/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2024
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

#ifndef _H_MPL_KALMAN_H_
#define _H_MPL_KALMAN_H_

#include <cstdio>
#include <cstdlib>

#include <mpl/math.hpp>

#include <opencv2/opencv.hpp>

//****************************************************************************
// namespace mpl
//****************************************************************************
namespace mpl {

  //****************************************************************************/
  // class kalman
  //****************************************************************************/
  class kalman_t {
    
  private:
    
    cv::KalmanFilter KF;
        
    cv::Point2d predicted;
    
  public:
    
    //****************************************************************************/
    // kalman_t()
    //****************************************************************************/
    kalman_t() {
      
      KF = cv::KalmanFilter(4, 2, 0);
      
      // equazione del moto
      KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);;
      
      setIdentity(KF.measurementMatrix);
      
      // Errore funzione lineare
      cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1));
      
      // Errore sulla misura
      cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1));
      
      // Covarianza iterazione zero
      cv::setIdentity(KF.errorCovPost, cv::Scalar::all(.2));
      cv::setIdentity(KF.errorCovPre,  cv::Scalar::all(.1));
      
      KF.statePost = cv::Scalar::all(0);
       
    }
    
    //****************************************************************************/
    // init()
    //****************************************************************************/
    inline void init(const cv::Point2d & point) {
      
      KF.statePost = cv::Scalar::all(0);

      KF.statePost.at<float>(0) = point.x;
      KF.statePost.at<float>(1) = point.y;
      
    }
    
    //****************************************************************************/
    // correct()
    //****************************************************************************/
    inline void correct(const cv::Point2d & point) {
     
      cv::Mat_<float> measurement(2,1);
      
      measurement.setTo(cv::Scalar(0));
      
      // aggiorno la misura
      measurement(0) = point.x;
      measurement(1) = point.y;
      
      KF.correct(measurement);

    }
    
    //****************************************************************************/
    // predict()
    //****************************************************************************/
    inline cv::Point2d predict() {
      
      cv::Mat _predicted = KF.predict();

      predicted.x = _predicted.at<float>(0);
      predicted.y = _predicted.at<float>(1);
      
      return predicted;
     
    }
    
    //****************************************************************************/
    // drawPrediction()
    //****************************************************************************/
    void drawPrediction(cv::Mat & image, int size = 30) {
      
      if(image.type() != CV_8UC3 && image.type() != CV_8UC1) {
        fprintf(stderr, "kalman_t::drawPrediction() error: destination type image must be CV_8UC3 or CV_8UC1\n");
        abort();
      }
      
      double norm = mpl::math::multivariate_normal_pdf(predicted, predicted, KF.errorCovPre(cv::Rect(0,0,2,2)));
      
      cv::Mat cova = KF.errorCovPre(cv::Rect(0,0,2,2));
      
      double det = cv::determinant(cova);
      
      double norm1 = 1.0 / (2.0 * M_PI * std::sqrt(det));
      
      int fromX = predicted.x - size;
      if(fromX < 0) fromX = 0;
   
      int toX = predicted.x + size;
      if(toX > image.rows) toX = image.rows;
      
      int fromY = predicted.y - size;
      if(fromY < 0) fromY = 0;
   
      int toY = predicted.y + size;
      if(toY > image.cols) toY = image.cols;

      if(image.type() == CV_8UC3) {
        for(int i=fromX; i<toX; ++i) {
          for(int j=fromY; j<toY; ++j) {
            cv::Mat centered = cv::Mat_<float>(cv::Point2d(i,j) - predicted);
            double exponent = cv::Mat(-0.5 * centered.t() * cova.inv() * centered).at<float>(0);
            double value = norm1 * std::exp(exponent);
            image.at<cv::Vec3b>(i,j) = cv::Vec3b(255 * (value / norm), 255 * (value / norm), 255 * (value / norm));
          }
        }
      }
      
      if(image.type() == CV_8UC1) {
        for(int i=fromX; i<toX; ++i) {
          for(int j=fromY; j<toY; ++j) {
            cv::Mat centered = cv::Mat_<float>(cv::Point2d(i,j) - predicted);
            double exponent = cv::Mat(-0.5 * centered.t() * cova.inv() * centered).at<float>(0);
            double value = norm1 * std::exp(exponent);
            image.at<uchar>(i,j) = 255 * (value / norm);
          }
        }
      }

    }
    
  };
  
} /* namespace mpl */
  
#endif /* _H_MPL_KALMAN_H_ */
