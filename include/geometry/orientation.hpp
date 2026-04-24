/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2026
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

#ifndef _H_MPL_ORIENTATION_H_
#define _H_MPL_ORIENTATION_H_

#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/opencv.hpp>

//*****************************************************************************
// namespace mpl
//*****************************************************************************
namespace mpl {

  //*****************************************************************************
  // orientation_t
  //*****************************************************************************
  struct orientation_t {
    
    cv::Point2d center    = cv::Point2d(0.0, 0.0);
    cv::Point2d majorDir  = cv::Point2d(1.0, 0.0);
    cv::Point2d minorDir  = cv::Point2d(0.0, 1.0);
    
    double angleDeg       = 0.0;
    double majorLength    = 0.0;
    double minorLength    = 0.0;

    bool valid() const {
      return majorLength > 0.0 || minorLength > 0.0;
    }
    
  };

  //*****************************************************************************
  // namespace orientation
  //*****************************************************************************
  namespace orientation {

    //*****************************************************************************
    // computationMethod
    //*****************************************************************************
    enum class computationMethod { MOMENTS = 0, PCA = 1 };

    //*****************************************************************************
    // namespace utils
    //*****************************************************************************
    namespace utils {

      //*****************************************************************************
      // extractNonZeroPixels()
      //*****************************************************************************
      inline std::vector<cv::Point> extractNonZeroPixels(const cv::Mat & binaryMask) {

        CV_Assert(!binaryMask.empty());
        CV_Assert(binaryMask.type() == CV_8UC1);

        std::vector<cv::Point> pts;
        pts.reserve(static_cast<size_t>(cv::countNonZero(binaryMask)));

        for(int y = 0; y < binaryMask.rows; ++y) {
          const uchar * row = binaryMask.ptr<uchar>(y);
          for(int x = 0; x < binaryMask.cols; ++x) {
            if(row[x] != 0) {
              pts.emplace_back(x, y);
            }
          }
        }

        return pts;
        
      }

      //*****************************************************************************
      // createContourMask()
      //*****************************************************************************
      template <class T>
      cv::Mat createContourMask(const std::vector<cv::Point> & contour, T * offset = nullptr, int thickness = 1, int padding = -1) {
        
        if(contour.empty()) return cv::Mat();

        if(thickness == 0) return cv::Mat();

        const cv::Rect bbox = cv::boundingRect(contour);

        const int effectivePadding = (padding >= 0) ? padding : ((thickness > 0) ? thickness * 2 : 1);

        const cv::Point maskOffset( bbox.x - effectivePadding, bbox.y - effectivePadding);

        const cv::Size maskSize(bbox.width  + 2 * effectivePadding, bbox.height + 2 * effectivePadding);

        if(maskSize.width <= 0 || maskSize.height <= 0)
          return cv::Mat();

        if(offset) { offset->x = maskOffset.x; offset->y = maskOffset.y; };

        cv::Mat contourMask = cv::Mat::zeros(maskSize, CV_8UC1);

        std::vector<cv::Point> shiftedContour;
        shiftedContour.reserve(contour.size());

        for(const auto & p : contour)
          shiftedContour.emplace_back(p.x - maskOffset.x, p.y - maskOffset.y);

        const std::vector<std::vector<cv::Point>> contours = { shiftedContour };

        cv::drawContours(contourMask, contours, 0, cv::Scalar(255),thickness, cv::LINE_8);

        return contourMask;
        
      }
    
      //*****************************************************************************
      // normalizeAngle180()
      // returns angle in [0,180)
      //*****************************************************************************
      inline double normalizeAngle180(double angleDeg) {
        while(angleDeg < 0.0)   angleDeg += 180.0;
        while(angleDeg >= 180.0) angleDeg -= 180.0;
        return angleDeg;
      }

      //*****************************************************************************
      // computeProjectedLengthsFromMask()
      //*****************************************************************************
      inline void computeProjectedLengthsFromMask(const cv::Mat & image, const cv::Point2d & center, const cv::Point2d & majorDir, const cv::Point2d & minorDir, double & majorLength, double & minorLength) {
        
        CV_Assert(!image.empty());
        CV_Assert(image.type() == CV_8UC1);

        double minMaj =  std::numeric_limits<double>::max();
        double maxMaj = -std::numeric_limits<double>::max();
        double minMin =  std::numeric_limits<double>::max();
        double maxMin = -std::numeric_limits<double>::max();

        bool found = false;

        for(int y = 0; y < image.rows; ++y) {
          
          const uchar * row = image.ptr<uchar>(y);

          for(int x = 0; x < image.cols; ++x) {
            
            if(row[x] == 0) continue;

            found = true;

            cv::Point2d d(x - center.x, y - center.y);

            double projMaj = d.x * majorDir.x + d.y * majorDir.y;
            double projMin = d.x * minorDir.x + d.y * minorDir.y;

            minMaj = std::min(minMaj, projMaj);
            maxMaj = std::max(maxMaj, projMaj);
            minMin = std::min(minMin, projMin);
            maxMin = std::max(maxMin, projMin);
            
          }
          
        }

        if(!found) {
          majorLength = 0.0;
          minorLength = 0.0;
          return;
        }

        majorLength = maxMaj - minMaj;
        minorLength = maxMin - minMin;
        
      }

      //*****************************************************************************
      // computeProjectedLengthsFromPoints()
      //*****************************************************************************
      inline void computeProjectedLengthsFromPoints(const std::vector<cv::Point> & pts, const cv::Point2d & center, const cv::Point2d & majorDir, const cv::Point2d & minorDir, double & majorLength, double & minorLength) {
        
        if(pts.empty()) {
          majorLength = 0.0;
          minorLength = 0.0;
          return;
        }

        double minMaj =  std::numeric_limits<double>::max();
        double maxMaj = -std::numeric_limits<double>::max();
        double minMin =  std::numeric_limits<double>::max();
        double maxMin = -std::numeric_limits<double>::max();

        for(const auto & p : pts) {
          
          cv::Point2d d(p.x - center.x, p.y - center.y);

          double projMaj = d.x * majorDir.x + d.y * majorDir.y;
          double projMin = d.x * minorDir.x + d.y * minorDir.y;

          minMaj = std::min(minMaj, projMaj);
          maxMaj = std::max(maxMaj, projMaj);
          minMin = std::min(minMin, projMin);
          maxMin = std::max(maxMin, projMin);
          
        }

        majorLength = maxMaj - minMaj;
        minorLength = maxMin - minMin;
        
      }

      //*****************************************************************************
      // setAxesFromAngle()
      //*****************************************************************************
      inline void setAxesFromAngle( double angleRad, cv::Point2d & majorDir, cv::Point2d & minorDir) {
        majorDir = cv::Point2d(std::cos(angleRad),  std::sin(angleRad));
        minorDir = cv::Point2d(-std::sin(angleRad), std::cos(angleRad));
      }

    } /* namespace utils */

    //*****************************************************************************
    // get
    //*****************************************************************************
    inline orientation_t get(const cv::Mat & image, computationMethod method) {

      orientation_t r;

      CV_Assert(!image.empty());
      CV_Assert(image.type() == CV_8UC1);

      if(method == computationMethod::MOMENTS) {

        cv::Moments mu = cv::moments(image, true);

        if(mu.m00 <= 0.0) return r;

        r.center.x = mu.m10 / mu.m00;
        r.center.y = mu.m01 / mu.m00;

        const double a = mu.mu20 / mu.m00;
        const double b = mu.mu11 / mu.m00;
        const double c = mu.mu02 / mu.m00;

        const double angleRad = 0.5 * std::atan2(2.0 * b, a - c);

        utils::setAxesFromAngle(angleRad, r.majorDir, r.minorDir);
        r.angleDeg = utils::normalizeAngle180(angleRad * 180.0 / CV_PI);

        utils::computeProjectedLengthsFromMask(image, r.center, r.majorDir, r.minorDir,  r.majorLength,  r.minorLength);

      } else if(method == computationMethod::PCA) {

        std::vector<cv::Point> pts = utils::extractNonZeroPixels(image);
        if(pts.size() < 2) return r;

        cv::Mat dataPts(static_cast<int>(pts.size()), 2, CV_64F);

        for(int i = 0; i < static_cast<int>(pts.size()); ++i) {
          dataPts.at<double>(i, 0) = static_cast<double>(pts[i].x);
          dataPts.at<double>(i, 1) = static_cast<double>(pts[i].y);
        }

        cv::PCA pca(dataPts, cv::Mat(), cv::PCA::DATA_AS_ROW);

        r.center.x = pca.mean.at<double>(0, 0);
        r.center.y = pca.mean.at<double>(0, 1);

        r.majorDir.x = pca.eigenvectors.at<double>(0, 0);
        r.majorDir.y = pca.eigenvectors.at<double>(0, 1);

        r.minorDir.x = pca.eigenvectors.at<double>(1, 0);
        r.minorDir.y = pca.eigenvectors.at<double>(1, 1);

        const double angleRad = std::atan2(r.majorDir.y, r.majorDir.x);
        r.angleDeg = utils::normalizeAngle180(angleRad * 180.0 / CV_PI);

        utils::computeProjectedLengthsFromPoints(pts, r.center, r.majorDir, r.minorDir, r.majorLength, r.minorLength);

      } else { return r; }

      return r;
      
    }

    //*****************************************************************************
    // get
    //*****************************************************************************
    inline orientation_t get(const std::vector<cv::Point> & contour, computationMethod method) {

      orientation_t r;

      if(contour.size() < 2) return r;

      if(method == computationMethod::MOMENTS) {

        cv::Point offset;
        cv::Mat mask = utils::createContourMask(contour, &offset);
        cv::Moments mu = cv::moments(mask, true);

        if(mu.m00 <= 0.0) return r;

        r.center.x = (mu.m10 / mu.m00) + offset.x;
        r.center.y = (mu.m01 / mu.m00) + offset.y;

        const double a = mu.mu20 / mu.m00;
        const double b = mu.mu11 / mu.m00;
        const double c = mu.mu02 / mu.m00;

        const double angleRad = 0.5 * std::atan2(2.0 * b, a - c);

        utils::setAxesFromAngle(angleRad, r.majorDir, r.minorDir);
        r.angleDeg = utils::normalizeAngle180(angleRad * 180.0 / CV_PI);

        utils::computeProjectedLengthsFromMask(mask, r.center, r.majorDir, r.minorDir, r.majorLength, r.minorLength);

      } else if(method == computationMethod::PCA) {

        cv::Mat dataPts(static_cast<int>(contour.size()), 2, CV_64F);

        for(int i = 0; i < static_cast<int>(contour.size()); ++i) {
          dataPts.at<double>(i, 0) = static_cast<double>(contour[i].x);
          dataPts.at<double>(i, 1) = static_cast<double>(contour[i].y);
        }

        cv::PCA pca(dataPts, cv::Mat(), cv::PCA::DATA_AS_ROW);

        r.center.x = pca.mean.at<double>(0, 0);
        r.center.y = pca.mean.at<double>(0, 1);

        r.majorDir.x = pca.eigenvectors.at<double>(0, 0);
        r.majorDir.y = pca.eigenvectors.at<double>(0, 1);

        r.minorDir.x = pca.eigenvectors.at<double>(1, 0);
        r.minorDir.y = pca.eigenvectors.at<double>(1, 1);

        const double angleRad = std::atan2(r.majorDir.y, r.majorDir.x);
        r.angleDeg = utils::normalizeAngle180(angleRad * 180.0 / CV_PI);

        utils::computeProjectedLengthsFromPoints(contour, r.center, r.majorDir, r.minorDir, r.majorLength, r.minorLength);

      } else { return r; }

      return r;
      
    }

    //*****************************************************************************
    // backward-compatible overloads
    //*****************************************************************************
    inline orientation_t get(const cv::Mat & image, int method) {
      return get(image, method);
    }

    inline orientation_t get(const std::vector<cv::Point> & contour, int method) {
      return get(contour, method);
    }

  } /* namespace orientation */

} /* namespace mpl */

#endif // _H_MPL_ORIENTATION_H_
