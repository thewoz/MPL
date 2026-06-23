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



#ifndef _H_MPL_VISION_CAMERASYSTEM_H_
#define _H_MPL_VISION_CAMERASYSTEM_H_

#include <cstdlib>
#include <cstdio>

#include <array>
#include <tuple>
#include <type_traits>

#include <opencv2/opencv.hpp>

#include <mpl/core/stdio.hpp>

#include <mpl/math/geometry.hpp>

#include <mpl/vision/camera.hpp>
#include <mpl/vision/undistort.hpp>
#include <mpl/vision/reprojection.hpp>
#include <mpl/vision/reconstruction.hpp>
#include <mpl/vision/utils.hpp>

#define CAM(num) (size_t)(num-1)

//*****************************************************************************/
// namespace mpl::vision
//*****************************************************************************/
namespace mpl::vision {

  //*****************************************************************************/
  // namespace detail
  //*****************************************************************************/
  namespace detail {

    // Variadic trait: ::value is true only if all the template arguments
    // are exactly the same type.
    template <typename ... args>
    struct allsame : public std::false_type {};

    template <typename T>
    struct allsame<T> : public std::true_type {};

    template <typename T, typename ... args>
    struct allsame<T, T, args...> : public allsame<T, args ... > {};

  } // namespace detail

  //*****************************************************************************/
  // class cameraSystem_t
  //*****************************************************************************/
  template <std::size_t N>
  class cameraSystem_t {
    
  public:
    
    std::array<mpl::vision::camera_t, N> cameras;
    
    std::array< std::array<cv::Mat, N>, N> fundamentalMatrix;

  public:
    
    inline std::size_t cameraNumber() const { return N; }
    
    //*****************************************************************************/
    // initFundamentalMatrix
    //*****************************************************************************/
    void initFundamentalMatrix(){
      
      
      for(std::size_t i=0; i<N; ++i){
        
        fundamentalMatrix[i][i] = cv::Mat::eye(3,3, CV_64F);
        
        for(std::size_t j=i+1; j<N; ++j){
          
          mpl::vision::fundamentalFromProjections(cameras[i].getProjectionalMatrix(), cameras[j].getProjectionalMatrix(), fundamentalMatrix[i][j]);
          
          fundamentalMatrix[j][i] = fundamentalMatrix[i][j].t();
          
        }
        
      }

    }
    
    //*****************************************************************************/
    // undistort
    //*****************************************************************************/
    template<typename T>
    void undistort(const T & src, T & dst, size_t camera) {
     
      if(camera > N){
        fprintf(stderr, "mpl::vision::cameraSystem_t::undistort() error: camera index out of range\n");
        abort();
      }
      
      mpl::vision::undistort(src, dst, cameras[camera].getCameraMatrix(), cameras[camera].getDistortionCoefficients());

    }
    
    template<typename T>
    void undistort(T & src, size_t camera) {
      
      if(camera > N){
        fprintf(stderr, "mpl::vision::cameraSystem_t::undistort() error: camera index out of range\n");
        abort();
      }
      
      mpl::vision::undistort(src, src, cameras[camera].getCameraMatrix(), cameras[camera].getDistortionCoefficients());
      
    }

    
    //*****************************************************************************/
    // reproject
    //*****************************************************************************/
    template<typename T3D, class ... args>
    void reproject(const T3D & point3D, args & ... list) {
      
      static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(detail::allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;

      std::array<std::reference_wrapper<T>, N> points2D {list ... };
      
       for(size_t i=0; i<N; ++i)
         mpl::vision::reproject(point3D, points2D[i].get(), cameras[i].getProjectionalMatrix());
      
    }

    //*****************************************************************************/
    // reconstruct
    //*****************************************************************************/
    template<typename T3D, class ... args>
    void reconstruct(T3D & point3D, const args & ... list) {
      
      //static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(detail::allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      std::array<std::reference_wrapper<const T>, sizeof...(list)> points2D {list ... };
      
      if constexpr (sizeof...(list)==3) {
        
        mpl::vision::reconstruct(points2D[0].get(), cameras[0].getProjectionalMatrix(),
                            points2D[1].get(), cameras[1].getProjectionalMatrix(),
                            points2D[2].get(), cameras[2].getProjectionalMatrix(),
                            point3D);
        
      }
      
      static_assert(sizeof...(list)==3, "Wrong params number");

    }
    
    //*****************************************************************************/
    // reconstruct
    //*****************************************************************************/
    template<typename T3D, class ... args>
    void reconstruct(T3D & point3D, size_t camA, size_t camB, const args & ... list) {
      
      //static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(detail::allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      std::array<std::reference_wrapper<const T>, sizeof...(list)> points2D {list ... };
      
      if constexpr (sizeof...(list)==2) {
        
        mpl::vision::reconstruct(points2D[0].get(), cameras[camA].getProjectionalMatrix(),
                            points2D[1].get(), cameras[camB].getProjectionalMatrix(),
                            point3D);
        
      }
      
      static_assert(sizeof...(list)==2, "Wrong params number");
      
    }
    
    
   #define RECO_ERROR(a,b,c,e) if(points2DC1[i].id == a && points2DC2[j].id == b && points2DC3[k].id == c) printf("RECO_ERROR %u %u %u -> %e\n", a, b, c, e);
   #define EPI_DIST(a,b,e) if(points2DC1[i].id == a && points2DC2[j].id == b) printf("EPI_DIST %u %u -> %e\n", a, b, e);

#ifdef OLD_RECO

    //*****************************************************************************/
    // reconstruct
    //*****************************************************************************/
    template<typename T3D, class ... args>
    void findMatches(T3D & tuplet, double maxError, const args & ... list) {
      
      static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(detail::allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      std::array<std::reference_wrapper<const T>, N> points2D {list ... };
      
      if constexpr (N==3) {
        
        const T & points2DC1 = points2D[0].get();
        const T & points2DC2 = points2D[1].get();
        const T & points2DC3 = points2D[2].get();

        cv::Mat prjMatC1 = cameras[0].getProjectionalMatrix();
        cv::Mat prjMatC2 = cameras[1].getProjectionalMatrix();
        cv::Mat prjMatC3 = cameras[2].getProjectionalMatrix();

        for(size_t i=0; i<points2DC1.size(); ++i){
          
          for(size_t j=0; j<points2DC2.size(); ++j){
            
            cv::Vec3f lineEp12 = epipolarLine(points2DC1[i], CAM(1), CAM(2));
            
            double distEp1 = mpl::geometric::dist(points2DC2[j], lineEp12);
            
            //EPI_DIST(8593,8848,dist);
            //EPI_DIST(9359,9614,dist);
            //EPI_DIST(9435,9690,dist);
            
            if(distEp1 > maxError*2) continue;
     
            cv::Vec3f lineEp13 = epipolarLine(points2DC1[i], CAM(1), CAM(3));
            cv::Vec3f lineEp23 = epipolarLine(points2DC2[j], CAM(2), CAM(3));
            
            for(size_t k=0; k<points2DC3.size(); ++k){
     
              cv::Point2f cross;
              
              if(!mpl::geometric::intersection(lineEp13, lineEp23, cross)) continue;
              
              double distCross = cv::norm(points2DC3[k], cross);
              /*
              RECO_ERROR(8, 167, 324, distCross);
              RECO_ERROR(8, 167, 327, distCross);

              if(points2DC1[i].id == 8 && points2DC2[j].id == 167 && points2DC3[k].id == 324){
              
                printf("lineEp13 %e %e %e\n", lineEp13[0], lineEp13[1], lineEp13[2]);
                printf("lineEp23 %e %e %e\n", lineEp23[0], lineEp23[1], lineEp23[2]);

                printf("cross %e %e \n", cross.x, cross.y);
                
                double error = mpl::vision::reconstruction::error(points2DC1[i], prjMatC1, points2DC2[j], prjMatC2, points2DC3[k], prjMatC3,  maxError);
              
                printf("AAA %e\n", error);
              
                points2DC3[k].print();
                
                //exit(0);
              }
              
              if(points2DC1[i].id == 8 && points2DC2[j].id == 167 && points2DC3[k].id == 327){
                
                printf("lineEp13 %e %e %e\n", lineEp13[0], lineEp13[1], lineEp13[2]);
                printf("lineEp23 %e %e %e\n", lineEp23[0], lineEp23[1], lineEp23[2]);
                
                printf("cross %e %e \n", cross.x, cross.y);
                
                double error = mpl::vision::reconstruction::error(points2DC1[i], prjMatC1, points2DC2[j], prjMatC2, points2DC3[k], prjMatC3,  maxError);
                
                printf("AAA %e\n", error);
                
                points2DC3[k].print();
                
                exit(0);
              }

              */
              //if(points2DC3[k].id == 324) points2DC3[k].print();
              
              if(distCross > maxError*2) continue;
              
              double error = mpl::vision::reconstruction::error(points2DC1[i], prjMatC1, points2DC2[j], prjMatC2, points2DC3[k], prjMatC3,  maxError);
              
              //RECO_ERROR(8, 167, 324, error);
              
             //RECO_ERROR(8593,8848,9104,error);
              //RECO_ERROR(9359,9614,9870,error);
              //RECO_ERROR(9435,9690,9870,error);
              
              if(error<=maxError) {
                
                tuplet.push_back(std::tuple<size_t,size_t,size_t>(i,j,k));
                
              }
              
            } // for k
            
          } // for j
          
        } // for i
        
      } else { static_assert(flase, "Wrong params number"); }
      
      //exit(0);
      
    }

#endif

    //*****************************************************************************/
    // reconstruct
    //*****************************************************************************/
    template<typename T3D, class ... args>
    void findMatches(T3D & tuplet, double maxError, const args & ... list) {
      
      static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(detail::allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      std::array<std::reference_wrapper<const T>, N> points2D {list ... };
      
      if constexpr (N==3) {
        
        const T & points2DC1 = points2D[0].get();
        const T & points2DC2 = points2D[1].get();
        const T & points2DC3 = points2D[2].get();

        cv::Mat prjMatC1 = cameras[0].getProjectionalMatrix();
        cv::Mat prjMatC2 = cameras[1].getProjectionalMatrix();
        cv::Mat prjMatC3 = cameras[2].getProjectionalMatrix();

        for(size_t i=0; i<points2DC1.size(); ++i){
          
          for(size_t j=0; j<points2DC2.size(); ++j){
            
            cv::Vec3f lineEp12 = epipolarLine(points2DC1[i], CAM(1), CAM(2));
            
            double distEp1 = mpl::geometry::distance::fromLine(points2DC2[j], lineEp12);

            if(distEp1 > maxError*2) continue;
     
            for(size_t k=0; k<points2DC3.size(); ++k){
     
              double error = mpl::vision::reconstruction::error(points2DC1[i], prjMatC1, points2DC2[j], prjMatC2, points2DC3[k], prjMatC3,  maxError);
              
              if(error<=maxError) {
                
                //tuplet.push_back(std::tuple<size_t, size_t, size_t, double>(i,j,k,error));
                tuplet.push_back(std::tuple<size_t, size_t, size_t>(i,j,k));   
                
              }
              
            } // for k
            
          } // for j
          
        } // for i
        
      }
      
      static_assert(N==3, "Wrong params number");
      
      //exit(0);
      
    }

    
    //*****************************************************************************/
    // reconstruct
    //*****************************************************************************/
    template<typename T3D, class ... args>
    auto reconstruct(const args & ... list) { T3D out; reconstruct(out, list ...); return out; }
    
    //*****************************************************************************/
    // reconstructError
    //*****************************************************************************/
    template<class ... args>
    void reconstructError(double & error, double maxError, const args & ... list) {
      
      static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(detail::allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      std::array<std::reference_wrapper<const T>, N> points2D {list ... };
      
      if constexpr (N==3) {
        
        error = mpl::vision::reconstruction::error(points2D[0].get(), cameras[0].getProjectionalMatrix(),
                                              points2D[1].get(), cameras[1].getProjectionalMatrix(),
                                              points2D[2].get(), cameras[2].getProjectionalMatrix(),
                                              maxError);
        
      }
      
      static_assert(N==3, "Wrong params number");
      
    }
    
    //*****************************************************************************/
    // reconstructError
    //*****************************************************************************/
    template<class ... args>
    double reconstructError(double maxError, const args & ... list) { double error; reconstructError(error, maxError, list ...); return error; }

    //*****************************************************************************/
    // epipolarLines
    //*****************************************************************************/
    template<typename T>
    inline cv::Vec3f epipolarLine(const T & point, size_t cameraFrom, size_t cameraTo) {
      
      cv::Vec3f line;
      
      mpl::vision::epipolarLine(point, line, fundamentalMatrix[cameraTo][cameraFrom]);
      
      return line;
      
    }

    
    //*****************************************************************************/
    // loadProjectionMatrices
    //*****************************************************************************/
    void loadProjectionMatrices(const std::string & file) {

      FILE * input = mpl::io::open(file,"r");
      
      char line[PATH_MAX];
      
      size_t lineRead = 0;
      
      // ciclo su tutte le linee del file
      while(fgets(line, PATH_MAX, input)){
        
        if(lineRead < N) cameras[lineRead].initProjectionMatrix(line);
        
        ++lineRead;
        
      }
      
      if(lineRead < N){
        fprintf(stderr, "mpl::vision::cameraSystem_t::loadProjectionMatrices() error: too few lines in '%s'\n", file.c_str());
        abort();
      }

      if(lineRead > N){
        fprintf(stderr, "mpl::vision::cameraSystem_t::loadProjectionMatrices() warning: too many lines in '%s'\n", file.c_str());
      }
      
      mpl::io::close(input);
      
      initFundamentalMatrix();

    }

    
  }; // class cameraSystem_t
  
  
  typedef cameraSystem_t<1> cameraSystem1_t;
  typedef cameraSystem_t<2> cameraSystem2_t;
  typedef cameraSystem_t<3> cameraSystem3_t;
  
  
  
  
  
  
  
  
  
  
  
  
  
  
#if(0)

  
    template<class ... args>
    double reconstructionError(const args & ... list) {
      
      static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(detail::allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      static_assert(std::is_same<T, cv::Point_<float>   >::value ||
                    std::is_same<T, cv::Point_<double>  >::value, "use a undefine specialization");
      
      const T * points2D = new T[sizeof...(list)]{ * list ... };
      
      double error;
      
      if constexpr (N==3)
        
        error = mpl::vision::reconstruction::error(points2D[0], cameras[0].getProjectionalMatrix(),
                                      points2D[1], cameras[1].getProjectionalMatrix(),
                                      points2D[2], cameras[2].getProjectionalMatrix());
      
      else
        static_assert(false, "Wrong params number");
      
      delete[] points2D;
      
      return error;
      
    }
  
  
  
  }; // class cameraSystem_t

  typedef cameraSystem_t<1> cameraSystem1_t;
  typedef cameraSystem_t<2> cameraSystem2_t;
  typedef cameraSystem_t<3> cameraSystem3_t;

#endif


} // namespace mpl::vision





















#if(0)
//*****************************************************************************/
// reproject
//*****************************************************************************/
template<typename T3D, class ... args>
void reproject(const cv::Point3_<T3D> & point3D, args & ... list) { reproject(point3D, & list ...); }


template<typename T3D, class ... args>
void reproject(const cv::Point3_<T3D> & point3D, args * ... list) {
  
  static_assert(sizeof...(list)==N, "Wrong params number");
  
  static_assert(detail::allsame<args ... >::value, "Params must have same types");
  
  typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
  
  T * points2D = new T[sizeof...(list)]{ * list ... };
  
  for(size_t i=0; i<N; ++i)
    mpl::vision::reproject(point3D, points2D[i], cameras[i].getProjectionalMatrix());
  
  delete[] points2D;
  
}
#endif


#endif // _H_MPL_VISION_CAMERASYSTEM_H_
