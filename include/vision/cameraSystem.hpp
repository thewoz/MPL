/*
 * MIT License
 *
 * Copyright © 2017 COBBS
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



#ifndef _COBBS_VISION_CAMERASYSTEM_H_
#define _COBBS_VISION_CAMERASYSTEM_H_


template <typename ... args>
struct allsame : public std::false_type {};


template <typename T>
struct allsame<T> : public std::true_type {};


template <typename T, typename ... args>
struct allsame<T, T, args...> : public allsame<T, args ... > {};

template <typename... args>
typename std::tuple_element<0, std::tuple<args...> >::type * make_arr(args... arg) {
  
  // Code to check if passed args are of the same type
  static_assert(allsame<args ...>::value, "Params must have same types");
  
  typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
  
  T* arr = new T[sizeof...(arg)]{ arg... };
  
  return arr;
  
};


#include <cstdlib>
#include <cstdio>

#include <array>
#include <tuple>

#include <opencv2/opencv.hpp>

#include <mpl/stdio.hpp>

#include <mpl/geometry/geometry.hpp>

#include <mpl/vision/camera.hpp>
#include <mpl/vision/undistort.hpp>
#include <mpl/vision/reprojection.hpp>
#include <mpl/vision/reconstruction.hpp>

#define CAM(num) (num-1)

/*****************************************************************************/
// namespace vision
/*****************************************************************************/
namespace vision {
  
  /*****************************************************************************/
  // class cameraSystem_t
  /*****************************************************************************/
  template <std::size_t N>
  class cameraSystem_t {
    
  public:
    
    std::array<vision::camera_t, N> cameras;
    
    std::array< std::array<cv::Mat, N>, N> fundamentalMatrix;

  public:
    
    inline std::size_t cameraNumber() const { return N; }
    
    /*****************************************************************************/
    // initFundamentalMatrix
    /*****************************************************************************/
    void initFundamentalMatrix(){
      
      
      for(std::size_t i=0; i<N; ++i){
        
        fundamentalMatrix[i][i] = cv::Mat::eye(3,3, CV_64F);
        
        for(std::size_t j=i+1; j<N; ++j){
          
          fundamentalFromProjections(cameras[i].getProjectionalMatrix(), cameras[j].getProjectionalMatrix(), fundamentalMatrix[i][j]);
          
          fundamentalMatrix[j][i] = fundamentalMatrix[i][j].t();
          
        }
        
      }

    }
    
    /*****************************************************************************/
    // undistort
    /*****************************************************************************/
    template<typename T>
    void undistort(const T & src, T & dst, uint32_t camera) {
     
      if(camera > N){
        fprintf(stderr, "errro\n");
        abort();
      }
      
      vision::undistort(src, dst, cameras[camera].getCameraMatrix(), cameras[index].getDistortionCoefficients());

    }
    
    template<typename T>
    void undistort(T & src, uint32_t camera) {
      
      if(camera > N){
        fprintf(stderr, "errro\n");
        abort();
      }
      
      vision::undistort(src, src, cameras[camera].getCameraMatrix(), cameras[camera].getDistortionCoefficients());
      
    }

    
    /*****************************************************************************/
    // reproject
    /*****************************************************************************/
    template<typename T3D, class ... args>
    void reproject(const T3D & point3D, args & ... list) {
      
      static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;

      std::array<std::reference_wrapper<T>, N> points2D {list ... };
      
       for(uint32_t i=0; i<N; ++i)
         vision::reproject(point3D, points2D[i].get(), cameras[i].getProjectionalMatrix());
      
    }

    /*****************************************************************************/
    // reconstruct
    /*****************************************************************************/
    template<typename T3D, class ... args>
    void reconstruct(T3D & point3D, const args & ... list) {
      
      //static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      std::array<std::reference_wrapper<const T>, sizeof...(list)> points2D {list ... };
      
      if constexpr (sizeof...(list)==3) {
        
        vision::reconstruct(points2D[0].get(), cameras[0].getProjectionalMatrix(),
                            points2D[1].get(), cameras[1].getProjectionalMatrix(),
                            points2D[2].get(), cameras[2].getProjectionalMatrix(),
                            point3D);
        
      } else if constexpr (sizeof...(list)==2) { static_assert(true, "Wrong params number");

        vision::reconstruct(points2D[0].get(), cameras[1].getProjectionalMatrix(),
                            points2D[1].get(), cameras[2].getProjectionalMatrix(),
                            point3D);
        
      } else { static_assert(true, "Wrong params number"); }
      
    }
    
   #define RECO_ERROR(a,b,c,e) if(points2DC1[i].id == a && points2DC2[j].id == b && points2DC3[k].id == c) printf("RECO_ERROR %u %u %u -> %e\n", a, b, c, e);
   #define EPI_DIST(a,b,e) if(points2DC1[i].id == a && points2DC2[j].id == b) printf("EPI_DIST %u %u -> %e\n", a, b, e);

#ifdef OLD_RECO

    /*****************************************************************************/
    // reconstruct
    /*****************************************************************************/
    template<typename T3D, class ... args>
    void findMatches(T3D & tuplet, double maxError, const args & ... list) {
      
      static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      std::array<std::reference_wrapper<const T>, N> points2D {list ... };
      
      if constexpr (N==3) {
        
        const T & points2DC1 = points2D[0].get();
        const T & points2DC2 = points2D[1].get();
        const T & points2DC3 = points2D[2].get();

        cv::Mat prjMatC1 = cameras[0].getProjectionalMatrix();
        cv::Mat prjMatC2 = cameras[1].getProjectionalMatrix();
        cv::Mat prjMatC3 = cameras[2].getProjectionalMatrix();

        for(uint32_t i=0; i<points2DC1.size(); ++i){
          
          for(uint32_t j=0; j<points2DC2.size(); ++j){
            
            cv::Vec3f lineEp12 = epipolarLine(points2DC1[i], CAM(1), CAM(2));
            
            double distEp1 = geometric::dist(points2DC2[j], lineEp12);
            
            //EPI_DIST(8593,8848,dist);
            //EPI_DIST(9359,9614,dist);
            //EPI_DIST(9435,9690,dist);
            
            if(distEp1 > maxError*2) continue;
     
            cv::Vec3f lineEp13 = epipolarLine(points2DC1[i], CAM(1), CAM(3));
            cv::Vec3f lineEp23 = epipolarLine(points2DC2[j], CAM(2), CAM(3));
            
            for(uint32_t k=0; k<points2DC3.size(); ++k){
     
              cv::Point2f cross;
              
              if(!geometric::intersection(lineEp13, lineEp23, cross)) continue;
              
              double distCross = cv::norm(points2DC3[k] - cross);
              /*
              RECO_ERROR(8, 167, 324, distCross);
              RECO_ERROR(8, 167, 327, distCross);

              if(points2DC1[i].id == 8 && points2DC2[j].id == 167 && points2DC3[k].id == 324){
              
                printf("lineEp13 %e %e %e\n", lineEp13[0], lineEp13[1], lineEp13[2]);
                printf("lineEp23 %e %e %e\n", lineEp23[0], lineEp23[1], lineEp23[2]);

                printf("cross %e %e \n", cross.x, cross.y);
                
                double error = vision::reconstruction::error(points2DC1[i], prjMatC1, points2DC2[j], prjMatC2, points2DC3[k], prjMatC3,  maxError);
              
                printf("AAA %e\n", error);
              
                points2DC3[k].print();
                
                //exit(0);
              }
              
              if(points2DC1[i].id == 8 && points2DC2[j].id == 167 && points2DC3[k].id == 327){
                
                printf("lineEp13 %e %e %e\n", lineEp13[0], lineEp13[1], lineEp13[2]);
                printf("lineEp23 %e %e %e\n", lineEp23[0], lineEp23[1], lineEp23[2]);
                
                printf("cross %e %e \n", cross.x, cross.y);
                
                double error = vision::reconstruction::error(points2DC1[i], prjMatC1, points2DC2[j], prjMatC2, points2DC3[k], prjMatC3,  maxError);
                
                printf("AAA %e\n", error);
                
                points2DC3[k].print();
                
                exit(0);
              }

              */
              //if(points2DC3[k].id == 324) points2DC3[k].print();
              
              if(distCross > maxError*2) continue;
              
              double error = vision::reconstruction::error(points2DC1[i], prjMatC1, points2DC2[j], prjMatC2, points2DC3[k], prjMatC3,  maxError);
              
              //RECO_ERROR(8, 167, 324, error);
              
             //RECO_ERROR(8593,8848,9104,error);
              //RECO_ERROR(9359,9614,9870,error);
              //RECO_ERROR(9435,9690,9870,error);
              
              if(error<=maxError) {
                
                tuplet.push_back(std::tuple<uint32_t,uint32_t,uint32_t>(i,j,k));
                
              }
              
            } /* for k */
            
          } /* for j */
          
        } /* for i */
        
      } else { static_assert(true, "Wrong params number"); }
      
      //exit(0);
      
    }

#endif

    /*****************************************************************************/
    // reconstruct
    /*****************************************************************************/
    template<typename T3D, class ... args>
    void findMatches(T3D & tuplet, double maxError, const args & ... list) {
      
      static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      std::array<std::reference_wrapper<const T>, N> points2D {list ... };
      
      if constexpr (N==3) {
        
        const T & points2DC1 = points2D[0].get();
        const T & points2DC2 = points2D[1].get();
        const T & points2DC3 = points2D[2].get();

        cv::Mat prjMatC1 = cameras[0].getProjectionalMatrix();
        cv::Mat prjMatC2 = cameras[1].getProjectionalMatrix();
        cv::Mat prjMatC3 = cameras[2].getProjectionalMatrix();

        for(uint32_t i=0; i<points2DC1.size(); ++i){
          
          for(uint32_t j=0; j<points2DC2.size(); ++j){
            
            cv::Vec3f lineEp12 = epipolarLine(points2DC1[i], CAM(1), CAM(2));
            
            double distEp1 = geometric::dist(points2DC2[j], lineEp12);

            if(distEp1 > maxError*2) continue;
     
            for(uint32_t k=0; k<points2DC3.size(); ++k){
     
              double error = vision::reconstruction::error(points2DC1[i], prjMatC1, points2DC2[j], prjMatC2, points2DC3[k], prjMatC3,  maxError);
              
              if(error<=maxError) {
                
                //tuplet.push_back(std::tuple<uint32_t, uint32_t, uint32_t, double>(i,j,k,error));
                tuplet.push_back(std::tuple<uint32_t, uint32_t, uint32_t>(i,j,k));   
                
              }
              
            } /* for k */
            
          } /* for j */
          
        } /* for i */
        
      } else { static_assert(true, "Wrong params number"); }
      
      //exit(0);
      
    }

    
    /*****************************************************************************/
    // reconstruct
    /*****************************************************************************/
    template<typename T3D, class ... args>
    auto reconstruct(const args & ... list) { T3D out; reconstruct(out, list ...); return out; }
    
    /*****************************************************************************/
    // reconstructError
    /*****************************************************************************/
    template<class ... args>
    void reconstructError(double & error, double maxError, const args & ... list) {
      
      static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      std::array<std::reference_wrapper<const T>, N> points2D {list ... };
      
      if constexpr (N==3) {
        
        error = vision::reconstruction::error(points2D[0].get(), cameras[0].getProjectionalMatrix(),
                                              points2D[1].get(), cameras[1].getProjectionalMatrix(),
                                              points2D[2].get(), cameras[2].getProjectionalMatrix(),
                                              maxError);
        
      } else { static_assert(true, "Wrong params number"); }
      
    }
    
    /*****************************************************************************/
    // reconstructError
    /*****************************************************************************/
    template<class ... args>
    double reconstructError(double maxError, const args & ... list) { double error; reconstructError(error, maxError, list ...); return error; }

    /*****************************************************************************/
    // epipolarLines
    /*****************************************************************************/
    template<typename T>
    inline cv::Vec3f epipolarLine(const T & point, uint32_t cameraFrom, uint32_t cameraTo) {
      
      cv::Vec3f line;
      
      vision::epipolarLine(point, line, fundamentalMatrix[cameraTo][cameraFrom]);
      
      return line;
      
    }

    
    /*****************************************************************************/
    // loadProjectionMatrices
    /*****************************************************************************/
    void loadProjectionMatrices(const char * file) {
      
      FILE * input = mpl::io::open(file,"r");
      
      char line[PATH_MAX];
      
      uint32_t lineRead = 0;
      
      // ciclo su tutte le linee del file
      while(fgets(line, PATH_MAX, input)){
        
        if(lineRead < N) cameras[lineRead].initProjectionMatrix(line);
        
        ++lineRead;
        
      }
      
      if(lineRead < N){
        fprintf(stderr, "error less line in %s\n", file);
        abort();
      }
      
      if(lineRead > N){
        fprintf(stderr, "waring more line in %s\n", file);
      }
      
      io::close(input);
      
      initFundamentalMatrix();

    }

    
  }; /* class cameraSystem_t */
  
  
  typedef cameraSystem_t<1> cameraSystem1_t;
  typedef cameraSystem_t<2> cameraSystem2_t;
  typedef cameraSystem_t<3> cameraSystem3_t;
  
  
  
  
  
  
  
  
  
  
  
  
  
  
#if(0)

  
    template<class ... args>
    double reconstructionError(const args & ... list) {
      
      static_assert(sizeof...(list)==N, "Wrong params number");
      
      static_assert(allsame<args ... >::value, "Params must have same types");
      
      typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
      
      static_assert(std::is_same<T, cv::Point_<float>   >::value ||
                    std::is_same<T, cv::Point_<double>  >::value, "use a undefine specialization");
      
      const T * points2D = new T[sizeof...(list)]{ * list ... };
      
      double error;
      
      if constexpr (N==3)
        
        error = vision::reconstruction::error(points2D[0], cameras[0].getProjectionalMatrix(),
                                      points2D[1], cameras[1].getProjectionalMatrix(),
                                      points2D[2], cameras[2].getProjectionalMatrix());
      
      else
        static_assert(true, "Wrong params number");
      
      delete[] points2D;
      
      return error;
      
    }
  
  
  
  }; /* class cameraSystem_t */

  typedef cameraSystem_t<1> cameraSystem1_t;
  typedef cameraSystem_t<2> cameraSystem2_t;
  typedef cameraSystem_t<3> cameraSystem3_t;

#endif


} /* namespace vision */





















#if(0)
/*****************************************************************************/
// reproject
/*****************************************************************************/
template<typename T3D, class ... args>
void reproject(const cv::Point3_<T3D> & point3D, args & ... list) { reproject(point3D, & list ...); }


template<typename T3D, class ... args>
void reproject(const cv::Point3_<T3D> & point3D, args * ... list) {
  
  static_assert(sizeof...(list)==N, "Wrong params number");
  
  static_assert(allsame<args ... >::value, "Params must have same types");
  
  typedef typename std::tuple_element<0, std::tuple<args...> >::type T;
  
  T * points2D = new T[sizeof...(list)]{ * list ... };
  
  for(uint32_t i=0; i<N; ++i)
    vision::reproject(point3D, points2D[i], cameras[i].getProjectionalMatrix());
  
  delete[] points2D;
  
}
#endif


#endif /* _COBBS_VISION_CAMERASYSTEM_H_ */
