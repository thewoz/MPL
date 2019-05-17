/*
 * MIT License
 *
 * Copyright © 2017
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

#ifndef _H_MPL_PARAMS_H_
#define _H_MPL_PARAMS_H_

#include <cstdlib>
#include <cstdio>

#include <string>
#include <vector>
#include <any>
#include <variant>

#include <opencv2/opencv.hpp>


/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
//  class param_base_t {
//
//  public:
//
//    enum type_t {INT, BOOL, REAL, STRING, IMAGE, OPTIONS, LIST, NONE};
//
//    type_t type;
//
//  };
  
  class param_base_t { };
  
  template <class T>
  class param_t : public param_base_t {
    
  public:
    
    T value;
    
    T & get()  { return value; }
    
    
  };
  
  typedef param_t<int>         int_t;
  typedef param_t<double>      real_t;
  typedef param_t<bool>        bool_t;
  typedef param_t<cv::Mat>     image_t;
  typedef param_t<std::string> string_t;
  typedef param_t<cv::Mat>     mat_t;
  typedef param_t<cv::Point2f>     point2d_t;
  typedef param_t<cv::Point3f>     point3d_t;
  
  
  class options_t : public param_t<std::string>{
    
  public:

    std::vector<std::string> options;
    
    
    template<class... args>
    options_t(args... _values) {
      
      options =  {_values ... };
      
      value = "";
      
      //type = type_t::OPTIONS;
      
    }
    
    
    
  };
  
  
  template <class T>
  class range_t : public param_t<T> {
    
  public:
    
    T _min;
    T _max;
    
    range_t(T defaultValue, T min, T max) {
      
      param_t<T>::value = defaultValue;
      
      _min = min;
      _max = max;
      
    }
    
    bool check() {
      
      if(param_t<T>::value >= _min && param_t<T>::value <= _max) return true;
      else return false;
      
      abort();
      
    }
    
  };
  
  
  typedef range_t<int>    int_range_t;
  typedef range_t<double> real_range_t;
  
  typedef param_t<std::vector<int>>         int_vector_t;
  typedef param_t<std::vector<double>>      real_vector_t;
  typedef param_t<std::vector<cv::Mat>>     img_vector_t;
  typedef param_t<std::vector<std::string>> string_vector_t;
  typedef param_t<std::vector<cv::Point2f>> point2d_vector_t;
  typedef param_t<std::vector<cv::Point3f>> point3d_vector_t;
  
  template <class T>
  using vector_t = param_t<std::vector<T>>;
  
  
  class params_t {
    
  public:
    
    std::map<std::string, param_base_t *> params;
    
    
    param_base_t * & operator [] (const std::string & key) {
      
      return params[key];
      
    }
    
    template <class T>
    T & get(const std::string & key) {
      
      try {
        
        param_base_t * param = params.at(key);
        
        return ((param_t<T> *)(param))->get();
        
      } catch (std::out_of_range) {
        
        fprintf(stderr, "Parameters %s not found\n", key.c_str());
        abort();
        
      }
      
    }
    
  };
  
  
#if(0)
  
  /*****************************************************************************/
  // param_t
  /*****************************************************************************/
  //template <class T>
  class param_t {

    
  public:

    enum type_t {INT, BOOL, REAL, STRING, IMAGE, OPTIONS, LIST, NONE};
    
    type_t type;
    
    bool check();
    
  };
  
  /*****************************************************************************/
  // int_t
  /*****************************************************************************/
  class int_t : public param_t{
    
  public:
    
    int value;
    
    int _min;
    int _max;
    
    int_t() {
      
      value = 0;
      
      _min = std::numeric_limits<int>::min();
      _max = std::numeric_limits<int>::max();
      
      type = type_t::INT;
      
    }
    
    int_t(int min, int max) {
      
      value = 0;
      
      _min = min;
      _max = max;
      
      type = type_t::INT;
      
    }

    
    int_t(int _value, int min, int max) {
      
      value = _value;
      
      _min = min;
      _max = max;
      
      type = type_t::INT;
      
    }
    
    
    inline bool check() const {
      
      if(value > _max || value < _min) return false;
      else return true;
      
      return true;
      
    }
    
    inline int min() const { return _min; }
    
    inline int max() const { return _max; }

    
  };
  
  /*****************************************************************************/
  // real_t
  /*****************************************************************************/
  class real_t : public param_t {
    
  public:
    
    double value;
    
    double min;
    double max;
    
    real_t(double _value, double _min = std::numeric_limits<double>::min(), double _max = std::numeric_limits<double>::max()) {
      
      value = _value;
      
      min = _min;
      max = _max;
      
      type = type_t::REAL;
      
    }
    
  };
  
  /*****************************************************************************/
  // bool_t
  /*****************************************************************************/
  class bool_t : public param_t {
    
  public:
    
    bool value;
    
    bool_t(bool _value = false) {
      
      value = _value;
      
      type = type_t::BOOL;
      
    }
    
  };
  
  /*****************************************************************************/
  // image_t
  /*****************************************************************************/
  class image_t : public param_t {
    
  public:
    
    cv::Mat value;
    
    image_t(cv::Mat _value = cv::Mat()) {
      
      value = _value;
      
      type = type_t::IMAGE;
      
    }
    
  };
  
  /*****************************************************************************/
  // string_t
  /*****************************************************************************/
  class string_t : public param_t {
    
  public:
    
    std::string value;
    
    string_t(std::string _value = "") {
      
      value = _value;
      
      type = type_t::STRING;
      
    }
    
  };
  

  
  /*****************************************************************************/
  // options_t
  /*****************************************************************************/
  class options_t : public param_t {
    
  public:
    
    std::vector<std::string> values;
    
    std::string selected;
    
    template<class... args>
    options_t(args... _values) {
      
      values =  {_values ... };
      
      selected = "";
      
      type = type_t::OPTIONS;
      
    }
    
  };
  
  /*****************************************************************************/
  // list_t
  /*****************************************************************************/
  template <class T>
  class list_t : public param_t {
    
  public:
    
    std::vector<T> values;
    
    template<class... args>
    list_t(args... _values) {
      
      values =  {_values ... };
      
      // type = type_t::LIST;
      
    }
    
    inline T operator [] (size_t i) { return values[i]; }
    
    inline const T operator [] (size_t i) const { return values[i]; }

    
    inline size_t size() const { return values.size(); }
    
  };
  
  
  /*****************************************************************************/
  // params_t
  /*****************************************************************************/
  class params_t {
    
  public:
    
    std::map<std::string, param_t *> params;
    
  public:
    
    mpl::param_t * & operator [] (const std::string & key) {
      
      return params[key];
      
    }
    
    int max(const std::string & key) {
      
      try {
        
        param_t * param = params.at(key);
        
        if(param->type == mpl::param_t::type_t::INT) {
          
          return ((mpl::int_t*)(param))->max();
          
        } else if(param->type == mpl::param_t::type_t::LIST) {
          
          return true;
          
        }
        
        //if(param->type == mpl::param_base_t::type_t::LIST)    return ((list_t<T>*)param)->values;
        
      } catch (std::out_of_range) {
        
        fprintf(stderr, "Parameters %s not found\n", key.c_str());
        abort();
        
      }
      
      abort();
      
    }
    
    int min(const std::string & key) {
      
      try {
        
        param_t * param = params.at(key);
        
        if(param->type == mpl::param_t::type_t::INT) {
          
          return ((mpl::int_t*)(param))->min();
          
        } else if(param->type == mpl::param_t::type_t::LIST) {
          
          return true;
          
        }
        
        //if(param->type == mpl::param_base_t::type_t::LIST)    return ((list_t<T>*)param)->values;
        
      } catch (std::out_of_range) {
        
        fprintf(stderr, "Parameters %s not found\n", key.c_str());
        abort();
        
      }
      
      abort();
      
    }
    
    bool check(const std::string & key) {
     
      try {
        
        param_t * param = params.at(key);
        
        if(param->type == mpl::param_t::type_t::INT) {
          
          return ((mpl::int_t*)(param))->check();
          
        } else if(param->type == mpl::param_t::type_t::LIST) {
          
          return true;
          
        }
        
        //if(param->type == mpl::param_base_t::type_t::LIST)    return ((list_t<T>*)param)->values;
        
      } catch (std::out_of_range) {
        
        fprintf(stderr, "Parameters %s not found\n", key.c_str());
        abort();
        
      }
      
      abort();
      
    }
    
    template <class T>
    const T & get(const std::string & key) {
      
      try {
        
        param_t * param = params.at(key);
        
        if(param->type == mpl::param_t::type_t::INT) {
                    
          return ((int_t*)(param))->value;
        
        } else if(param->type == mpl::param_t::type_t::STRING) {
          
          return ((string_t*)(param))->value;
          
        } else if(param->type == mpl::param_t::type_t::LIST) {
          
          return ((list_t<T>*)(param))->values;
          
        }
          
        //if(param->type == mpl::param_base_t::type_t::LIST)    return ((list_t<T>*)param)->values;

      } catch (std::out_of_range) {
        
        fprintf(stderr, "Parameters %s not found\n", key.c_str());
        abort();
        
      }
      
    }
    
    template <class T>
    T & operator () (const std::string & key) {
      
      try {
        
        param_t * param = params.at(key);
        
        if(param->type == mpl::param_t::type_t::INT) {
          
          return ((int_t*)(param))->value;
          
        } else if(param->type == mpl::param_t::type_t::LIST) {
          
          return ((list_t<T>*)(param))->values;
          
        }
        
        //if(param->type == mpl::param_base_t::type_t::LIST)    return ((list_t<T>*)param)->values;
        
      } catch (std::out_of_range) {
        
        fprintf(stderr, "Parameters %s not found\n", key.c_str());
        abort();
        
      }
      
    }
    
    
    template <class T>
    const mpl::list_t<T> & getList(const std::string & key) {
      
      try {
        
        param_t * param = params.at(key);
          
        return ((list_t<T>*)(param));
        
      } catch (std::out_of_range) {
        
        fprintf(stderr, "Parameters %s not found\n", key.c_str());
        abort();
        
      }
      
    }
    
    
  };
  
  
//
//  using T = std::decay_t<decltype(arg)>;
//  if constexpr (std::is_same_v<T, int>)
//    std::cout << "int with value " << arg << '\n';
//    else if constexpr (std::is_same_v<T, long>)
//    std::cout << "long with value " << arg << '\n';
//    else if constexpr (std::is_same_v<T, double>)
//    std::cout << "double with value " << arg << '\n';
//    else if constexpr (std::is_same_v<T, std::string>)
//    std::cout << "std::string with value " << std::quoted(arg) << '\n';
//    else
//    static_assert(always_false<T>::value, "non-exhaustive visitor!");
//
  
#endif
  
} /* namespace mpl */

#endif /* _H_MPL_PARAMS_H_ */


#if(0)

/*****************************************************************************/
// setIntParam
/*****************************************************************************/
int & setIntParam(std::string key){
  
  try {
    
    return ((int_t*)inputs.at(key))->value;
    
  } catch (std::out_of_range) {
    
    fprintf(stderr, "Parameters %s not found in %s algoritm\n", key.c_str(), name.c_str());
    abort();
    
  }
  
}

/*****************************************************************************/
// setRealParam
/*****************************************************************************/
double & setRealParam(std::string key){
  
  try {
    
    return ((real_t*)inputs.at(key))->value;
    
  } catch (std::out_of_range) {
    
    fprintf(stderr, "Parameters %s not found in %s algoritm\n", key.c_str(), name.c_str());
    abort();
    
  }
  
}

/*****************************************************************************/
// setBoolParam
/*****************************************************************************/
bool & setBoolParam(std::string key){
  
  try {
    
    return ((bool_t*)inputs.at(key))->value;
    
  } catch (std::out_of_range) {
    
    fprintf(stderr, "Parameters %s not found in %s algoritm\n", key.c_str(), name.c_str());
    abort();
    
  }
  
}

/*****************************************************************************/
// setImageParam
/*****************************************************************************/
cv::Mat & setImageParam(std::string key){
  
  try {
    
    return ((image_t*)inputs.at(key))->value;
    
  } catch (std::out_of_range) {
    
    fprintf(stderr, "Parameters %s not found in %s algoritm\n", key.c_str(), name.c_str());
    abort();
    
  }
  
}

/*****************************************************************************/
// getImageParam
/*****************************************************************************/
cv::Mat getImageParam(std::string key){
  
  try {
    
    return ((image_t*)outputs.at(key))->value;
    
  } catch (std::out_of_range) {
    
    fprintf(stderr, "Parameters %s not found in %s algoritm\n", key.c_str(), name.c_str());
    abort();
    
  }
  
}

/*****************************************************************************/
// setStringParam
/*****************************************************************************/
std::string & setStringParam(std::string key){
  
  try {
    
    return ((string_t*)inputs.at(key))->value;
    
  } catch (std::out_of_range) {
    
    fprintf(stderr, "Parameters %s not found in %s algoritm\n", key.c_str(), name.c_str());
    abort();
    
  }
  
}

/*****************************************************************************/
// setOptionsParam
/*****************************************************************************/
std::string & setOptionsParam(std::string key){
  
  try {
    
    return ((options_t*)inputs.at(key))->selected;
    
  } catch (std::out_of_range) {
    
    fprintf(stderr, "Parameters %s not found in %s algoritm\n", key.c_str(), name.c_str());
    abort();
    
  }
  
}

#endif
