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

#ifndef _H_MPL_CONFIGURE_H_
#define _H_MPL_CONFIGURE_H_


#include <stdlib.h>
#include <stdio.h>

#include <climits>
#include <cstring>

#include <map>
#include <string>
#include <vector>
#include <unordered_map>

#include <sstream>
#include <iostream>
#include <fstream>

#include <sys/types.h>

#include <mpl/stdlib.hpp>
#include <mpl/stdio.hpp>

#include <opencv2/opencv.hpp>

/*****************************************************************************/
// namespace
/*****************************************************************************/
namespace mpl {
  
  typedef std::map<std::string, std::string>  dictionary_t;
  typedef std::vector<std::map<std::string, dictionary_t>> dictionaries_t;
  
  //****************************************************************************//
  // asConfigure
  //****************************************************************************//
  class configure {
    
  private:
    
    //static std::unordered_map<pid_t,dictionaries_t> dictionaries;
    static dictionaries_t dictionaries;
    
    
  public:
    
    //****************************************************************************//
    // init
    //****************************************************************************//
    static void init(std::string filePath) { load(filePath); }
    
    //****************************************************************************//
    // init
    //****************************************************************************//
    static void init(int argc, char** argv) {
      
      load(argv[1]);
      
      std::string dictionary = "GLOBAL";
      
      std::string tmpLine;
      
      int index = 0;
      
#ifdef _OPENMP
      
#pragma omp critical
      {
#endif
        
        for(int i = 2; i < argc ; i++){
          
          tmpLine.assign(argv[i]);
          
          parseLine(tmpLine, index, &dictionary);
          
        }
        
#ifdef _OPENMP
        
      }
      
#endif
      
    }
    
    //****************************************************************************//
    // asConfigure
    //****************************************************************************//
    static void print(FILE * & output = stdout) {
      
      int index = 0;
      
#ifdef _OPENMP
      
#pragma omp critical
      {
        
        int index = omp_get_thread_num();
        
        if(((int)dictionaries.size()-1) < index) return
          
#endif
          
          std::map<std::string, dictionary_t>::const_iterator it_dic;
        dictionary_t::const_iterator it_param;
        
        for(it_dic=dictionaries[index].begin(); it_dic!=dictionaries[index].end(); it_dic++) {
          
          fprintf(output,"#==============================================================================\n");
          fprintf(output,"# @ %s \n", (*it_dic).first.c_str() );
          fprintf(output,"#==============================================================================\n");
          
          for(it_param=(*it_dic).second.begin(); it_param!=(*it_dic).second.end(); it_param++)
            fprintf(output, "%s = %s\n", it_param->first.c_str(), it_param->second.c_str());
          
          fprintf(output,"\n");
          
        }
        
#ifdef _OPENMP
        
      }
      
#endif
      
    }
    
    
    //****************************************************************************//
    // isDefined
    //****************************************************************************//
    static bool isDefined(const char * key, const char * dictionary = "GLOBAL") {
      
      std::map<std::string, dictionary_t>::const_iterator itrDics;
      
      int index = 0;
      
#ifdef _OPENMP
      
#pragma omp critical
      {
        
        int index = omp_get_thread_num();
        
        if(((int)dictionaries.size()-1) < index) return
          
#endif
          
          itrDics = dictionaries[index].find(dictionary);
        
        if(itrDics == dictionaries[index].end()) {
          
          return false;
          
        } else {
          
          std::map<std::string,std::string>::const_iterator itrKey;
          
          if((itrKey=itrDics->second.find(key)) == itrDics->second.end()) {
            
            return false;
            
          } else {
            
            //printf("%s\n", itrKey->second.c_str());
            
            if(itrKey->second.empty()) return false;
            
            if(itrKey->second.c_str()[0]=='\0') return false;
            
            //if(atoi(itrKey->second.c_str())==0) return false;
            
            //if(atof(itrKey->second.c_str())==0) return false;
            
            return true;
            
          }
          
        }
        
#ifdef _OPENMP
        
      }
      
#endif
      
    }
    
    //****************************************************************************//
    // haveKey
    //****************************************************************************//
    static bool haveKey(const char * key, const char * dictionary = "GLOBAL") {
      
      std::map <std::string, dictionary_t >::const_iterator itrDics;
      
      int index = 0;
      
#ifdef _OPENMP
      
#pragma omp critical
      {
        
        int index = omp_get_thread_num();
        
        if(((int)dictionaries.size()-1) < index) return
          
#endif
          
          itrDics = dictionaries[index].find(dictionary);
        
        if(itrDics == dictionaries[index].end()) {
          
          return false;
          
        } else {
          
          if(itrDics->second.find(key) == itrDics->second.end())
            return false;
          else
            return true;
          
        }
        
#ifdef _OPENMP
        
      }
      
#endif
      
    }
    
    //****************************************************************************//
    // haveDictionary
    //****************************************************************************//
    static bool haveDictionary(const char * dictionary) {
      
      int index = 0;
      
#ifdef _OPENMP
      
#pragma omp critical
      {
        
        int index = omp_get_thread_num();
        
        if(((int)dictionaries.size()-1) < index) return
          
#endif
          
          if(dictionaries[index].find(dictionary) == dictionaries[index].end())
            return false;
          else
            return true;
        
#ifdef _OPENMP
        
      }
      
#endif
      
    }
    
#if(0)
    
    //****************************************************************************//
    // setParam
    //****************************************************************************//
    template <class T>
    static void setParam(const char * key, const char value[], const char * dictionary = "GLOBAL") {
      
      std::map <std::string, dictionary_t >::iterator itrDics;
      
      int index = 0;
      
#ifdef _OPENMP
      
#pragma omp critical
      {
        
        int index = omp_get_thread_num();
        
        if(((int)dictionaries.size()-1) < index) return
          
#endif
          
        itrDics = dictionaries[index].find(dictionary);
        
        if(itrDics == dictionaries[index].end() ) {
          
          fprintf(stderr, "\n\nYou are trying to get a dictionary \"%s\" that is not defined into the configuration file.\n\n", dictionary);
          exit(1);
          
        } else {
          
          std::stringstream oss;
          oss << value;
                    
          itrDics->second[key] = oss.str();
          
        }
        
#ifdef _OPENMP
        
      }
      
#endif
      
    }
    
#endif
    
    
    //****************************************************************************//
    // setParam
    //****************************************************************************//
    template <class T>
    static void setParam(const char * key, const T & value, const char * dictionary = "GLOBAL") {
      
      std::map <std::string, dictionary_t >::iterator itrDics;
      
      int index = 0;
      
#ifdef _OPENMP
      
#pragma omp critical
      {
        
        int index = omp_get_thread_num();
        
        if(((int)dictionaries.size()-1) < index) return
          
#endif
          
          itrDics = dictionaries[index].find(dictionary);
        
        if(itrDics == dictionaries[index].end() ) {
          
          fprintf(stderr, "\n\nYou are trying to get a dictionary \"%s\" that is not defined into the configuration file.\n\n", dictionary);
          exit(1);
          
        } else {
          
          std::stringstream oss;
          oss << value;
          
          itrDics->second[key] = oss.str();
          
          //itrDics->second[key] = std::string(value);
          
        }
        
#ifdef _OPENMP
        
      }
      
#endif
      
    }
        
    //****************************************************************************//
    // setParam
    //****************************************************************************//
    template <class T>
    static void setParam(const char * key, const cv::Point_<T> & value, const char * dictionary = "GLOBAL") {
      
      std::map <std::string, dictionary_t >::iterator itrDics;
      
      int index = 0;
      
#ifdef _OPENMP
      
#pragma omp critical
      {
        
        int index = omp_get_thread_num();
        
        if(((int)dictionaries.size()-1) < index) return
          
#endif
          
          itrDics = dictionaries[index].find(dictionary);
        
        if(itrDics == dictionaries[index].end() ) {
          
          fprintf(stderr, "\n\nYou are trying to get a dictionary \"%s\" that is not defined into the configuration file.\n\n", dictionary);
          exit(1);
          
        } else {
          
          char str[1024];
          
          sprintf(str, "%e,%e", value.x, value.y);
          
          itrDics->second[key] = std::string(str);
          
        }
        
#ifdef _OPENMP
        
      }
      
#endif
      
    }
    
    //****************************************************************************//
    // isEqual
    //****************************************************************************//
    static bool isEqual(const std::string & key, const std::string & value, const char * dictionary = "GLOBAL"){
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key.c_str(), dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        exit(0);
        
      } else {
        
        if(itrKeys->second.compare(value) == 0) return true;
        else return false;
        
      }
        
    }
    
    static int getInt(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        exit(0);
        
      } else {
        
        std::stringstream iss(itrKeys->second);
        int value;
        iss >> value;
        return value;
                
      }
      
    }
    
    static std::string getString(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        exit(0);
        
      } else {
        
        std::stringstream iss(itrKeys->second);
        std::string value;
        iss >> value;
        return value;
                
      }
      
    }
    
    static std::pair<int,int> getRange(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        exit(0);
        
      } else {
        
        std::vector<std::string> tokens;
        
        std::parse(itrKeys->second, "-", tokens);
        
        if(tokens.size() != 2) {
          fprintf(stderr, "error in parse '%s' in configure::getRange()\n", itrKeys->second.c_str());
          abort();
        }
        
        return std::pair<int,int>(std::stoi(tokens[0]),std::stoi(tokens[1]));
        
      }
      
    }
    
    
    static double getDouble(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        exit(0);
        
      } else {
        
        std::stringstream iss(itrKeys->second);
        double value;
        iss >> value;
        return value;
        
      }
      
    }
    
    static bool getBool(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        exit(0);
        
      } else {
        
        //printf("%s %s\n", itrKeys->first.c_str(), itrKeys->second.c_str());
        
        return (itrKeys->second.compare("ON") == 0);
        
      }
      
    }
    
    
    
    
    //****************************************************************************//
    // addKey
    //****************************************************************************//
    static void addKey(const char * key, const char * value, const char * dictionary = "GLOBAL") {
      
      std::map <std::string, dictionary_t >::iterator itrDics;
      
      int index = 0;
      
#ifdef _OPENMP
      
#pragma omp critical
      {
        
        int index = omp_get_thread_num();
        
        if(((int)dictionaries.size()-1) < index) return
          
#endif
          
          itrDics = dictionaries[index].find(dictionary);
        
        if(itrDics == dictionaries[index].end() ) {
          
          fprintf(stderr, "\n\nYou are trying to get a dictionary \"%s\" that is not defined into the configuration file.\n\n", dictionary);
          exit(1);
          
        } else {
          
          itrDics->second[key] = std::string(value);
          
        }
        
#ifdef _OPENMP
        
      }
      
#endif
      
    }
    
    
    //****************************************************************************//
    // getParam
    //****************************************************************************//
    template <class T = std::string>
    static T getParam(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nyou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        exit(EXIT_FAILURE);
        
      } else {
        
        std::stringstream iss(itrKeys->second);
        T value;
        iss >> value;
        return value;
        
      }
      
#ifdef _OPENMP
      
    }
    
#endif
    
  }
  
  
  //****************************************************************************//
  // addDictionary
  //****************************************************************************//
  static const dictionary_t & getDictionary(const char * dictionary) {
    
    int index = 0;
    
#ifdef _OPENMP
    
#pragma omp critical
    {
      
      int index = omp_get_thread_num();
      
      if(((int)dictionaries.size()-1) < index) return
        
#endif
        
        std::map <std::string, dictionary_t >::const_iterator itrDics;
      
      itrDics = dictionaries[index].find(dictionary);
      
      if(itrDics == dictionaries[index].end() ) {
        
        fprintf(stderr, "\n\nyou are trying to get a dictionary \"%s\" that is not defined into the configuration file.\n\n", dictionary);
        exit(EXIT_FAILURE);
        
      } else {
        
        return itrDics->second;
        
      }
      
#ifdef _OPENMP
      
    }
    
#endif
    
  }
  
  
  /*
   asConfigure get_as_parseConf(const char * dictionary, const char * new_dictionary_name = NULL) const
   {
   
   char name[PATH_MAX];
   
   if(new_dictionary_name == NULL){
   strcpy(name, "GLOBAL");
   } else {
   strcpy(name, new_dictionary_name);
   }
   
   const dictionary_t & dict = get_dictionary(dictionary);
   
   asConfigure new_configure;
   
   new_configure.add_dictionary(dict, name);
   
   return new_configure;
   
   }
   */
  
  //****************************************************************************//
  // addDictionary
  //****************************************************************************//
  static void addDictionary(const dictionary_t & dictionary, const char * name = "GLOBAL") {
    
    int index = 0;
    
#ifdef _OPENMP
    
#pragma omp critical
    {
      
      int index = omp_get_thread_num();
      
      if(((int)dictionaries.size()-1) < index) return
        
#endif
        
        if(dictionaries[index].find(name) == dictionaries[index].end()){
          
          dictionaries[index][name].clear();
          dictionaries[index][name] = dictionary;
          
        } else {
          
          fprintf(stderr, "warning you are overwriting the dictionary \"%s\" \n", name);
          
          dictionaries[index][name] = dictionary;
          
        }
      
#ifdef _OPENMP
      
    }
    
#endif
    
  }
  
  //asConfigure & operator = (const asConfigure & configure) { dictionaries = configure.dictionaries; return *this;}
  
private:
  
  configure(){ };
  
  //****************************************************************************//
  // load
  //****************************************************************************//
  static void load(std::string filePath) {
    
    mpl::io::expandPath(filePath);
    
    int index = 0;
    
#ifdef _OPENMP
    
#pragma omp critical
    {
      
      int index = omp_get_thread_num();
      
#endif
      
      dictionaries.resize(index+1);
      
      dictionaries[index] = std::map<std::string, dictionary_t>();
      
#ifdef _OPENMP
      
    }
    
#endif
    
    std::string dictionary = "GLOBAL";
    
    std::string tmpLine;
    
    std::ifstream infile;
    
    infile.open(filePath);
    
    if(infile.is_open()) {
      
      while(!infile.eof()) {
        
        getline(infile, tmpLine);
        
        parseLine(tmpLine, index, &dictionary);
        
      }
      
      infile.close();
      
    } else {
      fprintf(stderr,"\nerror in opening the configuration file \"%s\"\n", filePath.c_str());
      exit(EXIT_FAILURE);
    }
    
  }
  
  //****************************************************************************//
  // trimSpace
  //****************************************************************************//
  static void trimSpace(const std::string & str,
                        size_t & start,
                        size_t & end){
    
    for(; end>=start; --end)
      if(isgraph(str[end]))
        break;
    
    ++end;
    
    for(; start<end; ++start)
      if(isgraph(str[start]))
        break;
    
  }
  
  
  //****************************************************************************//
  // parseLine
  //****************************************************************************//
  static void parseLine(std::string tmpLine, uint32_t index, std::string * dictionary) {
    
    if(tmpLine.empty()) return;
    
    if(tmpLine[0] == '#') return;
    
    if(tmpLine[0] == '@') {
      size_t start_value = 1;
      size_t end_value = tmpLine.length();
      trimSpace(tmpLine, start_value, end_value);
      *dictionary = tmpLine.substr(start_value, end_value - start_value);
      return;
    }
    
    size_t pos = tmpLine.find("=");
    
    if(pos == std::string::npos) return;
    
    size_t start_key = 0;
    size_t end_key = tmpLine.substr(0, pos - 1).length();
    
    trimSpace(tmpLine, start_key, end_key);
    
    size_t start_value = pos + 1;
    size_t end_value = tmpLine.length();
    
    trimSpace(tmpLine, start_value, end_value);
    
    dictionaries[index][*dictionary][tmpLine.substr(start_key, end_key - start_key)] = tmpLine.substr(start_value, end_value - start_value);
    
  }
  
};

dictionaries_t configure::dictionaries = dictionaries_t();

//****************************************************************************//
// specialization of the template function getParam()
//****************************************************************************//
template <>
bool configure::getParam(const char * key, const char * dictionary) {
  
  const dictionary_t & dict = getDictionary(dictionary);
  
  std::map <std::string, std::string>::const_iterator itrKeys;
  
  if((itrKeys = dict.find(key)) == dict.end()) {
    
    fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
    fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
    exit(0);
    
  } else {
    
    //printf("%s %s\n", itrKeys->first.c_str(), itrKeys->second.c_str());
    
    return (itrKeys->second.compare("ON") == 0);
    
  }
  
}

//****************************************************************************//
// specialization of the template function getParam()
//****************************************************************************//
template <>
const char* configure::getParam(const char * key, const char * dictionary) {
  
  const dictionary_t & dict = getDictionary(dictionary);
  
  std::map <std::string, std::string>::const_iterator itrKeys;
  
  if( (itrKeys = dict.find(key)) == dict.end()) {
    
    fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
    fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
    exit(0);
    
  } else {
    
    return itrKeys->second.c_str();
    
  }
  
}

//****************************************************************************//
// specialization of the template function getParam()
//****************************************************************************//
template <>
cv::Point2d configure::getParam(const char * key, const char * dictionary) {
  
  const dictionary_t & dict = getDictionary(dictionary);
  
  std::map <std::string, std::string>::const_iterator itrKeys;
  
  if((itrKeys = dict.find(key)) == dict.end()) {
    
    fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
    fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
    exit(0);
    
  } else {
    
    std::vector<std::string> tokens;
    
    std::parse(itrKeys->second, ",", tokens);
    
    if(tokens.size() != 2) {
      fprintf(stderr, "error in parse point config entry\n\n");
      exit(0);
    }
    
    return cv::Point2d(atof(tokens[0].c_str()),atof(tokens[1].c_str()));
    
  }
  
}


} /* namespace */

#endif /* _H_MPL_CONFIGURE_H_ */
