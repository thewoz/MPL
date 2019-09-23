/*
 * MIT License
 *
 * Copyright © 2019
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
    static void init(const std::string & filePath) {
      
      load(filePath);
      
    }
    
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
    // asConfigure
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
    template <class T>
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
  static void load(const std::string & filePath) {
    
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


} /* namespace */

#endif /* _H_MPL_CONFIGURE_H_ */
