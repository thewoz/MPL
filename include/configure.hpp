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

//*****************************************************************************/
// namespace
//*****************************************************************************/
namespace mpl {

  typedef std::map<std::string, std::string>  dictionary_t;
  typedef std::map<std::string, dictionary_t> dictionaries_t;

  //****************************************************************************//
  // asConfigure
  //****************************************************************************//
  class configure {
    
  private:
    
    static dictionaries_t dictionaries;
    
  public:
    
    //****************************************************************************//
    // init()
    //****************************************************************************//
    static void init(std::string filePath) { load(filePath); }
    
    //****************************************************************************//
    // update()
    //****************************************************************************//
    static void update(int argc, const char * argv[]) {
      
      std::string dictionary = "GLOBAL";
      
      std::string arg;
      
      // ciclo su i vari argomenti
      for(int i=2; i<argc; ++i){
        
        arg.assign(argv[i]);
        
        // se inizia con --configure="
        if(arg.rfind("--configure=\"", 0) == 0) {
          
          size_t inizio = arg.find("\"");             // Trova il primo apice
          size_t fine   = arg.find("\"", inizio + 1); // Trova il secondo apice

          // se trovo i due apici
          if(inizio != std::string::npos && fine != std::string::npos) {
            
            // estraggo la sotto stringa tra gli apici
            std::string substr = arg.substr(inizio + 1, fine - inizio - 1);
            
            // divido i vari update
            std::vector<std::string> tokens = std::parse(substr, ";");
            
            // li parserizzo
            for(int i=0; i<tokens.size(); ++i)
              parseLine(tokens[i], &dictionary);
            
          }
          
        }
        
      }
        
    }
    
    //****************************************************************************//
    // print()
    //****************************************************************************//
    static void print(FILE * & output = stdout) {
      
      std::map<std::string, dictionary_t>::const_iterator it_dic;
      dictionary_t::const_iterator it_param;
      
      for(it_dic=dictionaries.begin(); it_dic!=dictionaries.end(); it_dic++) {
        
        fprintf(output,"#==============================================================================\n");
        fprintf(output,"# @ %s \n", (*it_dic).first.c_str() );
        fprintf(output,"#==============================================================================\n");
        
        for(it_param=(*it_dic).second.begin(); it_param!=(*it_dic).second.end(); it_param++)
          fprintf(output, "%s = %s\n", it_param->first.c_str(), it_param->second.c_str());
        
        fprintf(output,"\n");
        
      }
      
    }
    
    //****************************************************************************//
    // isDefined
    //****************************************************************************//
    static bool isDefined(const char * key, const char * dictionary = "GLOBAL") {
      
      std::map<std::string, dictionary_t>::const_iterator itrDics;
      
      itrDics = dictionaries.find(dictionary);
      
      if(itrDics == dictionaries.end()) {
        
        return false;
        
      } else {
        
        std::map<std::string,std::string>::const_iterator itrKey;
        
        if((itrKey=itrDics->second.find(key)) == itrDics->second.end()) {
          
          return false;
          
        } else {
                    
          if(itrKey->second.empty()) return false;
          
          if(itrKey->second.c_str()[0]=='\0') return false;
          
          return true;
          
        }
        
      }
      
    }
    
    //****************************************************************************//
    // haveKey
    //****************************************************************************//
    static bool haveKey(const char * key, const char * dictionary = "GLOBAL") {
      
      std::map<std::string, dictionary_t >::const_iterator itrDics;
      
      itrDics = dictionaries.find(dictionary);
      
      if(itrDics == dictionaries.end()) {
        
        return false;
        
      } else {
        
        if(itrDics->second.find(key) == itrDics->second.end()) return false;
        else return true;
        
      }
      
    }
    
    //****************************************************************************//
    // haveDictionary
    //****************************************************************************//
    static bool haveDictionary(const char * dictionary) {
      
      if(dictionaries.find(dictionary) == dictionaries.end())  return false;
      else return true;
      
    }
    
    //****************************************************************************//
    // setParam
    //****************************************************************************//
    template <class T>
    static void setParam(const char * key, const T & value, const char * dictionary = "GLOBAL") {
      
      std::map<std::string, dictionary_t >::iterator itrDics;
      
      itrDics = dictionaries.find(dictionary);
      
      if(itrDics == dictionaries.end() ) {
        
        fprintf(stderr, "\n\nYou are trying to get a dictionary \"%s\" that is not defined into the configuration file.\n\n", dictionary);
        abort();
        
      } else {
        
        std::stringstream oss;
        oss << value;
        
        itrDics->second[key] = oss.str();
        
      }
      
    }
    
    //****************************************************************************//
    // setParam
    //****************************************************************************//
    template <class T>
    static void setParam(const char * key, const cv::Point_<T> & value, const char * dictionary = "GLOBAL") {
      
      std::map<std::string, dictionary_t >::iterator itrDics;
      
      itrDics = dictionaries.find(dictionary);
      
      if(itrDics == dictionaries.end() ) {
        
        fprintf(stderr, "\n\nYou are trying to get a dictionary \"%s\" that is not defined into the configuration file.\n\n", dictionary);
        abort();
        
      } else {
        
        char str[PATH_MAX];
        
        snprintf(str, PATH_MAX, "%e,%e", value.x, value.y);
        
        itrDics->second[key] = std::string(str);
        
      }
      
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
        abort();
        
      } else {
        
        if(itrKeys->second.compare(value) == 0) return true;
        else return false;
        
      }
      
    }
    
    //****************************************************************************//
    // getInt
    //****************************************************************************//
    static int getInt(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        abort();
        
      } else {
        
        std::stringstream iss(itrKeys->second);
        int value;
        iss >> value;
        return value;
        
      }
      
    }
    
    //****************************************************************************//
    // getString
    //****************************************************************************//
    static std::string getString(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        abort();
        
      } else {
        
        std::stringstream iss(itrKeys->second);
        std::string value;
        iss >> value;
        return value;
        
      }
      
    }
    
    //****************************************************************************//
    // getRange
    //****************************************************************************//
    static std::pair<int,int> getRange(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        abort();
        
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
    
    //*****************************************************************************/
    // getList()
    //*****************************************************************************/
    static std::vector<std::string> getList(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        abort();
        
      } else {
        
        std::vector<std::string> tokens;
        
        std::parse(itrKeys->second, ",", tokens);
        
        return tokens;
        
      }
      
    }
    
    //*****************************************************************************/
    // getDouble()
    //*****************************************************************************/
    static double getReal(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        abort();
        
      } else {
        
        std::stringstream iss(itrKeys->second);
        double value;
        iss >> value;
        return value;
        
      }
      
    }
    
    //*****************************************************************************/
    // getBool()
    //*****************************************************************************/
    static bool getBool(const char * key, const char * dictionary = "GLOBAL") {
      
      const dictionary_t & dict = getDictionary(dictionary);
      
      std::map <std::string, std::string>::const_iterator itrKeys;
      
      if((itrKeys = dict.find(key)) == dict.end()) {
        
        fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
        fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
        abort();
        
      } else {
        
        //printf("%s %s\n", itrKeys->first.c_str(), itrKeys->second.c_str());
        
        return (itrKeys->second.compare("ON") == 0);
        
      }
      
    }
    
    //****************************************************************************//
    // addKey
    //****************************************************************************//
    static void addKey(const std::string & key, const std::string & value, const char * dictionary = "GLOBAL") {
      
      std::map<std::string, dictionary_t >::iterator itrDics;
      
      itrDics = dictionaries.find(dictionary);
      
      if(itrDics == dictionaries.end() ) {
        
        fprintf(stderr, "\n\nYou are trying to get a dictionary \"%s\" that is not defined into the configuration file.\n\n", dictionary);
        abort();
        
      } else {
        
        itrDics->second[key] = std::string(value);
        
      }
      
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
        abort();
        
      } else {
        
        std::stringstream iss(itrKeys->second);
        T value;
        iss >> value;
        return value;
        
      }
      
    }
    
    //****************************************************************************//
    // addDictionary
    //****************************************************************************//
    static const dictionary_t & getDictionary(const char * dictionary) {
      
      std::map <std::string, dictionary_t >::const_iterator itrDics;
      
      itrDics = dictionaries.find(dictionary);
      
      if(itrDics == dictionaries.end() ) {
        
        fprintf(stderr, "\n\nyou are trying to get a dictionary \"%s\" that is not defined into the configuration file.\n\n", dictionary);
        abort();
        
      } else {
        
        return itrDics->second;
        
      }
      
    }
    
    //****************************************************************************//
    // addDictionary
    //****************************************************************************//
    static void addDictionary(const dictionary_t & dictionary, const char * name = "GLOBAL") {
      
      if(dictionaries.find(name) == dictionaries.end()){
        
        dictionaries[name].clear();
        dictionaries[name] = dictionary;
        
      } else {
        
        fprintf(stderr, "warning you are overwriting the dictionary \"%s\" \n", name);
        
        dictionaries[name] = dictionary;
        
      }
      
    }
    
  private:
    
    configure() { };
    
    //****************************************************************************//
    // load
    //****************************************************************************//
    static void load(std::string filePath) {
      
      mpl::io::expandPath(filePath);
            
      dictionaries = std::map<std::string, dictionary_t>();
      
      std::string dictionary = "GLOBAL";
      
      std::string tmpLine;
      
      std::ifstream infile;
      
      infile.open(filePath);
      
      if(infile.is_open()) {
        
        while(!infile.eof()) {
          
          getline(infile, tmpLine);
          
          parseLine(tmpLine, &dictionary);
          
        }
        
        infile.close();
        
      } else {
        fprintf(stderr,"\nerror in opening the configuration file \"%s\"\n", filePath.c_str());
        exit(EXIT_FAILURE);
      }
      
      addKey("CONFIGURE_FILE_PATH", filePath);
      
    }
    
    //****************************************************************************//
    // trimSpace
    //****************************************************************************//
    static void trimSpace(const std::string & str, size_t & start, size_t & end){
      
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
    static void parseLine(std::string tmpLine, std::string * dictionary) {
      
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
      
      dictionaries[*dictionary][tmpLine.substr(start_key, end_key - start_key)] = tmpLine.substr(start_value, end_value - start_value);
      
    }
    
  };

  dictionaries_t configure::dictionaries = dictionaries_t();

  //****************************************************************************//
  // specialization of the template function getParam()
  //****************************************************************************//
  template <>
  bool configure::getParam(const char * key, const char * dictionary) {
    
    const dictionary_t & dict = getDictionary(dictionary);
    
    std::map<std::string, std::string>::const_iterator itrKeys;
    
    if((itrKeys = dict.find(key)) == dict.end()) {
      
      fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
      fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
      abort();
      
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
    
    std::map<std::string, std::string>::const_iterator itrKeys;
    
    if( (itrKeys = dict.find(key)) == dict.end()) {
      
      fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
      fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
      abort();
      
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
    
    std::map<std::string, std::string>::const_iterator itrKeys;
    
    if((itrKeys = dict.find(key)) == dict.end()) {
      
      fprintf(stderr, "\n\nYou are trying to get a variable \"%s\" from the dictionary \"%s\", \n", key, dictionary);
      fprintf(stderr, "that is not defined into the configuration file. Check please!\n\n");
      abort();
      
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
