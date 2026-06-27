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

#ifndef _H_MPL_CORE_CONFIGURE_H_
#define _H_MPL_CORE_CONFIGURE_H_


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

#include <mpl/core/stdlib.hpp>
#include <mpl/core/stdio.hpp>

#include <opencv2/opencv.hpp>

//*****************************************************************************/
// namespace
//*****************************************************************************/
namespace mpl {

  typedef std::map<std::string, std::string> dictionary_t;

  //****************************************************************************/
  // asConfigure
  //****************************************************************************/
  class configure {

  private:

    static dictionary_t params;

  public:

    //****************************************************************************/
    // init()
    //****************************************************************************/
    static void init(const std::string & filePath) { load(filePath); }

    //****************************************************************************/
    // update()
    //****************************************************************************/
    static void update(int argc, const char * argv[]) {

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
            for(size_t i=0; i<tokens.size(); ++i)
              parseLine(tokens[i]);

          }

        }

      }

    }

    //****************************************************************************/
    // print()
    //****************************************************************************/
    static void print(FILE * output = stdout) {

      dictionary_t::const_iterator it_param;

      for(it_param=params.begin(); it_param!=params.end(); it_param++)
        fprintf(output, "%s = %s\n", it_param->first.c_str(), it_param->second.c_str());

    }

    //****************************************************************************/
    // isDefined
    //****************************************************************************/
    static bool isDefined(const char * key) {

      dictionary_t::const_iterator itrKey;

      if((itrKey=params.find(key)) == params.end()) {

        return false;

      } else {

        if(itrKey->second.empty()) return false;

        if(itrKey->second.c_str()[0]=='\0') return false;

        return true;

      }

    }

    //****************************************************************************/
    // haveKey
    //****************************************************************************/
    static bool haveKey(const char * key) {

      if(params.find(key) == params.end()) return false;
      else return true;

    }

    //****************************************************************************/
    // setParam
    //****************************************************************************/
    template <class T>
    static void setParam(const char * key, const T & value) {

      std::stringstream oss;
      oss << value;

      params[key] = oss.str();

    }

    //****************************************************************************/
    // setParam
    //****************************************************************************/
    template <class T>
    static void setParam(const char * key, const cv::Point_<T> & value) {

      char str[PATH_MAX];

      snprintf(str, PATH_MAX, "%e,%e", value.x, value.y);

      params[key] = std::string(str);

    }

    //****************************************************************************/
    // isEqual
    //****************************************************************************/
    static bool isEqual(const std::string & key, const std::initializer_list<std::string> & values){

      dictionary_t::const_iterator itrKeys;

      if((itrKeys = params.find(key)) == params.end()) {

        fprintf(stderr, "mpl::configure::isEqual() error: variable \"%s\" not defined\n", key.c_str());
        abort();

      }

      return std::isEqual(itrKeys->second, values);

    }

    //****************************************************************************/
    // isEqual
    //****************************************************************************/
    static bool isEqual(const std::string & key, const std::string & value){

      return isEqual(key, {value});

    }

    //****************************************************************************/
    // getInt
    //****************************************************************************/
    static int getInt(const char * key) {

      dictionary_t::const_iterator itrKeys;

      if((itrKeys = params.find(key)) == params.end()) {

        fprintf(stderr, "mpl::configure::getInt() error: variable \"%s\" not defined\n", key);
        abort();

      } else {

        std::stringstream iss(itrKeys->second);
        int value;
        iss >> value;
        return value;

      }

    }

    //****************************************************************************/
    // getString
    //****************************************************************************/
    static std::string getString(const char * key) {

      dictionary_t::const_iterator itrKeys;

      if((itrKeys = params.find(key)) == params.end()) {

        fprintf(stderr, "mpl::configure::getString() error: variable \"%s\" not defined\n", key);
        abort();

      } else {

        std::stringstream iss(itrKeys->second);
        std::string value;
        iss >> value;
        return value;

      }

    }

    //****************************************************************************/
    // getRange
    //****************************************************************************/
    static std::pair<int,int> getRange(const char * key) {

      dictionary_t::const_iterator itrKeys;

      if((itrKeys = params.find(key)) == params.end()) {

        fprintf(stderr, "mpl::configure::getRange() error: variable \"%s\" not defined\n", key);
        abort();

      } else {

        std::vector<std::string> tokens;

        std::parse(itrKeys->second, "-", tokens);

        if(tokens.size() != 2) {
          fprintf(stderr, "mpl::configure::getRange() error: cannot parse '%s'\n", itrKeys->second.c_str());
          abort();
        }

        return std::pair<int,int>(std::stoi(tokens[0]),std::stoi(tokens[1]));

      }

    }

    //*****************************************************************************/
    // getList()
    //*****************************************************************************/
    static std::vector<std::string> getList(const char * key) {

      dictionary_t::const_iterator itrKeys;

      if((itrKeys = params.find(key)) == params.end()) {

        fprintf(stderr, "mpl::configure::getList() error: variable \"%s\" not defined\n", key);
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
    static double getReal(const char * key) {

      dictionary_t::const_iterator itrKeys;

      if((itrKeys = params.find(key)) == params.end()) {

        fprintf(stderr, "mpl::configure::getReal() error: variable \"%s\" not defined\n", key);
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
    static bool getBool(const char * key) {

      dictionary_t::const_iterator itrKeys;

      if((itrKeys = params.find(key)) == params.end()) {

        fprintf(stderr, "mpl::configure::getBool() error: variable \"%s\" not defined\n", key);
        abort();

      } else {

        return (itrKeys->second.compare("ON") == 0);

      }

    }

    //****************************************************************************/
    // addKey
    //****************************************************************************/
    static void addKey(const std::string & key, const std::string & value) {

      params[key] = std::string(value);

    }

    //****************************************************************************/
    // getParam
    //****************************************************************************/
    template <class T = std::string>
    static T getParam(const char * key) {

      dictionary_t::const_iterator itrKeys;

      if((itrKeys = params.find(key)) == params.end()) {

        fprintf(stderr, "mpl::configure::getParam() error: variable \"%s\" not defined\n", key);
        abort();

      } else {

        std::stringstream iss(itrKeys->second);
        T value;
        iss >> value;
        return value;

      }

    }

  private:

    configure() { };

    //****************************************************************************/
    // load
    //****************************************************************************/
    static void load(std::string filePath) {

      mpl::io::expandPath(filePath);

      params = dictionary_t();

      std::string tmpLine;

      std::ifstream infile;

      infile.open(filePath);

      if(infile.is_open()) {

        while(!infile.eof()) {

          getline(infile, tmpLine);

          parseLine(tmpLine);

        }

        infile.close();

      } else {
        fprintf(stderr, "mpl::configure::load() error: cannot open the configuration file \"%s\"\n", filePath.c_str());
        abort();
      }

      addKey("CONFIGURE_FILE_PATH", filePath);

    }

    //****************************************************************************/
    // trimSpace
    //****************************************************************************/
    static void trimSpace(const std::string & str, size_t & start, size_t & end){

      for(; end>=start; --end)
        if(isgraph(str[end]))
          break;

      ++end;

      for(; start<end; ++start)
        if(isgraph(str[start]))
          break;

    }

    //****************************************************************************/
    // parseLine
    //****************************************************************************/
    static void parseLine(std::string tmpLine) {

      if(tmpLine.empty()) return;

      if(tmpLine[0] == '#') return;

      if(tmpLine[0] == '@') return;

      size_t pos = tmpLine.find("=");

      if(pos == std::string::npos) return;

      size_t start_key = 0;
      size_t end_key = tmpLine.substr(0, pos - 1).length();

      trimSpace(tmpLine, start_key, end_key);

      size_t start_value = pos + 1;
      size_t end_value = tmpLine.length();

      trimSpace(tmpLine, start_value, end_value);

      params[tmpLine.substr(start_key, end_key - start_key)] = tmpLine.substr(start_value, end_value - start_value);

    }

  };

  dictionary_t configure::params = dictionary_t();

  //****************************************************************************/
  // specialization of the template function getParam()
  //****************************************************************************/
  template <>
  bool configure::getParam(const char * key) {

    dictionary_t::const_iterator itrKeys;

    if((itrKeys = params.find(key)) == params.end()) {

      fprintf(stderr, "mpl::configure::getParam() error: variable \"%s\" not defined\n", key);
      abort();

    } else {

      return (itrKeys->second.compare("ON") == 0);

    }

  }

  //****************************************************************************/
  // specialization of the template function getParam()
  //****************************************************************************/
  template <>
  const char* configure::getParam(const char * key) {

    dictionary_t::const_iterator itrKeys;

    if( (itrKeys = params.find(key)) == params.end()) {

      fprintf(stderr, "mpl::configure::getParam() error: variable \"%s\" not defined\n", key);
      abort();

    } else {

      return itrKeys->second.c_str();

    }

  }

  //****************************************************************************/
  // specialization of the template function getParam()
  //****************************************************************************/
  template <>
  cv::Point2d configure::getParam(const char * key) {

    dictionary_t::const_iterator itrKeys;

    if((itrKeys = params.find(key)) == params.end()) {

      fprintf(stderr, "mpl::configure::getParam() error: variable \"%s\" not defined\n", key);
      abort();

    } else {

      std::vector<std::string> tokens;

      std::parse(itrKeys->second, ",", tokens);

      if(tokens.size() != 2) {
        fprintf(stderr, "mpl::configure::getParam() error: cannot parse point config entry\n");
        abort();
      }

      return cv::Point2d(atof(tokens[0].c_str()),atof(tokens[1].c_str()));

    }

  }

} // namespace

#endif // _H_MPL_CORE_CONFIGURE_H_
