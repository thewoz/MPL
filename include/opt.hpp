/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017
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

#ifndef _H_MPL_GETOPT_H_
#define _H_MPL_GETOPT_H_

#include <cstdlib>
#include <cstdio>
#include <cstring>

#include <iomanip>
#include <iostream>

#include <vector>
#include <string>
#include <sstream>

#include <opencv2/opencv.hpp>

#include <mpl/stdlib.hpp>

//*****************************************************************************/
// namespace mpl
//*****************************************************************************/
namespace mpl {

  //*****************************************************************************/
  // class opt
  //*****************************************************************************/
  class opt {
    
  private:
    
    //*****************************************************************************/
    // struct param_t
    //*****************************************************************************/
    struct param_t {
      
      std::string shortKey;
      
      std::string longKey;
      
      std::string name;
      
      std::string description;
      
      std::string defaultValue;
      
      std::string value;
      
      int haveArgument;
      
      bool isMandatory;
      
      bool isInInput = false;
      
      bool _isActive = false;
      
      param_t() { }
      
      //*****************************************************************************/
      // printHeader
      //*****************************************************************************/
      void printHeader(FILE * output = stderr) {
        
        if(!isMandatory) fprintf(output, "[");
        
        if(!shortKey.empty()) fprintf(output, "-%s", shortKey.c_str());
        else fprintf(output, "--%s", longKey.c_str());
        
        if(haveArgument) fprintf(output, " {%s}", name.c_str());
        
        if(!isMandatory) fprintf(output, "]");
        
        fprintf(output, " ");
        
      }
      
      //*****************************************************************************/
      // printInfo
      //*****************************************************************************/
      void printInfo(FILE * output = stderr) {
        
        /*
         << "           -p FILE : points file     [points.dat]" << endl
         << "           -c FILE : config file     [config.dat]" << endl
         << "           -T FILE : 3focal file     [3focal.bin]" << endl
         << "           -P FILE : prjmat file     [prjmat.bin]" << endl
         << "           -e VAL  : epistripe              [5.0]" << endl
         << "           -t VAL  : triradius              [5.0]" << endl
         << "           -E ERR  : error type               [2]" << endl
         */
        
        std::cout << std::setw(10);
        
        fprintf(output, "\t[");
        
        if(!shortKey.empty()) fprintf(output, "-%s", shortKey.c_str());
        if(!shortKey.empty() && !longKey.empty()) fprintf(output, " ");
        if(!longKey.empty()) fprintf(output, "--%s", longKey.c_str());
        
        fprintf(output, "] %s", name.c_str());
        
        if(!defaultValue.empty()) fprintf(output, " | default %s", defaultValue.c_str());
        
        if(!description.empty()) fprintf(output, "\n\n\t\t%s", description.c_str());
        
        fprintf(output, "\n\n");
        
      }
      
      //*****************************************************************************/
      // isSet
      //*****************************************************************************/
      inline bool isSet() const { return (!value.empty() || !defaultValue.empty()); }
      
      //*****************************************************************************/
      // isDefined
      //*****************************************************************************/
      inline bool isDefined() const { return isInInput; }
      
      //*****************************************************************************/
      // isActive
      //*****************************************************************************/
      inline bool isActive() const { return _isActive; }
      
      //*****************************************************************************/
      // operator ==
      //*****************************************************************************/
      inline bool operator == (const std::string & str) {
        
        if(shortKey == str || longKey == str) return true;
        
        return false;
        
      }
      
    };
    
    
    //*****************************************************************************/
    // class opt
    //*****************************************************************************/
    opt() { add("h help", "print this usage", NONE_ARGUMENT, NO_MANDATORY); }
    
    static std::string name;
    
    static std::string shortDescription;
    
    static std::string description;
    
    static std::string version;
    
    static std::string credits;
    
    static std::string license;
    
    static std::vector<param_t> opts;
    
  public:
    
    enum { NO_MANDATORY, IS_MANDATORY };
    enum { NONE_ARGUMENT,  HAVE_ARGUMENT };
    
    static void addProgram(const std::string & _name, const std::string & _shortDescription = "", const std::string & _description = "") { name = _name;  shortDescription = _shortDescription; description = _description; }
    
    static void addProgramName(const std::string & _name) { name = _name; }
    static void addShortDescription(const std::string & _shortDescription) { shortDescription = _shortDescription; }
    static void addDescription(const std::string & _description) { description = _description; }
    
    static void addInfo(const std::string & _version, const std::string & _credits, const std::string & _license) { version = _version; credits = _credits; license = _license; }
    static void addVersion(const std::string & _version) { version = _version; }
    static void addCredits(const std::string & _credits) { credits = _credits; }
    static void addLicense(const std::string & _license) { license = _license; }
    
    //*****************************************************************************/
    // add
    //*****************************************************************************/
    static void add(const std::string & key, const std::string & shortInfo, int haveArgument, bool isMandatory, const std::string & defaultValue = "") {
      
      add(key, shortInfo, "", haveArgument, isMandatory, defaultValue);
      
    }
    
    //*****************************************************************************/
    // add
    //*****************************************************************************/
    static bool add(const std::string & key, const std::string & name, const std::string & description, int haveArgument, bool isMandatory, const std::string & defaultValue = "") {
      
      opt::param_t opt;
      
      std::vector<std::string> keys;
      
      std::parse(key, " ", keys);
      
      if(keys.size() > 2) {
        fprintf(stderr, "a parameter can be only one long and one short descriptor\n");
        abort();
      }
      
      for(std::size_t i=0; i<keys.size(); ++i)
        if(keys[i].length() == 1) opt.shortKey = keys[i];
        else opt.longKey = keys[i];
      
      if(find(key)){
        fprintf(stderr, "error parameter with the same keys already defined\n");
        abort();
      }
      
      opt.name         = name;
      opt.description  = description;
      opt.haveArgument = haveArgument;
      opt.isMandatory  = isMandatory;
      
      opt.defaultValue = defaultValue;
      
      opts.push_back(opt);
      
      return true;
      
    }

    private:

    //*****************************************************************************/
    // extractOption()
    //*****************************************************************************/
    static std::string extractOption(const char* arg) {
        std::string option;
        
        if(strlen(arg) > 0) {
          if (arg[0] == '-') {
            int skip = 1;
            if(strlen(arg) > 1) {
              if (arg[1] == '-') {
                skip = 2;
              }
            }    
            // estrae il nome dell'opzione
            option = std::string(&arg[skip], strlen(&arg[skip]));
          }
        }
        // vuoto se non è un'opzione e quindi è un possibile argomento
        return option;

    }

    //*****************************************************************************/
    // processArg()
    //*****************************************************************************/
    static void processArg(const char* arg, bool& waitForArgument, opt::param_t*& optPtr) {
    
      // se arg inizia con "-" o "--" extractOption ritorna l'opzione, altrimenti ritorna stringa vuota
      std::string option = extractOption(arg);
      if(strlen(arg) > 0){
        // se arg non è un'opzione e stavo aspettando per un argomento e l'opzione puntata da optPtr aveva bisogno di un argomento
        if(option.empty() && waitForArgument && optPtr->haveArgument == HAVE_ARGUMENT) {
            // arg diventa il valore dell'opzione
            optPtr->value = arg;
            waitForArgument = false;
            return;
        // se l'option restituita è un numero allora arg può essere un argomento (un numero negativo)
        } else if (!option.empty() && std::is_number(option)) {
            if(strlen(arg) > 1){
              //evito che un arg come "--34" sia visto come un valore
              if(arg[1] == '-'){
                fprintf(stderr, "error: the '%s' option was not recognised\n", arg);
                exit(EXIT_FAILURE);
              }
            }
            if(waitForArgument && optPtr->haveArgument == HAVE_ARGUMENT){
              optPtr->value = arg;
              waitForArgument = false;
              return;
            } else {
              fprintf(stderr, "error: the '%s' argument was not expected here\n", arg);
              exit(EXIT_FAILURE);
            }
        // se arg non è un'opzione ma non stavo aspettando un argomento
        } else if (option.empty() && !waitForArgument) {
            fprintf(stderr, "error: the '%s' argument was not expected here\n", arg);
            exit(EXIT_FAILURE);

        // se arg non è un'opzione ma non rientra nei casi precedenti qualcosa è andato storto
        } else if (option.empty()) {
            fprintf(stderr, "error: the '%s' argument was not expected\n", arg);
            exit(EXIT_FAILURE);
        }
      }

      // se arg è un'opzione provo a trovarla tra quelle predefinite
      optPtr = opt::find(option);
      // se la trovo
      if (optPtr != NULL) {
        if(optPtr->isInInput){
          fprintf(stderr, "error: the '%s' option has been set more than one time\n", option.c_str());
          exit(EXIT_FAILURE);
        }
          optPtr->isInInput = true;
          optPtr->_isActive = true;

          if (optPtr->haveArgument == NONE_ARGUMENT) {
              optPtr->value = "true";
              waitForArgument = false;
          } else {
              waitForArgument = true;
          }

          if(optPtr->shortKey == "h") { usage(); exit(EXIT_SUCCESS); }

          return;
      // se non la trovo
      } else {
          fprintf(stderr, "error: the '%s' option was not recognized\n", option.c_str());
          exit(EXIT_FAILURE);
      }

      return;
    }
    
    public:

    //*****************************************************************************/
    // init()
    //*****************************************************************************/
    static void init(int argc, const char * argv[]) {
            
      if(argc == 1)  { usage(); exit(EXIT_FAILURE); }
      
      std::string option;
      
      opt::param_t * optPtr = NULL;
      
      bool waitForArgument = false;
      
      // ciclo su tutti gli argomenti saltando il primo
      for(std::size_t i=1; i<argc; ++i) {
        // se non è vuoto
        if(strlen(argv[i]) > 0) {          
          processArg(argv[i], waitForArgument, optPtr);
        }
        
      } // for
      
      check();
      
    }
    
    //*****************************************************************************/
    // set()
    //*****************************************************************************/
    static bool set(const std::string & key, const std::string & value) {
      
      opt::param_t * optPtr = NULL;
      
      if((optPtr = find(key)) != NULL){
        optPtr->value = value;
        return true;
      } else {
        fprintf(stderr, "error parameter '%s' not found\n", key.c_str());
        abort();
      }
      
      return false;
      
    }
    
    //*****************************************************************************/
    // set()
    //*****************************************************************************/
    template <class T>
    static bool set(const std::string &  key, const T & value) {
      
      opt::param_t * optPtr = NULL;
      
      if((optPtr = find(key)) != NULL){
        
        std::stringstream iss;
        
        iss << value;
        
        optPtr->value = iss.str();
        
        return true;
        
      } else {
        fprintf(stderr, "error parameter '%s' not found\n", key.c_str());
        abort();
      }
      
      abort();
      
    }
    
    //*****************************************************************************/
    // get()
    //*****************************************************************************/
    static std::string get(const std::string & key) {
      
      opt::param_t * optPtr = NULL;
      
      if((optPtr = find(key)) != NULL) {
        
        if(optPtr->haveArgument == NONE_ARGUMENT) {
          fprintf(stderr, "error parameter '%s' have not argument\n", key.c_str());
          abort();
        }
        
        if(optPtr->value.empty()) {
          
          //          if(optPtr->defaultValue.empty()) {
          //            fprintf(stderr, "error parameter '%s' is empty\n", key.c_str());
          //            abort();
          //          }
          
          return optPtr->defaultValue;
          
        }
        
        return optPtr->value;
        
      } else {
        fprintf(stderr, "error parameter '%s' not found\n", key.c_str());
        abort();
      }
      
      abort();
      
    }
    
    //*****************************************************************************/
    // get()
    //*****************************************************************************/
    template <class T>
    static T get(const std::string & key) {
      
      std::string tmp = get(key);
      
      std::stringstream iss(tmp);
      
      T value;
      
      iss >> value;
      
      return value;
      
    }
    
    //*****************************************************************************/
    // getList()
    //*****************************************************************************/
    static std::vector<std::string> getList(const std::string & key) {
      
      std::vector<std::string> tokens;
      
      std::parse(mpl::opt::get(key), ",", tokens);
      
      return tokens;
      
    }
    
    //*****************************************************************************/
    // getInt()
    //*****************************************************************************/
    static int getInt(const std::string & key) { return get<int>(key); }
    
    //*****************************************************************************/
    // getReal()
    //*****************************************************************************/
    static double getReal(const std::string & key) { return get<double>(key); }
    
    //*****************************************************************************/
    // getListPairs()
    //*****************************************************************************/
    template <class T>
    static std::vector<T> getListPairs(const std::string & key) {
      
      std::vector<T> output;
      
      std::vector<std::string> tokens;
      
      std::parse(mpl::opt::get(key), " ", tokens);
      
      std::vector<std::string> coords;
      
      for(size_t i=0; i<tokens.size(); ++i) {
        
        if(tokens[i].empty()) continue;
        
        std::parse(tokens[i], ",", coords);
        
        output.push_back(T(atof(coords[0].c_str()),atof(coords[1].c_str())));
        
      }
      
      return output;
      
    }
    
    //****************************************************************************//
    // getRange()
    //****************************************************************************//
    static cv::Range getRange(const std::string & key) {
      
      std::vector<std::string> tokens;
      
      std::parse(mpl::opt::get(key), "-", tokens);
      
      if(tokens.size() != 2) {
        fprintf(stderr, "error in parse '%s' in opt::getRange()\n", key.c_str());
        abort();
      }
      
      return cv::Range(std::stoi(tokens[0]),std::stoi(tokens[1]));
      
    }
    
    //*****************************************************************************/
    // isDefined()
    //*****************************************************************************/
    static bool isDefined(const std::string & key) {
      
      const opt::param_t * optPtr = NULL;
      
      if((optPtr = find(key)) != NULL) {
        
        return optPtr->isDefined();
        
      } else { return false; }
      
    }
    
    //*****************************************************************************/
    // isActive()
    //*****************************************************************************/
    static bool isActive(const std::string & key) {
      
      const opt::param_t * optPtr = NULL;
      
      if((optPtr = find(key)) != NULL) {
        
        return optPtr->isActive();
        
      } else { return false; }
      
    }
    
    //*****************************************************************************/
    // isEqual()
    //*****************************************************************************/
    static bool isEqual(const std::string & key, const std::string & value) {
      
      const opt::param_t * optPtr = NULL;
      
      if((optPtr = find(key)) != NULL) {
        
        if(isDefined(key)) 
          return (optPtr->value.compare(value) == 0);
        else 
          return (optPtr->defaultValue.compare(value) == 0);
        
      } else { fprintf(stderr, "error parameter '%s' not found\n", key.c_str()); abort(); }
      
      return false;
      
    }
    
    //*****************************************************************************/
    // deactivate()
    //*****************************************************************************/
     static void deactivate(const std::string & key) {
      
      opt::param_t * optPtr = NULL;
      
      if((optPtr = find(key)) != NULL) {
        
        optPtr->_isActive = false;
        
      }
      
    }
    
    //*****************************************************************************/
    // usage()
    //*****************************************************************************/
    static void usage(FILE * output = stderr) {
      
      //fprintf(stderr, "  Usage: Kali -p {points file} -P {pijk file} -e {external prjmat} -o {output file} [-m min frame] [-M max frame] [-h]\n");
      
      fprintf(output, "\nUSAGE: %s", name.c_str());
      
      if(!shortDescription.empty()) fprintf(output, " - %s", shortDescription.c_str());
      
      fprintf(output, "\n\n\n");
      
      if(!description.empty()) fprintf(output, "DESCRIPTION:\n\n\t%s\n\n\n", description.c_str());
      
      fprintf(output, "SYNOPSIS:\n\n\t%s ", name.c_str());
      
      for(std::size_t i=0; i<opts.size(); ++i)
        if(opts[i].isMandatory) opts[i].printHeader(output);
      
      fprintf(output, "\n\n\n");
      
      fprintf(output, "OPTIONS:\n\n");
      
      fprintf(output, "The following options are available:\n\n");
      
      for(std::size_t i=0; i<opts.size(); ++i)
        opts[i].printInfo(output);
      
      fprintf(output, "\n");
      
      if(!version.empty()) fprintf(output, "Version: %s ", version.c_str());
      if(!credits.empty()) fprintf(output, "Credits: %s ", credits.c_str());
      if(!license.empty()) fprintf(output, "- %s ", license.c_str());
      
      fprintf(output, "\n\n");
      
      //abort();
      
    }
    
    //*****************************************************************************/
    // getInfo() - print the information about an option
    //*****************************************************************************/
    static bool getInfo(std::string key, ::FILE * output = stderr) {
      
      opt::param_t * optPtr = NULL;
      
      if((optPtr = find(key)) != NULL){
        optPtr->printInfo(output);
        return true;
      } else {
        fprintf(output, "error parameter '%s' not found\n", key.c_str());
        abort();
      }
      
      abort();
      
    }
    
    //*****************************************************************************/
    // getDefault
    //*****************************************************************************/
    //    static bool getDefault(std::string key, ::FILE * output = stderr) {
    //
    //        opt::data * optPtr = NULL;
    //
    //        if((optPtr = find(key)) != NULL){
    //          optPtr->printDefault(output);
    //          return true;
    //        } else {
    //          fprintf(output, "error parameter '%s' not found\n", key.c_str());
    //          return false;
    //        }
    //
    //        return false;
    //
    //      }
    
  private:
    
    //*****************************************************************************/
    // find()
    //*****************************************************************************/
    static inline opt::param_t * find(const std::string & key) {
      
      for(std::size_t i=0; i<opts.size(); ++i){
        if(opts[i] == key) {
          return &opts[i];
        }
      }
      
      return NULL;
      
    }
    
    //*****************************************************************************/
    // check()
    //*****************************************************************************/
    static bool check() {
      
      std::vector<std::size_t> isNotOk;
      
      for(std::size_t i=0; i<opts.size(); ++i){
        
        if(opts[i].isMandatory && !opts[i].isSet()){
          isNotOk.push_back(i);
        }
        
      }
      
      if(isNotOk.size() > 0){
        
        fprintf(stderr, "\nerror you forgot to specify:\n\n");
        
        for(std::size_t i=0; i<isNotOk.size(); ++i){
          fprintf(stderr, "\t");
          opts[isNotOk[i]].printHeader(stderr);
          fprintf(stderr, "\n");
        }
        
        fprintf(stderr, "\n");
        
        exit(EXIT_FAILURE);
        
      }
      
      return true;
      
    }
    
    static void finalize();
    
  }; /* class opt */


  std::vector<opt::param_t> opt::opts             = std::vector<opt::param_t>();
  std::string               opt::name             = "";
  std::string               opt::shortDescription = "";
  std::string               opt::description      = "";
  std::string               opt::version          = "";
  std::string               opt::credits          = "";
  std::string               opt::license          = "";


  //****************************************************************************//
  // const char * opt::get() - specialization
  //****************************************************************************//
  //  template <>
  //  const char * opt::get(const std::string & key) {
  //
  //    return get(key).c_str();
  //
  //  }


  //****************************************************************************//
  // bool opt::get() - specialization
  //****************************************************************************//
  template <>
  bool opt::get(const std::string & key) {
    
    std::string value = get(key);
    
    if(value.compare("ON") == 0 || value.compare("true") == 0 || value.compare("0") != 0) return true;
    else return false;
    
  }


  //****************************************************************************//
  // const cv::Size opt::get() - specialization
  //****************************************************************************//
  template <>
  cv::Size opt::get(const std::string & key) {
    
    std::vector<std::string> tokens;
    
    std::parse(mpl::opt::get(key), "x", tokens);
    
    if(tokens.size() != 2) {
      fprintf(stderr, "error in parse '%s' in opt::get(cv::Size)\n", key.c_str());
      abort();
    }
    
    return cv::Size(std::stoi(tokens[0]),std::stoi(tokens[1]));
    
  }

  } /* namespace mpl */

#endif /* _H_MPL_GETOPT_H_ */
