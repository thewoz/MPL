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

#ifndef _H_MPL_GETOPT_H_
#define _H_MPL_GETOPT_H_

#include <cstdlib>
#include <cstdio>

#include <iomanip>
#include <iostream>

//#include <set>
#include <vector>
#include <string>
#include <sstream>
//#include <unordered_map>

#include "stdlib.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // class opt
  /*****************************************************************************/
  class opt {
    
  private:
    
    /*****************************************************************************/
    // struct param_t
    /*****************************************************************************/
    struct param_t {
      
      std::string shortKey;
      
      std::string longKey;
      
      std::string name;
      
      std::string description;
      
      std::string defaultValue;
      
      std::string value;
      
      bool haveArgument;
      
      bool isMandatory;
      
      //opt_t() : shortKey(""), value(""), defaultValue("") { }
      
      param_t() { }
      
      /*****************************************************************************/
      // printHeader
      /*****************************************************************************/
      void printHeader(FILE * output = stderr) {
        
        if(!isMandatory) fprintf(output, "[");
        
        if(!shortKey.empty()) fprintf(output, "-%s", shortKey.c_str());
        else fprintf(output, "--%s", longKey.c_str());
        
        if(haveArgument) fprintf(output, " {%s}", name.c_str());
        
        if(!isMandatory) fprintf(output, "]");
        
        fprintf(output, " ");
        
      }
      
      /*****************************************************************************/
      // printInfo
      /*****************************************************************************/
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
      
      /*****************************************************************************/
      // isSet
      /*****************************************************************************/
      inline bool isSet() { return (!value.empty() || !defaultValue.empty()); }
      
      /*****************************************************************************/
      // operator ==
      /*****************************************************************************/
      inline bool operator == (const std::string & str) {
        
        if(shortKey == str || longKey == str) return true;
        
        return false;
        
      }
      
    };
    
    opt() { add("h help", "print this usage", NOT_HAVE_ARGUMENT, IS_NOT_MANDATORY); }
    
    static std::string name;
    
    static std::string shortDescription;

    static std::string description;
    
    static std::string version;
    
    static std::string credits;
    
    static std::string license;
    
    static std::vector<param_t> opts;
    
  public:
    
    //enum TYPE { FILE, INT, REAL, STR, CHAR, BOOL };
    
    enum { IS_NOT_MANDATORY,  IS_MANDATORY  };
    enum { NOT_HAVE_ARGUMENT, HAVE_ARGUMENT };
    
    
    static void addProgram(const std::string & _name, const std::string & _shortDescription = "", const std::string & _description = "") { name = _name;  shortDescription = _shortDescription; description = _description; }

    static void addProgramName(const std::string & _name) { name = _name; }
    static void addShortDescription(const std::string & _shortDescription) { shortDescription = _shortDescription; }
    static void addDescription(const std::string & _description) { description = _description; }
    
    static void addInfo(const std::string & _version, const std::string & _credits, const std::string & _license) { version = _version; credits = _credits; license = _license; }
    static void addVersion(const std::string & _version) { version = _version; }
    static void addCredits(const std::string & _credits) { credits = _credits; }
    static void addLicense(const std::string & _license) { license = _license; }

    /*****************************************************************************/
    // add
    /*****************************************************************************/
    static void add(const std::string & key, const std::string & shortInfo, bool haveArgument, bool isMandatory, const std::string & defaultValue = "") {
      
      add(key, shortInfo, "", haveArgument, isMandatory, defaultValue);
      
    }
    
    /*****************************************************************************/
    // add
    /*****************************************************************************/
    static bool add(const std::string & key, const std::string & name, const std::string & description, bool haveArgument, bool isMandatory, const std::string & defaultValue = "") {
      
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
    
    /*****************************************************************************/
    // init()
    /*****************************************************************************/
    static void init(int argc, char * const argv []) {
      
      bool status = true;
      
      bool argFound = false;
      
      std::string option;
      std::string value;
      
      for(std::size_t i=1; i<argc; ++i){
        
        if(strlen(argv[i]) > 1){
          
          if(argv[i][0] == '-'){
            
            argFound = true;
            
            if(argv[i][1] == '-'){
              
              option = std::string(&argv[i][2], strlen(&argv[i][2]));
              
            } else option = argv[i][1];
            
          } else if(argFound){
            
            value = argv[i];
            
            argFound = false;
            
            opt::param_t * optPtr = NULL;
            
            if((optPtr = find(option)) != NULL){
              
              optPtr->value = value;
              
            } else { status = false; fprintf(stderr, "error option '%s' not reconized\n", option.c_str()); }
            
          } else { status = false; fprintf(stderr, "error option '%s' not reconized\n", argv[i]); }
          
        } else { status = false; fprintf(stderr, "error option '%s' not reconized\n", argv[i]); }
        
      }
      
      if(!check() || !status) { usage(); abort(); }
      
      // TODO: eseguire h e v
      
    }
    
    /*****************************************************************************/
    // set()
    /*****************************************************************************/
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
    
    /*****************************************************************************/
    // set()
    /*****************************************************************************/
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
    
    /*****************************************************************************/
    // get()
    /*****************************************************************************/
    static std::string get(const std::string & key) {
      
      opt::param_t * optPtr = NULL;
      
      if((optPtr = find(key)) != NULL){
        
        if(optPtr->value.empty()) {
          
          if(optPtr->defaultValue.empty()) {
            fprintf(stderr, "error parameter '%s' is empty\n", key.c_str());
            abort();
          }

          return optPtr->defaultValue;
          
        }
        
        return optPtr->value;
        
      } else {
        fprintf(stderr, "error parameter '%s' not found\n", key.c_str());
        abort();
      }
      
      abort();
      
    }
    
    /*****************************************************************************/
    // get()
    /*****************************************************************************/
    template <class T>
    static T get(const std::string & key) {
      
      std::string tmp = get(key);
      
      std::stringstream iss(tmp);
      
      T value;
      
      iss >> value;
      
      return value;
      
    }
    
    
    /*****************************************************************************/
    // usage()
    /*****************************************************************************/
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
      
      if(!version.empty()) fprintf(output, "Version %s ", version.c_str());
      if(!credits.empty()) fprintf(output, "Credits %s ", credits.c_str());      
      if(!license.empty()) fprintf(output, "%s ", license.c_str());

      fprintf(output, "\n");

      //abort();
      
    }
    
    /*****************************************************************************/
    // getInfo() - print the information about an option
    /*****************************************************************************/
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
    
    /*****************************************************************************/
    // getDefault
    /*****************************************************************************/
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
    
    /*****************************************************************************/
    // find()
    /*****************************************************************************/
    static inline opt::param_t * find(const std::string & key) {
      
      for(std::size_t i=0; i<opts.size(); ++i){
        if(opts[i] == key) {
          return &opts[i];
        }
      }
      
      return NULL;
      
    }
    
    /*****************************************************************************/
    // check()
    /*****************************************************************************/
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
        
        abort();
        
      }
      
      return true;
      
    }
    
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
  template <>
  const char * opt::get(const std::string & key) {
    
    return get(key).c_str();
    
  }
  
  
  //****************************************************************************//
  // bool opt::get() - specialization
  //****************************************************************************//
  template <>
  bool opt::get(const std::string & key) {
    
    std::string value = get(key);
    
    if(value.compare("ON") == 0 || value.compare("true") == 0 || value.compare("0") != 0) return true;
    else return false;
    
  }
  
  
} /* namespace mpl */

#endif /* _H_MPL_GETOPT_H_ */
