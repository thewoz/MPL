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

#ifndef _H_COBBS_GETOPT_H_
#define _H_COBBS_GETOPT_H_

#include <cstdlib>
#include <cstdio>

#include <iomanip>
#include <iostream>

#include <set>
#include <vector>
#include <string>
#include <unordered_map>

#include "stdlib.hpp"

class getopt {
  
  enum TYPE { FILE, INT, REAL, STR, CHAR, BOOL };
  
private:
  
  /*****************************************************************************/
  // struct opt_t
  /*****************************************************************************/
  struct opt_t {
    
    std::string shortKey;
    
    std::string longKey;

    std::string name;
    
    std::string description;
    
    std::string defaultValue;

    std::string value;
    
    bool haveArgument;

    bool isMandatory;
    
    //opt_t() : shortKey(""), value(""), defaultValue("") { }
    
    opt_t() { }

    
    /*****************************************************************************/
    // printHeader
    /*****************************************************************************/
    void printHeader(FILE * output = stderr) {
      
      if(!shortKey.empty()) fprintf(output, "-%s", shortKey.c_str());
      else fprintf(output, "--%s", longKey.c_str());
        
      if(haveArgument) fprintf(output, " {%s}", name.c_str());
        
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
      
      if(!shortKey.empty()) fprintf(output, "-%s ", shortKey.c_str());
      if(!longKey.empty())  fprintf(output, "--%s", longKey.c_str());
      
      fprintf(output, "] %s\n", name.c_str());
      
      if(!description.empty()) fprintf(output, "\n\t\t%s\n", description.c_str());
      
      fprintf(output, "\n");
     
      
      
      
    }
    
    /*****************************************************************************/
    // printDefault
    /*****************************************************************************/
    void printDefault(FILE * output = stderr) {
      
    }
    
    inline bool isSet() { return (!value.empty() || !defaultValue.empty()); }
    
    
  private:
    
    /*****************************************************************************/
    // printKey
    /*****************************************************************************/
    void printKey(FILE * output = stderr) {
      
      //for(std::size_t i=0; i<key.size(); ++i){
      //  fprintf(output, "%s", key[i].c_str());
      //}
      
      
    }

  public:
    
    /*****************************************************************************/
    // operator ==
    /*****************************************************************************/
    inline bool operator == (const std::string & str) {
      
      if(shortKey == str || longKey == str) return true;
      
      return false;
      
    }

    
  };
  
  
private:
  
  getopt() { add("h help", "print this usage", NOT_HAVE_ARGUMENT, IS_NOT_MANDATORY); }
  
  static std::string programName;
  
  static std::vector<opt_t> opts;
  
public:
  
  enum { IS_NOT_MANDATORY,  IS_MANDATORY  };
  enum { NOT_HAVE_ARGUMENT, HAVE_ARGUMENT };
  
  
  static void program(const std::string & name) { programName = name; }
  
  
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
    
    opt_t opt;
    
    std::vector<std::string> keys;
    
    std::parse(key, " ", keys);
    
    for(std::size_t i=0; i<keys.size(); ++i)
      if(keys[i].length() == 1) opt.shortKey = keys[i];
      else opt.longKey = keys[i];
    
    if(find(key)){
      fprintf(stderr, "error parameter with the same keys already defined\n");
      return false;
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
  // init
  /*****************************************************************************/
  static void init(int argc, const char * argv[]) {
    
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
          
          opt_t * optPtr = NULL;
          
          if((optPtr = find(option)) != NULL){
            
            optPtr->value = value;
            
          } else { status = false; printf("error option '%s' not reconized\n", option.c_str()); }
          
        } else { status = false; printf("error option '%s' not reconized\n", argv[i]); }
        
      } else { status = false; printf("error option '%s' not reconized\n", argv[i]); }

    }
    
    if(!check() || !status) usage();
    
  }
  
  /*****************************************************************************/
  // set
  /*****************************************************************************/
  static bool set(std::string key, std::string value) {
    
    opt_t * optPtr = NULL;
    
    if((optPtr = find(key)) != NULL){
      optPtr->value = value;
      return true;
    } else {
      fprintf(stderr, "error parameter '%s' not found\n", key.c_str());
      return false;
    }
    
    return false;
    
  }
  
  /*****************************************************************************/
  // get
  /*****************************************************************************/
  static std::string get(std::string key) {
    
    opt_t * optPtr = NULL;
    
    if((optPtr = find(key)) != NULL){
      return optPtr->value;
    } else {
      fprintf(stderr, "error parameter '%s' not found\n", key.c_str());
      return "error parameter not found";
    }
    
    return "error parameter not found";
    
  }
  
  /*****************************************************************************/
  // usage
  /*****************************************************************************/
  static void usage(FILE * output = stderr) {
  
    //fprintf(stderr, "  Usage: Kali -p {points file} -P {pijk file} -e {external prjmat} -o {output file} [-m min frame] [-M max frame] [-h]\n");

    fprintf(output, "\nUsage: %s ", programName.c_str());
    
    for(std::size_t i=0; i<opts.size(); ++i)
      if(opts[i].isMandatory) opts[i].printHeader(output);
    
    fprintf(output, "\n\n");

    for(std::size_t i=0; i<opts.size(); ++i)
      opts[i].printInfo(output);
    
    fprintf(output, "\n");
    
    //abort();
    
  }
  
  /*****************************************************************************/
  // info
  /*****************************************************************************/
  static bool info(std::string key, FILE * output = stderr) {

    opt_t * optPtr = NULL;
    
    if((optPtr = find(key)) != NULL){
      optPtr->printInfo(output);
      return true;
    } else {
      fprintf(output, "error parameter '%s' not found\n", key.c_str());
      return false;
    }
    
    return false;
    
  }
  
  /*****************************************************************************/
  // getDefault
  /*****************************************************************************/
  static bool getDefault(std::string key, FILE * output = stderr) {
    
    opt_t * optPtr = NULL;
    
    if((optPtr = find(key)) != NULL){
      optPtr->printDefault(output);
      return true;
    } else {
      fprintf(output, "error parameter '%s' not found\n", key.c_str());
      return false;
    }
    
    return false;
    
  }
  
  
private:
  
  /*****************************************************************************/
  // find
  /*****************************************************************************/
  static inline opt_t * find(const std::string & key) {
  
    for(std::size_t i=0; i<opts.size(); ++i){
      if(opts[i] == key) {
        return &opts[i];
      }
    }
    
    return NULL;
    
  }
  
  /*****************************************************************************/
  // check
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
      
      return false;
      
    }
    
    return true;
    
  }
  
  
};


std::vector<getopt::opt_t> getopt::opts = std::vector< getopt::opt_t>();
std::string getopt::programName = "noname";


#endif /* _H_COBBS_GETOPT_H_ */
