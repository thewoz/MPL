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

#ifndef _H_MPL_PRINT_H_
#define _H_MPL_PRINT_H_

#include <cstdio>
#include <cstdlib>

#include <cerrno>
#include <cstring>
#include <cstdarg>

/*****************************************************************************/
// namespace
/*****************************************************************************/
namespace mpl {

  /*****************************************************************************/
  // log
  /*****************************************************************************/
  class log {
    
  private:
    
    static FILE * output;
    
    static bool onScreen;
    
    static int indentation;
    
    log();
    
  public:
    
    enum { ON, OFF };
    
    static FILE * getOutput();
    
    static void setOutputOnScreen(const bool & mode);
    
    static void init(const char * format, ...);
    
    static void msn(const char * format, ...);
    
    static void warning(const char * format, ...);
    
    static void error(const char * format, ...);
    
    static void flush();
    
    static void close();
    
    static void chapterStart(const char *format, ...);
    
    static void chapterEnd();
    
    static void increaseIndentation();
    
    static void decreaseIndentation();
    
  };


  //***************************************************************************************************//
  // setOutputOnScreen
  //***************************************************************************************************//
  void log::setOutputOnScreen(const bool & mode) { onScreen = mode; }

  
  //***************************************************************************************************//
  // getOutput
  //***************************************************************************************************//
  FILE * log::getOutput() { return output; }

  
  //***************************************************************************************************//
  // init
  //***************************************************************************************************//
  void log::init(const char * format, ...){
    
    char filename[PATH_MAX];
    
    va_list ap;
    
    va_start(ap, format);
    
    vsprintf(filename, format, ap);
    
    va_end(ap);
    
    if(output != NULL) fclose(output);
    
    output = fopen(filename, "w");
    
    if(output == NULL) {
      fprintf(stderr, "error in opening '%s': %s\n", filename, strerror(errno));
      exit(EXIT_FAILURE);
    }

  }


  //***************************************************************************************************//
  // flush
  //***************************************************************************************************//
  void log::flush() {
    
    fflush(stdout);
    
    fflush(stderr);

    if(output != NULL) fflush(output);
    
  }


  //***************************************************************************************************//
  // close
  //***************************************************************************************************//
  void log::close() { if(output != NULL) fclose(output); }


  //***************************************************************************************************//
  // msn
  //***************************************************************************************************//
  void log::msn(const char * format, ...) {
    
    if(onScreen == ON) {
      va_list ap;
      
      va_start(ap, format);
      
      vprintf(format, ap);
      
      flush();
      
      va_end(ap);
    }
    
    if(output != NULL) {
      va_list ap;
      
      va_start(ap, format);
      
      vfprintf(output, format, ap);
      
      flush();
      
      va_end(ap);
    }
    
  }


  //***************************************************************************************************//
  // warning
  //***************************************************************************************************//
  void log::warning(const char * format, ...) {
    
    va_list ap;
    
    va_start(ap, format);
    
    vfprintf(stderr, format, ap);
    
    flush();
    
    va_end(ap);
    
    if(output != NULL) {
      va_list ap;
      
      va_start(ap, format);
      
      vfprintf(output, format, ap);
      
      flush();
      
      va_end(ap);
    }

  }


  //***************************************************************************************************//
  // error
  //***************************************************************************************************//
  void log::error(const char * format, ...) {

    va_list ap;
    
    va_start(ap, format);
    
    vfprintf(stderr, format, ap);
    
    flush();
    
    va_end(ap);
    
    if(output != NULL) {
      va_list ap;
      
      va_start(ap, format);
      
      vfprintf(output, format, ap);
      
      flush();
      
      va_end(ap);
    }
    
    exit(EXIT_FAILURE);

  }
  
  //***************************************************************************************************//
  // chapterEnd
  //***************************************************************************************************//
  void log::chapterEnd() {
    
    indentation--;
    
    if(indentation<0)
      indentation = 0;
    
  }

  //***************************************************************************************************//
  // chapter
  //***************************************************************************************************//
  inline void log::chapterStart(const char *format, ...) {
    
    //FIXME: qua fa schifo

//    if(!inited){
//      fprintf(stderr, "error: the class mpl::log:: wasn't inited\n");
//      fflush(stderr);
//      exit(EXIT_FAILURE);
//    }
    
//    va_list ap;
//
//    va_start(ap, format);
//
//    vsprintf(str, format, ap);
//
//    va_end(ap);
//
//    //FIXME: qua fa schifo
//
//    for(int i=0; i<indentation; i++){
//      fprintf(stdout, "   ");
//      //fprintf(logFile, "   ");
//    }
//
//    msn(str);
//
//    indentation++;
    
  }

  //***************************************************************************************************//
  // decreaseIndentation
  //***************************************************************************************************//
  inline void log::decreaseIndentation() {
    
    indentation--;
    
    if(indentation<0)
      indentation = 0;
    
  }
  
  //***************************************************************************************************//
  // increaseIndentation
  //***************************************************************************************************//
  inline void log::increaseIndentation() {
    
    indentation++;
    
  }

  
  FILE * log::output = NULL;
  bool log::onScreen = true;
  int log::indentation = 0;

  
} /* namespace */


#endif /* _H_MPL_PRINT_H_ */
