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
